/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file mission.h
 *
 * Delivery mode to perform deliveries
 *
 * @author Brian Krentz <brian.krentz@gmail.com>
 */
 
#include <px4_posix.h>
#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <float.h>
#include <math.h>
#include <fcntl.h>

#include <drivers/drv_hrt.h>

#include <dataman/dataman.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <lib/mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission_result.h>

#include <commander/state_machine_helper.h>

#include <mavlink/mavlink_log.h>

#include "navigator.h"
#include "delivery.h"

#define DELAY_SIGMA	0.01f

Delivery::Delivery(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	mavlink_fd(0),
	_complete(false),
	_drop_alt(5.0),
	// safety({0}),
	// status({0}),
	// armed({0}),
	////////////////////////////////
	_param_onboard_enabled(this, "DEL_ONBOARD_EN", false),
	_param_takeoff_alt(this, "DEL_TAKEOFF_ALT", false),
	_param_dist_1wp(this, "DEL_DIST_1WP", false),
	_param_altmode(this, "DEL_ALTMODE", false),
	_param_yawmode(this, "DEL_YAWMODE", false),
	_onboard_mission({0}),
	_offboard_mission({0}),
	_current_onboard_mission_index(-1),
	_current_offboard_mission_index(-1),
	_need_takeoff(true),
	_takeoff(false),
	_mission_type(MISSION_TYPE_NONE),
	_inited(false),
	_home_inited(false),
	_missionFeasiblityChecker(),
	_min_current_sp_distance_xy(FLT_MAX),
	_mission_item_previous_alt(NAN),
  	_on_arrival_yaw(NAN),
	_distance_current_previous(0.0f),
	////////////////////////////////
	_rtl_state(RTL_STATE_NONE),
	_param_return_alt(this, "DEL_RETURN_ALT", false),
	_param_descend_alt(this, "DEL_DESCEND_ALT", false),
	_param_land_delay(this, "DEL_LAND_DELAY", false)
{
	/* load initial params */
	updateParams();
}

Delivery::~Delivery()
{
}

void
Delivery::on_inactive()
{
////////////////////////////////////
	if (_inited) {
		/* check if missions have changed so that feedback to ground station is given */
		bool onboard_updated = false;
		orb_check(_navigator->get_onboard_mission_sub(), &onboard_updated);
		if (onboard_updated) {
			update_onboard_mission();
		}

		bool offboard_updated = false;
		orb_check(_navigator->get_offboard_mission_sub(), &offboard_updated);
		if (offboard_updated) {
			update_offboard_mission();
		}

		/* check if the home position became valid in the meantime */
		if ((_mission_type == MISSION_TYPE_NONE || _mission_type == MISSION_TYPE_OFFBOARD) &&
			!_home_inited && _navigator->home_position_valid()) {

			dm_item_t dm_current = DM_KEY_WAYPOINTS_OFFBOARD(_offboard_mission.dataman_id);

			_navigator->get_mission_result()->valid = _missionFeasiblityChecker.checkMissionFeasible(_navigator->get_mavlink_fd(), _navigator->get_vstatus()->is_rotary_wing,
					dm_current, (size_t) _offboard_mission.count, _navigator->get_geofence(),
					_navigator->get_home_position()->alt, _navigator->home_position_valid(),
					_navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
					_param_dist_1wp.get(), _navigator->get_mission_result()->warning);

			_navigator->increment_mission_instance_count();
			_navigator->set_mission_result_updated();

			_home_inited = true;
		}

	} else {
		/* read mission topics on initialization */
		_inited = true;

		update_onboard_mission();
		update_offboard_mission();
	}

	/* require takeoff after non-loiter or landing */
	if (!_navigator->get_can_loiter_at_sp() || _navigator->get_vstatus()->condition_landed) {
		_need_takeoff = true;
	}

	/* reset RTL state only if setpoint moved */
	if (!_navigator->get_can_loiter_at_sp()) {
		_rtl_state = RTL_STATE_NONE;
	}
}

void
Delivery::on_activation()
{
	// change delivery_status to initial state
	// check conditions and acquire needed GPS info
	delivery_status = DELIV_PREFLIGHT;

	_rtl_state = RTL_STATE_CLIMB;
}

void
Delivery::on_active()
{
	//check for delivery_status and run through delivery routine
	switch(delivery_status){
		case DELIV_PREFLIGHT:
			set_delivery_items();
			advance_delivery();
			break;
		case DELIV_ENROUTE:
			set_delivery_items();
			to_destination();
			break;
		case DELIV_DROPOFF:
			set_delivery_items();
			activate_gripper();
			break;
		case DELIV_RETURN:
			set_delivery_items();
			return_home();
			break;
		case DELIV_DISARM:
			set_delivery_items();
			shutoff();
			break;
		case DELIV_COMPLETE:
		default:
			break;
	}
}

void
Delivery::to_destination()
{
	// set a mission to destination with takeoff enabled
	// Status = enroute ; change to Dropoff at completion

	/* check if missions have changed so that feedback to ground station is given */
	bool onboard_updated = false;
	orb_check(_navigator->get_onboard_mission_sub(), &onboard_updated);
	if (onboard_updated) {
		update_onboard_mission();
	}

	bool offboard_updated = false;
	orb_check(_navigator->get_offboard_mission_sub(), &offboard_updated);
	if (offboard_updated) {
		update_offboard_mission();
	}

	/* reset mission items if needed */
	if (onboard_updated || offboard_updated) {
		set_mission_items();
	}

	/* lets check if we reached the current mission item */
	if (_mission_type != MISSION_TYPE_NONE && is_mission_item_reached()) {
		set_mission_item_reached();
		if (_mission_item.autocontinue) {
			/* switch to next waypoint if 'autocontinue' flag set */
			advance_mission();
			set_mission_items();

		}

	} else if (_mission_type != MISSION_TYPE_NONE &&_param_altmode.get() == MISSION_ALTMODE_FOH) {
		altitude_sp_foh_update();
	} else {
		/* if waypoint position reached allow loiter on the setpoint */
		if (_waypoint_position_reached && _mission_item.nav_cmd != NAV_CMD_IDLE) {
			_navigator->set_can_loiter_at_sp(true);
			_complete = true;
		}
	}

	/* see if we need to update the current yaw heading for rotary wing types */
	if (_navigator->get_vstatus()->is_rotary_wing 
			&& _param_yawmode.get() != MISSION_YAWMODE_NONE
			&& _mission_type != MISSION_TYPE_NONE) {
		heading_sp_update();
	}

	if (is_mission_item_reached()) {
		_complete = true;
	}

	if (_complete) {
		// Update status now that travel to destination is complete
		advance_delivery();
		mavlink_log_critical(mavlink_fd, "Black Hawk at Location");
	}
}

void
Delivery::activate_gripper()
{
	// activate the gripper script to let down the object and release
	// status = enroute ; change to return at completion

	// the code for descent can be found in set_delivery_items

	// keep descending until _drop_alt reached
	_complete = is_mission_item_reached();

	if (_complete) {
		//Drop the item by activating the servo
		unload_package();

		// Update status now that dropoff is complete
		advance_delivery();
		mavlink_log_critical(mavlink_fd, "The Eagle Has Landed");
	}
}

void
Delivery::return_home()
{
	// turn on RTL with landing enabled
	// should halt/switch to loiter if collision_imminent is true
	// status = return ; change to complete

	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached()) {
		advance_rtl();
		set_rtl_item();
	}

	if (_rtl_state == RTL_STATE_LANDED) {
		_complete = true;
	}

	if (_complete) {
		// Update Status now that return is complete
		advance_delivery();
		mavlink_log_critical(mavlink_fd, "Black Hawk Has Nested");
	}
}

void
Delivery::shutoff()
{
	// Disarm the drone when it is done with the landing
	// look at commander 380
			// if (_navigator->get_vstatus()->condition_landed) {
			// 	int mavlink_fd_local = px4_open(MAVLINK_LOG_DEVICE, 0);
			// 	arm_disarm(false, mavlink_fd_local, "Delivery.cpp");
			// 	px4_close(mavlink_fd_local);
			// }	

	// Update status now that the copter is disarmed
	advance_delivery();
	mavlink_log_critical(mavlink_fd, "Black Hawk Sleeping");
}

void
Delivery::set_delivery_items()
{
	// set the parameters needed for getting to the destination
	if (delivery_status == DELIV_ENROUTE) {

		set_mission_items();

	} else if (delivery_status == DELIV_DROPOFF) {

		// setting the mission items for a descent to the _drop_alt
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

		/* make sure we have the latest params */
		updateParams();

		set_previous_pos_setpoint();
		_navigator->set_can_loiter_at_sp(false);

		// mission_item to descend to _drop_alt
		_mission_item.lat = _navigator->get_global_position()->lat;
		_mission_item.lon = _navigator->get_global_position()->lon;
		_mission_item.altitude_is_relative = false;
		// .altitude needs to be referenced to location altitude, not home altitude
		_mission_item.altitude = _navigator->get_home_position()->alt + _drop_alt;
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 10.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = false;
		_mission_item.origin = ORIGIN_ONBOARD;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "RTL: descend to %d m (%d m above home)",
		(int)(_mission_item.altitude),
		(int)(_mission_item.altitude - _navigator->get_home_position()->alt));

		// loiter at destination
		_navigator->set_can_loiter_at_sp(true);

		reset_mission_item_reached();

		/* convert mission item to current position setpoint and make it valid */
		mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
		pos_sp_triplet->next.valid = false;

		_navigator->set_position_setpoint_triplet_updated();

	} else if (delivery_status == DELIV_RETURN) {

		// setting the proper _rtl_state for current position
		if (_navigator->get_global_position()->alt < _navigator->get_home_position()->alt
				   + _param_return_alt.get()) {
			_rtl_state = RTL_STATE_CLIMB;

			/* otherwise go straight to return */
		} else {
			/* set altitude setpoint to current altitude */
			_rtl_state = RTL_STATE_RETURN;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = _navigator->get_global_position()->alt;
		}

		set_rtl_item();
	}

	_complete = false;
}

void
Delivery::arm_disarm(bool arm, const int mavlink_fd_local, const char *armedBy)
{
	// transition_result_t arming_res = TRANSITION_NOT_CHANGED;

	// // Transition the armed state. By passing mavlink_fd to arming_state_transition it will
	// // output appropriate error messages if the state cannot transition.
	// arming_res = arming_state_transition(&status, &safety, vehicle_status_s::ARMING_STATE_STANDBY, &armed,
	// 				     false , mavlink_fd_local);

	// if (arming_res == TRANSITION_CHANGED && mavlink_fd) {
	// 	mavlink_log_info(mavlink_fd_local, "[cmd] %s by %s", arm ? "ARMED" : "DISARMED", armedBy);

	// } else if (arming_res == TRANSITION_DENIED) {
	// 	mavlink_log_critical(mavlink_fd, "Cannot Disarm");
	// }
}

void
Delivery::advance_delivery()
{
	switch(delivery_status) {
	case DELIV_PREFLIGHT:
		delivery_status = DELIV_ENROUTE;
		break;
	case DELIV_ENROUTE:
		delivery_status = DELIV_DROPOFF;
		break;
	case DELIV_DROPOFF:
		delivery_status = DELIV_RETURN;
		break;
	case DELIV_RETURN:
		delivery_status = DELIV_DISARM;
		break;
	case DELIV_DISARM:
		delivery_status = DELIV_COMPLETE;
	case DELIV_COMPLETE:
	default:
		break;
	}
}

void
Delivery::unload_package()
{
	// TODO
}

///////////////////////////////

void
Delivery::update_onboard_mission()
{
	if (orb_copy(ORB_ID(onboard_mission), _navigator->get_onboard_mission_sub(), &_onboard_mission) == OK) {
		/* accept the current index set by the onboard mission if it is within bounds */
		if (_onboard_mission.current_seq >=0
		&& _onboard_mission.current_seq < (int)_onboard_mission.count) {
			_current_onboard_mission_index = _onboard_mission.current_seq;
		} else {
			/* if less WPs available, reset to first WP */
			if (_current_onboard_mission_index >= (int)_onboard_mission.count) {
				_current_onboard_mission_index = 0;
			/* if not initialized, set it to 0 */
			} else if (_current_onboard_mission_index < 0) {
				_current_onboard_mission_index = 0;
			}
			/* otherwise, just leave it */
		}

		// XXX check validity here as well
		_navigator->get_mission_result()->valid = true;
		_navigator->increment_mission_instance_count();
		_navigator->set_mission_result_updated();

	} else {
		_onboard_mission.count = 0;
		_onboard_mission.current_seq = 0;
		_current_onboard_mission_index = 0;
	}
}

void
Delivery::update_offboard_mission()
{
	bool failed = true;

	if (orb_copy(ORB_ID(offboard_mission), _navigator->get_offboard_mission_sub(), &_offboard_mission) == OK) {
		warnx("offboard mission updated: dataman_id=%d, count=%d, current_seq=%d", _offboard_mission.dataman_id, _offboard_mission.count, _offboard_mission.current_seq);
		/* determine current index */
		if (_offboard_mission.current_seq >= 0 && _offboard_mission.current_seq < (int)_offboard_mission.count) {
			_current_offboard_mission_index = _offboard_mission.current_seq;
		} else {
			/* if less items available, reset to first item */
			if (_current_offboard_mission_index >= (int)_offboard_mission.count) {
				_current_offboard_mission_index = 0;

			/* if not initialized, set it to 0 */
			} else if (_current_offboard_mission_index < 0) {
				_current_offboard_mission_index = 0;
			}
			/* otherwise, just leave it */
		}

		/* Check mission feasibility, for now do not handle the return value,
		 * however warnings are issued to the gcs via mavlink from inside the MissionFeasiblityChecker */
		dm_item_t dm_current = DM_KEY_WAYPOINTS_OFFBOARD(_offboard_mission.dataman_id);

		failed = !_missionFeasiblityChecker.checkMissionFeasible(_navigator->get_mavlink_fd(), _navigator->get_vstatus()->is_rotary_wing,
				dm_current, (size_t) _offboard_mission.count, _navigator->get_geofence(),
				_navigator->get_home_position()->alt, _navigator->home_position_valid(),
				_navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
				_param_dist_1wp.get(), _navigator->get_mission_result()->warning);

		_navigator->get_mission_result()->valid = !failed;
		_navigator->increment_mission_instance_count();
		_navigator->set_mission_result_updated();

	} else {
		PX4_WARN("offboard mission update failed, handle: %d", _navigator->get_offboard_mission_sub());
	}

	if (failed) {
		_offboard_mission.count = 0;
		_offboard_mission.current_seq = 0;
		_current_offboard_mission_index = 0;
	}

	set_current_offboard_mission_item();
}


void
Delivery::advance_mission()
{
	if (_takeoff) {
		_takeoff = false;

	} else {
		switch (_mission_type) {
		case MISSION_TYPE_ONBOARD:
			_current_onboard_mission_index++;
			break;

		case MISSION_TYPE_OFFBOARD:
			_current_offboard_mission_index++;
			break;

		case MISSION_TYPE_NONE:
		default:
			break;
		}
	}
}

int
Delivery::get_absolute_altitude_for_item(struct mission_item_s &mission_item)
{
	if (_mission_item.altitude_is_relative) {
		return _mission_item.altitude + _navigator->get_home_position()->alt;
	} else {
		return _mission_item.altitude;
	}
}

void
Delivery::set_mission_items()
{
	/* make sure param is up to date */
	updateParams();

	/* reset the altitude foh logic, if altitude foh is enabled (param) a new foh element starts now */
	altitude_sp_foh_reset();

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* set previous position setpoint to current */
	set_previous_pos_setpoint();

	/* Copy previous mission item altitude (can be extended to a copy of the full mission item if needed) */
	if (pos_sp_triplet->previous.valid) {
		_mission_item_previous_alt = get_absolute_altitude_for_item(_mission_item);
	}

	/* the home dist check provides user feedback, so we initialize it to this */
	bool user_feedback_done = false;

	/* try setting onboard mission item */
	if (_param_onboard_enabled.get() && read_mission_item(true, true, &_mission_item)) {
		/* if mission type changed, notify */
		if (_mission_type != MISSION_TYPE_ONBOARD) {
			mavlink_log_critical(_navigator->get_mavlink_fd(), "onboard mission now running");
			user_feedback_done = true;
		}
		_mission_type = MISSION_TYPE_ONBOARD;

	/* try setting offboard mission item */
	} else if (read_mission_item(false, true, &_mission_item)) {
		/* if mission type changed, notify */
		if (_mission_type != MISSION_TYPE_OFFBOARD) {
			mavlink_log_info(_navigator->get_mavlink_fd(), "offboard mission now running");
			user_feedback_done = true;
		}
		_mission_type = MISSION_TYPE_OFFBOARD;
	} else {
		/* no mission available or mission finished, switch to loiter */
		if (_mission_type != MISSION_TYPE_NONE) {
			/* https://en.wikipedia.org/wiki/Loiter_(aeronautics) */
			mavlink_log_critical(_navigator->get_mavlink_fd(), "mission finished, loitering");
			user_feedback_done = true;

			/* use last setpoint for loiter */
			_navigator->set_can_loiter_at_sp(true);

		}

		_mission_type = MISSION_TYPE_NONE;

		/* set loiter mission item and ensure that there is a minimum clearance from home */
		set_loiter_item(&_mission_item, _param_takeoff_alt.get());

		/* update position setpoint triplet  */
		pos_sp_triplet->previous.valid = false;
		mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
		pos_sp_triplet->next.valid = false;

		/* reuse setpoint for LOITER only if it's not IDLE */
		_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

		set_mission_finished();

		if (!user_feedback_done) {
			/* only tell users that we got no mission if there has not been any
			 * better, more specific feedback yet
			 * https://en.wikipedia.org/wiki/Loiter_(aeronautics)
			 */

			if (_navigator->get_vstatus()->condition_landed) {
				/* landed, refusing to take off without a mission */

				mavlink_log_critical(_navigator->get_mavlink_fd(), "no valid mission available, refusing takeoff");
			} else {
				mavlink_log_critical(_navigator->get_mavlink_fd(), "no valid mission available, loitering");
			}

			user_feedback_done = true;

		}

		_navigator->set_position_setpoint_triplet_updated();
		return;
	}

	if (pos_sp_triplet->current.valid) {
		_on_arrival_yaw = _mission_item.yaw;
	}

	/* do takeoff on first waypoint for rotary wing vehicles */
	if (_navigator->get_vstatus()->is_rotary_wing) {
		/* force takeoff if landed (additional protection) */
		if (!_takeoff && _navigator->get_vstatus()->condition_landed) {
			_need_takeoff = true;
		}

		/* new current mission item set, check if we need takeoff */
		if (_need_takeoff && (
				_mission_item.nav_cmd == NAV_CMD_TAKEOFF ||
				_mission_item.nav_cmd == NAV_CMD_WAYPOINT ||
				_mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
				_mission_item.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
				_mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
				_mission_item.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH)) {
			_takeoff = true;
			_need_takeoff = false;
		}
	}

	if (_takeoff) {
		/* do takeoff before going to setpoint */
		/* set mission item as next position setpoint */
		mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->next);

		/* calculate takeoff altitude */
		float takeoff_alt = get_absolute_altitude_for_item(_mission_item);

		/* takeoff to at least NAV_TAKEOFF_ALT above home/ground, even if first waypoint is lower */
		if (_navigator->get_vstatus()->condition_landed) {
			takeoff_alt = fmaxf(takeoff_alt, _navigator->get_global_position()->alt + _param_takeoff_alt.get());

		} else {
			takeoff_alt = fmaxf(takeoff_alt, _navigator->get_home_position()->alt + _param_takeoff_alt.get());
		}

		/* check if we already above takeoff altitude */
		if (_navigator->get_global_position()->alt < takeoff_alt) {
			mavlink_log_critical(_navigator->get_mavlink_fd(), "takeoff to %.1f meters above home", (double)(takeoff_alt - _navigator->get_home_position()->alt));

			_mission_item.nav_cmd = NAV_CMD_TAKEOFF;
			_mission_item.lat = _navigator->get_global_position()->lat;
			_mission_item.lon = _navigator->get_global_position()->lon;
			_mission_item.yaw = NAN;
			_mission_item.altitude = takeoff_alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.autocontinue = true;
			_mission_item.time_inside = 0;

			mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);

			_navigator->set_position_setpoint_triplet_updated();
			return;

		} else {
			/* skip takeoff */
			_takeoff = false;
		}
	}

	/* set current position setpoint from mission item */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);

	/* require takeoff after landing or idle */
	if (pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LAND || pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		_need_takeoff = true;
	}

	_navigator->set_can_loiter_at_sp(false);
	reset_mission_item_reached();

	if (_mission_type == MISSION_TYPE_OFFBOARD) {
		set_current_offboard_mission_item();
	}
	// TODO: report onboard mission item somehow

	if (_mission_item.autocontinue && _mission_item.time_inside <= 0.001f) {
		/* try to read next mission item */
		struct mission_item_s mission_item_next;

		if (read_mission_item(_mission_type == MISSION_TYPE_ONBOARD, false, &mission_item_next)) {
			/* got next mission item, update setpoint triplet */
			mission_item_to_position_setpoint(&mission_item_next, &pos_sp_triplet->next);
		} else {
			/* next mission item is not available */
			pos_sp_triplet->next.valid = false;
		}

	} else {
		/* vehicle will be paused on current waypoint, don't set next item */
		pos_sp_triplet->next.valid = false;
	}

	/* Save the distance between the current sp and the previous one */
	if (pos_sp_triplet->current.valid && pos_sp_triplet->previous.valid) {
		_distance_current_previous = get_distance_to_next_waypoint(pos_sp_triplet->current.lat,
				pos_sp_triplet->current.lon,
				pos_sp_triplet->previous.lat,
				pos_sp_triplet->previous.lon);
	}

	_navigator->set_position_setpoint_triplet_updated();
}

void
Delivery::heading_sp_update()
{
	if (_takeoff) {
		/* we don't want to be yawing during takeoff */
		return;
	}

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* Don't change setpoint if last and current waypoint are not valid */
	if (!pos_sp_triplet->previous.valid || !pos_sp_triplet->current.valid ||
			!PX4_ISFINITE(_on_arrival_yaw)) {
		return;
	}

	/* Don't do FOH for landing and takeoff waypoints, the ground may be near
	 * and the FW controller has a custom landing logic */
	if (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_TAKEOFF) {
		return;
	}

	/* set yaw angle for the waypoint iff a loiter time has been specified */
	if (_waypoint_position_reached && _mission_item.time_inside > 0.0f) {
		_mission_item.yaw = _on_arrival_yaw;
	/* always keep the front of the rotary wing pointing to the next waypoint */
	} else if (_param_yawmode.get() == MISSION_YAWMODE_FRONT_TO_WAYPOINT) {
		_mission_item.yaw = get_bearing_to_next_waypoint(
		        _navigator->get_global_position()->lat,
		        _navigator->get_global_position()->lon,
		        _mission_item.lat,
		        _mission_item.lon);
	/* always keep the back of the rotary wing pointing towards home */
	} else if (_param_yawmode.get() == MISSION_YAWMODE_FRONT_TO_HOME) {
		_mission_item.yaw = get_bearing_to_next_waypoint(
		        _navigator->get_global_position()->lat,
		        _navigator->get_global_position()->lon,
		        _navigator->get_home_position()->lat,
		        _navigator->get_home_position()->lon);
	/* always keep the back of the rotary wing pointing towards home */
	} else if (_param_yawmode.get() == MISSION_YAWMODE_BACK_TO_HOME) {
		_mission_item.yaw = _wrap_pi(get_bearing_to_next_waypoint(
		        _navigator->get_global_position()->lat,
		        _navigator->get_global_position()->lon,
		        _navigator->get_home_position()->lat,
		        _navigator->get_home_position()->lon) + M_PI_F);
	}

	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	_navigator->set_position_setpoint_triplet_updated();
}


void
Delivery::altitude_sp_foh_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* Don't change setpoint if last and current waypoint are not valid */
	if (!pos_sp_triplet->previous.valid || !pos_sp_triplet->current.valid ||
			!PX4_ISFINITE(_mission_item_previous_alt)) {
		return;
	}

	/* Do not try to find a solution if the last waypoint is inside the acceptance radius of the current one */
	if (_distance_current_previous - _navigator->get_acceptance_radius(_mission_item.acceptance_radius) < 0.0f) {
		return;
	}

	/* Don't do FOH for landing and takeoff waypoints, the ground may be near
	 * and the FW controller has a custom landing logic */
	if (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_TAKEOFF) {
		return;
	}


	/* Calculate distance to current waypoint */
	float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
			_navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

	/* Save distance to waypoint if it is the smallest ever achieved, however make sure that
	 * _min_current_sp_distance_xy is never larger than the distance between the current and the previous wp */
	_min_current_sp_distance_xy = math::min(math::min(d_current, _min_current_sp_distance_xy),
			_distance_current_previous);

	/* if the minimal distance is smaller then the acceptance radius, we should be at waypoint alt
	 * navigator will soon switch to the next waypoint item (if there is one) as soon as we reach this altitude */
	if (_min_current_sp_distance_xy < _navigator->get_acceptance_radius(_mission_item.acceptance_radius)) {
		pos_sp_triplet->current.alt = get_absolute_altitude_for_item(_mission_item);
	} else {
		/* update the altitude sp of the 'current' item in the sp triplet, but do not update the altitude sp
		* of the mission item as it is used to check if the mission item is reached
		* The setpoint is set linearly and such that the system reaches the current altitude at the acceptance
		* radius around the current waypoint
		**/
		float delta_alt = (get_absolute_altitude_for_item(_mission_item) - _mission_item_previous_alt);
		float grad = -delta_alt/(_distance_current_previous - _navigator->get_acceptance_radius(_mission_item.acceptance_radius));
		float a = _mission_item_previous_alt - grad * _distance_current_previous;
		pos_sp_triplet->current.alt = a + grad * _min_current_sp_distance_xy;

	}

	_navigator->set_position_setpoint_triplet_updated();
}

void
Delivery::altitude_sp_foh_reset()
{
	_min_current_sp_distance_xy = FLT_MAX;
}

bool
Delivery::read_mission_item(bool onboard, bool is_current, struct mission_item_s *mission_item)
{
	/* select onboard/offboard mission */
	int *mission_index_ptr;
	struct mission_s *mission;
	dm_item_t dm_item;
	int mission_index_next;

	if (onboard) {
		/* onboard mission */
		mission_index_next = _current_onboard_mission_index + 1;
		mission_index_ptr = is_current ? &_current_onboard_mission_index : &mission_index_next;

		mission = &_onboard_mission;

		dm_item = DM_KEY_WAYPOINTS_ONBOARD;

	} else {
		/* offboard mission */
		mission_index_next = _current_offboard_mission_index + 1;
		mission_index_ptr = is_current ? &_current_offboard_mission_index : &mission_index_next;

		mission = &_offboard_mission;

		dm_item = DM_KEY_WAYPOINTS_OFFBOARD(_offboard_mission.dataman_id);
	}

	/* Repeat this several times in case there are several DO JUMPS that we need to follow along, however, after
	 * 10 iterations we have to assume that the DO JUMPS are probably cycling and give up. */
	for (int i = 0; i < 10; i++) {

		if (*mission_index_ptr < 0 || *mission_index_ptr >= (int)mission->count) {
			/* mission item index out of bounds */
			return false;
		}

		const ssize_t len = sizeof(struct mission_item_s);

		/* read mission item to temp storage first to not overwrite current mission item if data damaged */
		struct mission_item_s mission_item_tmp;

		/* read mission item from datamanager */
		if (dm_read(dm_item, *mission_index_ptr, &mission_item_tmp, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			mavlink_log_critical(_navigator->get_mavlink_fd(),
			                     "ERROR waypoint could not be read");
			return false;
		}

		/* check for DO_JUMP item, and whether it hasn't not already been repeated enough times */
		if (mission_item_tmp.nav_cmd == NAV_CMD_DO_JUMP) {

			/* do DO_JUMP as many times as requested */
			if (mission_item_tmp.do_jump_current_count < mission_item_tmp.do_jump_repeat_count) {

				/* only raise the repeat count if this is for the current mission item
				* but not for the next mission item */
				if (is_current) {
					(mission_item_tmp.do_jump_current_count)++;
					/* save repeat count */
					if (dm_write(dm_item, *mission_index_ptr, DM_PERSIST_POWER_ON_RESET,
					    &mission_item_tmp, len) != len) {
						/* not supposed to happen unless the datamanager can't access the
						 * dataman */
						mavlink_log_critical(_navigator->get_mavlink_fd(),
								     "ERROR DO JUMP waypoint could not be written");
						return false;
					}
					report_do_jump_mission_changed(*mission_index_ptr,
								       mission_item_tmp.do_jump_repeat_count);
				}
				/* set new mission item index and repeat
				* we don't have to validate here, if it's invalid, we should realize this later .*/
				*mission_index_ptr = mission_item_tmp.do_jump_mission_index;

			} else {
				if (is_current) {
					mavlink_log_critical(_navigator->get_mavlink_fd(),
							     "DO JUMP repetitions completed");
				}
				/* no more DO_JUMPS, therefore just try to continue with next mission item */
				(*mission_index_ptr)++;
			}

		} else {
			/* if it's not a DO_JUMP, then we were successful */
			memcpy(mission_item, &mission_item_tmp, sizeof(struct mission_item_s));
			return true;
		}
	}

	/* we have given up, we don't want to cycle forever */
	mavlink_log_critical(_navigator->get_mavlink_fd(),
			     "ERROR DO JUMP is cycling, giving up");
	return false;
}

void
Delivery::save_offboard_mission_state()
{
	mission_s mission_state;

	/* lock MISSION_STATE item */
	dm_lock(DM_KEY_MISSION_STATE);

	/* read current state */
	int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

	if (read_res == sizeof(mission_s)) {
		/* data read successfully, check dataman ID and items count */
		if (mission_state.dataman_id == _offboard_mission.dataman_id && mission_state.count == _offboard_mission.count) {
			/* navigator may modify only sequence, write modified state only if it changed */
			if (mission_state.current_seq != _current_offboard_mission_index) {
				if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state, sizeof(mission_s)) != sizeof(mission_s)) {
					warnx("ERROR: can't save mission state");
					mavlink_log_critical(_navigator->get_mavlink_fd(), "ERROR: can't save mission state");
				}
			}
		}

	} else {
		/* invalid data, this must not happen and indicates error in offboard_mission publisher */
		mission_state.dataman_id = _offboard_mission.dataman_id;
		mission_state.count = _offboard_mission.count;
		mission_state.current_seq = _current_offboard_mission_index;

		warnx("ERROR: invalid mission state");
		mavlink_log_critical(_navigator->get_mavlink_fd(), "ERROR: invalid mission state");

		/* write modified state only if changed */
		if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state, sizeof(mission_s)) != sizeof(mission_s)) {
			warnx("ERROR: can't save mission state");
			mavlink_log_critical(_navigator->get_mavlink_fd(), "ERROR: can't save mission state");
		}
	}

	/* unlock MISSION_STATE item */
	dm_unlock(DM_KEY_MISSION_STATE);
}

void
Delivery::report_do_jump_mission_changed(int index, int do_jumps_remaining)
{
	/* inform about the change */
	_navigator->get_mission_result()->item_do_jump_changed = true;
	_navigator->get_mission_result()->item_changed_index = index;
	_navigator->get_mission_result()->item_do_jump_remaining = do_jumps_remaining;
	_navigator->set_mission_result_updated();
}

void
Delivery::set_mission_item_reached()
{
	_navigator->get_mission_result()->reached = true;
	_navigator->get_mission_result()->seq_reached = _current_offboard_mission_index;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();
}

void
Delivery::set_current_offboard_mission_item()
{
	_navigator->get_mission_result()->reached = false;
	_navigator->get_mission_result()->finished = false;
	_navigator->get_mission_result()->seq_current = _current_offboard_mission_index;
	_navigator->set_mission_result_updated();

	save_offboard_mission_state();
}

void
Delivery::set_mission_finished()
{
	_navigator->get_mission_result()->finished = true;
	_navigator->set_mission_result_updated();
}

///////////////////////////////

void
Delivery::set_rtl_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* make sure we have the latest params */
	updateParams();

	set_previous_pos_setpoint();
	_navigator->set_can_loiter_at_sp(false);

	switch (_rtl_state) {
	case RTL_STATE_CLIMB: {
		float climb_alt = _navigator->get_home_position()->alt + _param_return_alt.get();

		_mission_item.lat = _navigator->get_global_position()->lat;
		_mission_item.lon = _navigator->get_global_position()->lon;
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = climb_alt;
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "RTL: climb to %d m (%d m above home)",
			(int)(climb_alt),
			(int)(climb_alt - _navigator->get_home_position()->alt));
		break;
	}

	case RTL_STATE_RETURN: {
		_mission_item.lat = _navigator->get_home_position()->lat;
		_mission_item.lon = _navigator->get_home_position()->lon;
		 // don't change altitude

		 if (pos_sp_triplet->previous.valid) {
		 	/* if previous setpoint is valid then use it to calculate heading to home */
		 	_mission_item.yaw = get_bearing_to_next_waypoint(
		 	        pos_sp_triplet->previous.lat, pos_sp_triplet->previous.lon,
		 	        _mission_item.lat, _mission_item.lon);

		 } else {
		 	/* else use current position */
		 	_mission_item.yaw = get_bearing_to_next_waypoint(
		 	        _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
		 	        _mission_item.lat, _mission_item.lon);
		 }
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "RTL: return at %d m (%d m above home)",
			(int)(_mission_item.altitude),
			(int)(_mission_item.altitude - _navigator->get_home_position()->alt));
		break;
	}

	case RTL_STATE_DESCEND: {
		_mission_item.lat = _navigator->get_home_position()->lat;
		_mission_item.lon = _navigator->get_home_position()->lon;
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = _navigator->get_home_position()->alt + _param_descend_alt.get();
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = false;
		_mission_item.origin = ORIGIN_ONBOARD;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "RTL: descend to %d m (%d m above home)",
			(int)(_mission_item.altitude),
			(int)(_mission_item.altitude - _navigator->get_home_position()->alt));
		break;
	}

	case RTL_STATE_LOITER: {
		bool autoland = _param_land_delay.get() > -DELAY_SIGMA;

		_mission_item.lat = _navigator->get_home_position()->lat;
		_mission_item.lon = _navigator->get_home_position()->lon;
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = _navigator->get_home_position()->alt + _param_descend_alt.get();
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = autoland ? NAV_CMD_LOITER_TIME_LIMIT : NAV_CMD_LOITER_UNLIMITED;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = _param_land_delay.get() < 0.0f ? 0.0f : _param_land_delay.get();
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = autoland;
		_mission_item.origin = ORIGIN_ONBOARD;

		_navigator->set_can_loiter_at_sp(true);

		if (autoland) {
			mavlink_log_critical(_navigator->get_mavlink_fd(), "RTL: loiter %.1fs", (double)_mission_item.time_inside);

		} else {
			mavlink_log_critical(_navigator->get_mavlink_fd(), "RTL: completed, loiter");
		}
		break;
	}

	case RTL_STATE_LAND: {
		_mission_item.lat = _navigator->get_home_position()->lat;
		_mission_item.lon = _navigator->get_home_position()->lon;
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = _navigator->get_home_position()->alt;
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_LAND;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "RTL: land at home");
		break;
	}

	case RTL_STATE_LANDED: {
		_mission_item.lat = _navigator->get_home_position()->lat;
		_mission_item.lon = _navigator->get_home_position()->lon;
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = _navigator->get_home_position()->alt;
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_IDLE;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "RTL: completed, landed");
		break;
	}

	default:
		break;
	}

	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
Delivery::advance_rtl()
{
	switch (_rtl_state) {
	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_RETURN:
		_rtl_state = RTL_STATE_DESCEND;
		break;

	case RTL_STATE_DESCEND:
		/* only go to land if autoland is enabled */
		if (_param_land_delay.get() < -DELAY_SIGMA || _param_land_delay.get() > DELAY_SIGMA) {
			_rtl_state = RTL_STATE_LOITER;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}
		break;

	case RTL_STATE_LOITER:
		_rtl_state = RTL_STATE_LAND;
		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_LANDED;
		break;

	default:
		break;
	}
}