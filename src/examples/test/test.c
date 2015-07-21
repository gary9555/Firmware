/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file test.c
 * Tester
 */

#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include "uORB/topics/test_uorb.h"
#include "uORB/topics/adc_prox.h"
#include "uORB/topics/offboard_control_mode.h"
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int test_main(int argc, char *argv[]);

int test_main(int argc, char *argv[])
{
	printf("Hello There!\n");

    // TESTING for ADC input
    int adc_prox_sub_fd = orb_subscribe(ORB_ID(adc_prox));
    orb_set_interval(adc_prox_sub_fd, 100);

    struct pollfd fds[] = {
        {.fd=adc_prox_sub_fd,   .events =POLLIN},
    };

    int error_counter = 0;

    for (int i = 0; i < 100; i++) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            printf("[adc_prox] Got no data within a second\n");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                printf("[px4_simple_app] ERROR return value from poll(): %d\n"
                       , poll_ret);
            }

            error_counter++;

        } else {
            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct adc_prox_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(adc_prox), adc_prox_sub_fd, &raw);
                printf("[adc_prox] Test data: %.4f cm sensor no.%d \n",
                       (double)raw.data,
                       (int)raw.prox_num);
            }


#ifdef yingjun
    int manual_sub_fd=orb_subscribe(ORB_ID(manual_control_setpoint));
    orb_set_interval(manual_sub_fd,1000);
/*
    // subscribe to sensor_combined topic
    int test_sub_fd = orb_subscribe(ORB_ID(test_uorb));
    orb_set_interval(test_sub_fd, 1000);

    // advertise attitude topic
    struct test_uorb_s t;
    (&t)->timestamp=10;
    (&t)->shuai=12;
    //memset(&t, 2, sizeof(t));
    orb_advert_t t_pub = orb_advertise(ORB_ID(test_uorb), &t);
    orb_publish(ORB_ID(test_uorb), t_pub, &t);
*/

    struct pollfd fds[] = {
     //   { .fd = test_sub_fd,   .events = POLLIN },
        {.fd=manual_sub_fd,   .events =POLLIN},

	};

	int error_counter = 0;

    for (int i = 0; i < 100; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
            printf("[test_app] Got no data within a second\n");

		} else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[px4_simple_app] ERROR return value from poll(): %d\n"
				       , poll_ret);
			}

			error_counter++;

		} else {
            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct manual_control_setpoint_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(manual_control_setpoint), manual_sub_fd, &raw);
                printf("[Test_uORB] Test data: %.4f  %.4f \n",
                       (double)raw.x,
                       (double)raw.y);
                       //(double)raw.accelerometer_m_s2[2]);

                /* set att and publish this information for other apps */
              //  t.timestamp = raw.timestamp;
                //t.shuai = raw.shuai;
            //	att.yaw = raw.accelerometer_m_s2[2];
           // orb_publish(ORB_ID(test_uorb), t_pub, &t);
            }

            /* there could be more file descriptors here, in the form like:
             * if (fds[1..n].revents & POLLIN) {}
             */
#endif





#ifdef shuai
            if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
                struct test_uorb_s raw;
				/* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(test_uorb), test_sub_fd, &raw);
                printf("[Test_uORB] Test data:\t%8.4f\t%8.4f\t\n",
                       (double)raw.timestamp,
                       (double)raw.shuai);
                       //(double)raw.accelerometer_m_s2[2]);

				/* set att and publish this information for other apps */
                t.timestamp = raw.timestamp;
                t.shuai = raw.shuai;
            //	att.yaw = raw.accelerometer_m_s2[2];
           // orb_publish(ORB_ID(test_uorb), t_pub, &t);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
#endif
		}
	}

	return 0;
}

