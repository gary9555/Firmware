#!/usr/bin/env python
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
PKG = 'px4'

import sys
import unittest
import rospy
import math

from numpy import linalg
import numpy as np

from px4.msg import vehicle_control_mode
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

class OffboardPosctlTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        rospy.Subscriber('px4_multicopter/vehicle_control_mode', vehicle_control_mode, self.vehicle_control_mode_callback)
        rospy.Subscriber("px4_multicopter/mavros/position/local", PoseStamped, self.position_callback)
        self.pubSpt = rospy.Publisher('px4_multicopter/mavros/setpoint/local_position', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        self.hasPos = False

    #
    # General callback functions used in tests
    #
    def position_callback(self, data):
        self.hasPos = True
        self.localPosition = data

    def vehicle_control_mode_callback(self, data):
        self.controlMode = data


    #
    # Helper methods
    #
    def is_at_position(self, x, y, z, offset):
        if(not self.hasPos):
            return False

        rospy.logdebug("current position %f, %f, %f" % (self.localPosition.pose.position.x, self.localPosition.pose.position.y, self.localPosition.pose.position.z))
        desired = np.array((x, y, z))
        pos = np.array((self.localPosition.pose.position.x, self.localPosition.pose.position.y, self.localPosition.pose.position.z))
        return linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        # set a position setpoint
        pos = PoseStamped()
        pos.header = Header() 
        pos.header.frame_id = "base_footprint"
        pos.header.stamp = rospy.Time.now()
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in X seconds?
        count = 0
        while(count < timeout):
            self.pubSpt.publish(pos)
            
            if(self.is_at_position(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z, 0.5)):
                break
            count = count + 1
            self.rate.sleep()

        self.assertTrue(count < timeout, "took too long to get to position")

    #
    # Test offboard POSCTL
    #
    def test_posctl(self):
        # prepare flight path assertion
        positions = (
            (0,0,0),
            (2,2,2),
            (2,-2,2),
            (-2,-2,2),
            (2,2,2))

        for i in range(0, len(positions)):
            self.reach_position(positions[i][0], positions[i][1], positions[i][2], 120)
        
        # does it hold the position for Y seconds?
        positionHeld = True
        count = 0
        timeout = 50
        while(count < timeout):
            if(not self.is_at_position(2, 2, 2, 0.5)):
                positionHeld = False
                break
            count = count + 1
            self.rate.sleep()

        self.assertTrue(count == timeout, "position could not be held")
    

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'posctl_test', OffboardPosctlTest)
    #unittest.main()