#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# cmd1.data=from 0 to 180
# cmd2.data=from 0 to 255
# cmd3.data=1 CW
# cmd3.data=2 CCW
# cmd3.data=0 brake
# cmd3.data=3 cut off power
# servo 0 is at right and 180 is at left

import rospy
import numpy as np

from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist


if __name__=="__main__":
    cmd1 = UInt8()
    cmd2 = UInt8()
    cmd3 = UInt8() 
    def callback(msg):
        cmd1.data = msg.angular.z*180.0/np.pi+90
        cmd2.data = abs(msg.linear.x) * 100
        cmd3.data = 2-np.sign(msg.linear.x) 
    rospy.init_node('arduino_test')
    pub1 = rospy.Publisher('servoCmd', UInt8, queue_size=1)
    pub2 = rospy.Publisher('motorSpdCmd', UInt8, queue_size=1)
    pub3 = rospy.Publisher('motorModeCmd', UInt8, queue_size=1)
    rospy.Subscriber("/turtlebot_teleop/cmd_vel", Twist, callback)
    rate = rospy.Rate(20)

    def _shutdown():
        cmd1.data = 90
        cmd2.data = 0
        cmd3.data = 0
        pub1.publish(cmd1)
        pub2.publish(cmd2)
        pub3.publish(cmd3)

    rospy.on_shutdown(_shutdown) 
    


    while not rospy.is_shutdown():
        pub1.publish(cmd1)
        pub2.publish(cmd2)
        pub3.publish(cmd3)
        rate.sleep()




