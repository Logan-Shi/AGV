#!/usr/bin/env python

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
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

class base_controller():
    def __init__(self, mode):
        self.servoCmdMsg = UInt8()
        self.motorSpdCmdMsg = UInt8()
        self.motorModeCmdMsg = UInt8()
        self.odomMsg = Twist()
        self.servoCmdMsg.data = 90

        rospy.init_node('base_controller', anonymous=True)
        self.servoCmdPub = rospy.Publisher('servoCmd', UInt8, queue_size=1)
        self.motorSpdCmdPub = rospy.Publisher('motorSpdCmd', UInt8, queue_size=1)
        self.motorModeCmdPub = rospy.Publisher('motorModeCmd', UInt8, queue_size=1)
        self.odomPub = rospy.Publisher('odom', Twist, queue_size=1)

        self.scale = 121.75
        if mode == "PID":
            self.KP = 1
            self.KI = 1
            self.KD = 1
            self.error = [0, 0, 0]
            rospy.Subscriber('cmd_vel', Twist, self.cmdPIDCallback)
        else: 
            rospy.Subscriber('cmd_vel', Twist, self.cmdCallback)
        rospy.Subscriber('rpm', Int16, self.rpmCallback)
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self._shutdown)
    
    def cmdCallback(self, msg):
        _servoCmdMsg = msg.angular.z * 2 + 90
        self.servoCmdMsg.data = min(max(0, _servoCmdMsg), 180)
        _motorSpdCmdMsg = msg.linear.x
        if _motorSpdCmdMsg:
            self.motorSpdCmdMsg.data = min(abs(_motorSpdCmdMsg) * self.scale, 255)
            self.motorModeCmdMsg.data = 2 - np.sign(_motorSpdCmdMsg) 
        else:
            self.stopMotor()

    def cmdPIDCallback(self, msg):
        _servoCmdMsg = msg.angular.z * 2 + 90
        self.servoCmdMsg.data = min(max(0, _servoCmdMsg), 180)
        _motorSpdCmdMsg = msg.linear.x
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error[0] = _motorSpdCmdMsg - self.odomMsg.linear.x
        if _motorSpdCmdMsg:
            _motorSpdCmdMsg *= self.scale
            _motorSpdCmdMsg += self.KP*(self.error[0]-self.error[1])+self.KI*self.error[0] \
              +self.KD*(self.error[0]-2*self.error[1]+self.error[2])
            self.motorSpdCmdMsg.data = min(abs(_motorSpdCmdMsg), 255)
            self.motorModeCmdMsg.data = 2 - np.sign(_motorSpdCmdMsg) 
        else:
            self.stopMotor()

    def rpmCallback(self, msg):
        self.odomMsg.angular.z = (self.servoCmdMsg.data - 90) / 2
        self.odomMsg.linear.x = msg.data * 0.09 * 2 * np.pi / 60   
        self.odomPub.publish(self.odomMsg)     

    def stopMotor(self):
        self.motorSpdCmdMsg.data = 0
        self.motorModeCmdMsg.data = 0
    
    def _shutdown(self):
        self.servoCmdMsg.data = 90
        self.stopMotor()
        self.servoCmdPub.publish(self.servoCmdMsg)
        self.motorSpdCmdPub.publish(self.motorSpdCmdMsg)
        self.motorModeCmdPub.publish(self.motorModeCmdMsg)

    
    def spin(self):
        while not rospy.is_shutdown():
            self.servoCmdPub.publish(self.servoCmdMsg)
            self.motorSpdCmdPub.publish(self.motorSpdCmdMsg)
            self.motorModeCmdPub.publish(self.motorModeCmdMsg)
            self.rate.sleep()
        
if __name__=="__main__":
    mode = rospy.get_param('mode', 'simple')
    baseController = base_controller(mode)
    baseController.spin()
