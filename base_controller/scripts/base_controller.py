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
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
import tf
import math
import matplotlib.pyplot as plt

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    # rospy.loginfo("v: " + str(v))
    # rospy.loginfo("omega: " + str(omega))
    # if omega == 0 :
    #     return 90
    # if abs(v) < 0.1 :
    #     if omega * v == 0 :
    #         return 90
    #     elif omega * v > 0 :
    #         return 180
    #     else :
    #         return 0
    # radius = v / omega
    # steering_angle = math.atan(wheelbase / radius)
    # data = steering_angle / 0.0098 + 90
    # rospy.loginfo("pre_servo: " + str(data))
    # return data
    data = omega * 188.5 + 90.55
    return data

class base_controller():
    def __init__(self, mode):
        self.servoCmdMsg = UInt8()
        self.motorSpdCmdMsg = UInt8()
        self.motorModeCmdMsg = UInt8()
        self.odomMsg = Odometry()
        self.servoCmdMsg.data = 90
        self.wheelbase = 0.58
        self.x = 0
        self.y = 0
        self.yaw = 0
        
        rospy.init_node('base_controller', anonymous=True)

        self.last_time = rospy.Time.now()

        self.servoCmdPub = rospy.Publisher('servoCmd', UInt8, queue_size=1)
        self.motorSpdCmdPub = rospy.Publisher('motorSpdCmd', UInt8, queue_size=1)
        self.motorModeCmdPub = rospy.Publisher('motorModeCmd', UInt8, queue_size=1)
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.tfPub = tf.TransformBroadcaster()
        self.min_speed = 0.3
        self.scale = 121.75

        self.servo_PID_init(0.02,0.03,0.05)

        if mode == "PID":
            self.KP = 0.60
            self.KI = 0.07
            self.KD = 0.05
            self.error = [0.0, 0.0, 0.0]
            rospy.Subscriber('cmd_vel', Twist, self.cmdPIDCallback)
        else: 
            rospy.Subscriber('cmd_vel', Twist, self.cmdCallback)
        rospy.Subscriber('rpm', Int16, self.rpmCallback)
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self._shutdown)
    
    def cmdCallback(self, msg):
        _servoCmdMsg = convert_trans_rot_vel_to_steering_angle(self.odomMsg.twist.twist.linear.x, msg.angular.z, self.wheelbase)
        _servoCmdMsg += self.servo_PID_update( _servoCmdMsg,self.servoCmdMsg.data)
        self.servoCmdMsg.data = min(max(0, _servoCmdMsg), 180)
        target_speed = msg.linear.x
        if target_speed:
            self.motorSpdCmdMsg.data = min(abs(target_speed * 60 / (0.18 * np.pi)), 255)
            #self.motorModeCmdMsg.data = 2 - np.sign(target_speed) 
            if(target_speed>0):
                self.motorModeCmdMsg.data = 1
            elif(target_speed<0):
                self.motorModeCmdMsg.data = 2
        else:
            self.stopMotor()

    def cmdPIDCallback(self, msg):
        _servoCmdMsg = convert_trans_rot_vel_to_steering_angle(self.odomMsg.twist.twist.linear.x, msg.angular.z, self.wheelbase)
        _servoCmdMsg += self.servo_PID_update( _servoCmdMsg,self.servoCmdMsg.data)
        # rospy.loginfo("_servoCmdMsg: " + str(_servoCmdMsg))
        self.servoCmdMsg.data = min(max(0, _servoCmdMsg), 180)    
        target_speed = max(self.min_speed,abs(msg.linear.x)) * np.sign(msg.linear.x)
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error[0] = target_speed - self.odomMsg.twist.twist.linear.x
        if target_speed:
            target_speed += self.KP*(self.error[0]-self.error[1])+self.KI*self.error[0] \
                +self.KD*(self.error[0]-2*self.error[1]+self.error[2])
            self.motorSpdCmdMsg.data = min(abs(target_speed * 60 / (0.18 * np.pi)), 255)
            #self.motorModeCmdMsg.data = 2 - np.sign(target_speed) 
            if(target_speed>0):
                self.motorModeCmdMsg.data = 1
            elif(target_speed<0):
                self.motorModeCmdMsg.data = 2
        else:
            self.stopMotor()


    def servo_PID_init(self,P,I,D):
        self.s_KP = P
        self.s_KI = I
        self.s_KD = D

        self.s_error = [0,0,0]
    
    def servo_PID_update(self,target,feedback):
        self.s_error[2] = self.s_error[1]
        self.s_error[1] = self.s_error[0]
        self.s_error[0] = target - feedback

        return (self.s_KP*(self.s_error[0]-self.s_error[1])+self.s_KI*self.s_error[0] \
                +self.s_KD*(self.s_error[0]-2*self.s_error[1]+self.s_error[2]))
    
    def rpmCallback(self, msg):
        # self.odomMsg.angular.z = (self.servoCmdMsg.data - 90) / 2
        # self.odomMsg.linear.x = msg.data * 0.09 * 2 * np.pi / 60
        if (msg.data > 0):
            gain = 1.0
        else :
            gain = 1.0
        current_speed = msg.data * gain * 0.09 * 2 * np.pi / 60
        current_steering_angle = 0.005279 * self.servoCmdMsg.data - 0.4782 
        current_angular_velocity = current_speed * math.tan(current_steering_angle) / self.wheelbase
        # rospy.loginfo("steering angle = " + str(current_steering_angle))
        # rospy.loginfo("omega = " + str(current_angular_velocity))

        # dt used to calc odom
        if(not self.last_time):
            self.last_time = rospy.Time.now()
        dt = (rospy.Time.now() - self.last_time).to_sec()
        self.last_time = rospy.Time.now()
        # spd orthogonal decomposition
        x_dot = current_speed # * math.cos(self.yaw)
        y_dot = current_speed * math.sin(self.yaw)

        # odom calculation
        self.x += x_dot * dt
        self.y += y_dot * dt
        self.yaw += current_angular_velocity * dt

        # Odom header msg
        self.odomMsg.header.frame_id = 'odom'
        self.odomMsg.header.stamp =  self.last_time
        self.odomMsg.child_frame_id = 'base_link'

        # Position
        self.odomMsg.pose.pose.position.x = self.x
        self.odomMsg.pose.pose.position.y = self.y
        self.odomMsg.pose.pose.orientation.z = math.sin(self.yaw/2)
        self.odomMsg.pose.pose.orientation.w = math.cos(self.yaw/2)
        
        # Velocity
        self.odomMsg.twist.twist.linear.x = current_speed
        self.odomMsg.twist.twist.angular.z = current_angular_velocity
        self.odomPub.publish(self.odomMsg)
        self.tfPub.sendTransform((self.x, self.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.yaw),
            self.last_time,
            'base_link',
            'odom'
        )

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
    try:
        mode = rospy.get_param('mode', 'PID')
        baseController = base_controller(mode)
        baseController.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("base controller stopped.")
    
