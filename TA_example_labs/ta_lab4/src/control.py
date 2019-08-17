#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header
import numpy as np
from threading import Thread #imsosorry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan

from ta_lab4.msg import Detection


Kp_angle = 1.0 # angle term
Kp_speed = 30.0  # speed term
Kd_speed = 1.0
HISTORY_SIZE = 5 # Size of the circular array for smoothing steering commands
PUBLISH_RATE = 20.0 # number of control commands to publish per second
SPEED = 2.0
MIN_SPEED = 0.1

ACCEPTABLE_DISTANCE_ERROR = 0.008

EPSILON = 0.000001

class CircularArray(object):
    """docstring for CircularArray"""
    def __init__(self, size):
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value):
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    def mean(self):
        return np.mean(self.arr[:self.num_els])

    def median(self):
        return np.median(self.arr[:self.num_els])

class Control():
    def __init__(self):

        
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped, queue_size =1 )
       
        self.sub = rospy.Subscriber("/object_detection", Detection, self.compute_pd_control, queue_size=1)

        self.last_error = None

        # computed control instructions
        self.control = None
        self.steering_hist = CircularArray(HISTORY_SIZE)
        self.deriv_hist = CircularArray(HISTORY_SIZE)

                
        # flag to indicate the first laser scan has been received
        self.received_data = False

        self.drive_thread = Thread(target=self.apply_control)
        self.drive_thread.start()

    
    def apply_control(self):
        while not rospy.is_shutdown():
            if self.control is None:
                print "No control data"
                rospy.sleep(0.5)
            else:
                self.steering_hist.append(self.control[0])
                # smoothed_steering = self.steering_hist.mean()
                smoothed_steering = self.steering_hist.median()

                # print "control: ", smoothed_steering, self.control[1]
                
                drive_msg_stamped = AckermannDriveStamped()
                drive_msg = AckermannDrive()
                drive_msg.speed = self.control[1]
                drive_msg.steering_angle = smoothed_steering
                drive_msg.acceleration = 0
                drive_msg.jerk = 0
                drive_msg.steering_angle_velocity = 0
                drive_msg_stamped.drive = drive_msg
                self.pub.publish(drive_msg_stamped)

                rospy.sleep(1.0/PUBLISH_RATE)

    def compute_pd_control(self, msg):
        # print "GOT MESSAGE"
        if not self.received_data: self.received_data = True
        angle_term = Kp_angle*msg.error_center

        this_error = {
            "time": rospy.get_time(),
            "size": msg.error_size
        }

        if not self.last_error == None:
            dt = this_error["time"] - self.last_error["time"]
            de = this_error["size"] - self.last_error["size"]
            de_over_dt = de / dt
        else:
            de_over_dt = 0.0

        deriv_term = de_over_dt
        self.deriv_hist.append(deriv_term)
        smoothed_deriv = self.deriv_hist.mean()

        if smoothed_deriv < 0.007:
            smoothed_deriv = 0.0

        # clip the distance term to above a threshold speed, only move if the error is unacceptable
        if np.abs(msg.error_size) < ACCEPTABLE_DISTANCE_ERROR:
            speed_term = 0.0
        else:
            distance_term = Kp_speed*msg.error_size

            # print "dist:", distance_term, "deriv:", smoothed_deriv, "driv_term:", smoothed_deriv * Kd_speed

            speed_term = distance_term + smoothed_deriv * Kd_speed
            if speed_term < 0:
                speed_term = np.clip(speed_term, -SPEED,-MIN_SPEED)
            else:
                speed_term = np.clip(speed_term, MIN_SPEED,SPEED)

        self.control =  (np.clip(angle_term, -0.3, 0.3), speed_term)
        self.last_error = this_error


            
if __name__=="__main__":
    rospy.init_node("visual_servo")
    vs = Control()
    rospy.spin()
