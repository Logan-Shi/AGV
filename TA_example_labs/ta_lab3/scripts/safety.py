#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from threading import Thread #imsosorry

import pdb
import numpy as np

"""
Author: Ariel Anders & Corey Walsh
This program implements a simple safety node based on laser
scan data.

# Single scan from a planar laser range-finder

Header header
# stamp: The acquisition time of the first ray in the scan.
# frame_id: The laser is assumed to spin around the positive Z axis
# (counterclockwise, if Z is up) with the zero angle forward along the x axis

float32 angle_min # start angle of the scan [rad]
float32 angle_max # end angle of the scan [rad]
float32 angle_increment # angular distance between measurements [rad]

float32 time_increment # time between measurements [seconds] - if your scanner
# is moving, this will be used in interpolating position of 3d points
float32 scan_time # time between scans [seconds]

float32 range_min # minimum range value [m]
float32 range_max # maximum range value [m]

float32[] ranges # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities # intensity data [device-specific units]. If your
# device does not provide intensities, please leave the array empty.

"""

MIN_FRONT_DIST = 1.0 # meters
FAN_ANGLE = 15.0 # angle that is considered the front
N_BINS = 19

class Safety():
    def __init__(self):
        self.received_data = None
        self.parsed_data = None

        self.angles = None
        self.bins = None
        self.averages = None

        self.sub = rospy.Subscriber("/scan", LaserScan, self.lidarCB, queue_size=1)
        self.pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety",\
                AckermannDriveStamped, queue_size =1 )
        self.thread = Thread(target=self.drive)
        self.thread.start()
        rospy.loginfo("safety node initialized")

    def drive(self):
        while not rospy.is_shutdown():
            if self.received_data is None or self.parsed_data is None:
                rospy.sleep(0.5)
                continue

            if np.any(self.parsed_data['front'][:,0] < MIN_FRONT_DIST):
                rospy.loginfo("stoping!")
                # this is overkill to specify the message, but it doesn't hurt
                # to be overly explicit
                drive_msg_stamped = AckermannDriveStamped()
                drive_msg = AckermannDrive()
                drive_msg.speed = 0.0
                drive_msg.steering_angle = 0.0
                drive_msg.acceleration = 0
                drive_msg.jerk = 0
                drive_msg.steering_angle_velocity = 0
                drive_msg_stamped.drive = drive_msg
                self.pub.publish(drive_msg_stamped)
            
            # don't spin too fast
            rospy.sleep(.1)

    def lidarCB(self, msg):
        # for performance, cache data that does not need to be recomputed on each iteration
        if not self.received_data:
            rospy.loginfo("success! first message received")
            # cache laser scanner angles
            self.angles = (np.arange(len(msg.ranges)) * msg.angle_increment) + msg.angle_min
            self.angles *= 180.0 / np.pi # convert to degrees
            
            # bins for chunking data
            self.bins = np.linspace(self.angles[0], self.angles[-1], num=N_BINS+1)
            
            # find center angle for each bin and set to second value of each row
            self.averages = np.zeros((N_BINS,3))
            self.averages[:,1] = (self.bins[:-1] + self.bins[1:]) / 2.0

            # bins for left, center, right data
            self.large_bins = np.array([-float('inf'), -1*FAN_ANGLE, FAN_ANGLE, float('inf')])
        
        values = np.array(msg.ranges)
        
        # filter out range values that are outside the bounds of the laser scanner
        ranges = values[(values > msg.range_min) & (values < msg.range_max)]
        angles = self.angles[(values > msg.range_min) & (values < msg.range_max)]

        # Compute average range for each bin 
        #   - this is a linear time algorithm which scans through all scan ranges 
        #     assigning each range to the correct bin for the corresponding angle
        #     works because the angles array is sorted
        self.averages[:,0] = 0
        self.averages[:,2] = 0
        bin_num = 0
        for i in xrange(angles.size):
            if angles[i] > self.bins[bin_num+1]:
                bin_num += 1
            self.averages[bin_num,0] += ranges[i] # add range to bin
            self.averages[bin_num,2] += 1 # number of elements per bin

        # compute the average value for each bin
        # first remove bins with no elements to avoid divide by zero errors
        averages = self.averages[self.averages[:,2]!= 0]
        averages[:,0] = averages[:,0] / averages[:,2]

        # get left, center, right data
        digits = np.digitize(averages[:,1], self.large_bins)
        result = { 
                'left': averages [digits==1, :2],
                'front': averages [digits==2, :2],
                'right': averages [digits==3, :2],
            }

        self.received_data = True
        self.parsed_data = result

if __name__=="__main__":
    rospy.init_node("lidar_safety")
    Safety()
    rospy.spin()

