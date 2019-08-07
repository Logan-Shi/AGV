#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header
import numpy as np
from threading import Thread #imsosorry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan
from scipy import signal, stats
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import Polygon, Point32, PolygonStamped

RIGHT = 'right'
LEFT  = 'left'

SHOW_VIS = False
FAN_ANGLE = np.pi/5.0
TARGET_DISTANCE = 1.0
MEDIAN_FILTER_SIZE=141
KP = 0.4 # distance term
KD = 0.3  # angle term
# KD = 0.5  # angle term
PUBLISH_LINE = True
HISTORY_SIZE = 5 # Size of the circular array for smoothing steering commands
PUBLISH_RATE = 20.0 # number of control commands to publish per second
SPEED = 1.0

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

# this is a convenient class for visualizing data while developing in Python
# once your code is done, you should instead use RViz to visualize important
# intermediate products as you see fit
class DynamicPlot():
    def initialize(self):
        plt.ion()
        #Set up plot
        self.fig = plt.figure(figsize=plt.figaspect(2.))
        
        self.ax0 = self.fig.add_subplot(2,1,1)

        self.laser_angular, = self.ax0.plot([],[], 'r.')
        self.laser_filtered, = self.ax0.plot([],[], 'b-')

        self.ax0.set_ylim(-1, 15)
        self.ax0.set_xlim(-np.pi, +np.pi)
        self.ax0.invert_xaxis()
        self.ax0.grid()

        self.ax1 = self.fig.add_subplot(2,1,2) 
        self.ax1.invert_xaxis()
        self.ax1.grid()
        self.laser_euclid, = self.ax1.plot([],[], '.')
        self.laser_regressed, = self.ax1.plot([],[], 'g')
        
        self.redraw()
        
    def redraw(self):
        #Need both of these in order to rescale
        self.ax0.relim()
        self.ax0.autoscale_view()
        
        self.ax1.relim()
        self.ax1.autoscale_view()
        
        #We need to draw *and* flush
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

class WallFollow():
    def __init__(self, direction):
        if direction not in [RIGHT, LEFT]:
            rospy.loginfo("incorect %s wall selected.  choose left or right")
            rospy.signal_shutdown()
        self.direction = direction

        if SHOW_VIS:
            self.viz = DynamicPlot()
            self.viz.initialize()
        
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped, queue_size =1 )
        self.sub = rospy.Subscriber("/scan", LaserScan, self.lidarCB, queue_size=1)
        
        if PUBLISH_LINE:
            self.line_pub = rospy.Publisher("/viz/line_fit", PolygonStamped, queue_size =1 )

        # computed control instructions
        self.control = None
        self.steering_hist = CircularArray(HISTORY_SIZE)

        # containers for laser scanner related data
        self.data = None
        self.xs = None
        self.ys = None
        self.m = 0
        self.c = 0
        
        # flag to indicate the first laser scan has been received
        self.received_data = False
        
        # cached constants
        self.min_angle = None
        self.max_angle = None
        self.direction_muliplier = 0
        self.laser_angles = None

        self.drive_thread = Thread(target=self.apply_control)
        self.drive_thread.start()

        if SHOW_VIS:
            while not rospy.is_shutdown():
                self.viz_loop()
                rospy.sleep(0.1)

    def publish_line(self):
        # find the two points that intersect between the fan angle lines and the found y=mx+c line
        x0 = self.c / (np.tan(FAN_ANGLE) - self.m)
        x1 = self.c / (-np.tan(FAN_ANGLE) - self.m)

        y0 = self.m*x0+self.c
        y1 = self.m*x1+self.c

        poly = Polygon()
        p0 = Point32()
        p0.y = x0
        p0.x = y0

        p1 = Point32()
        p1.y = x1
        p1.x = y1
        poly.points.append(p0)
        poly.points.append(p1)

        polyStamped = PolygonStamped()
        polyStamped.header.frame_id = "base_link"
        polyStamped.polygon = poly

        self.line_pub.publish(polyStamped)

    def drive_straight(self):
        while not rospy.is_shutdown():
            drive_msg_stamped = AckermannDriveStamped()
            drive_msg = AckermannDrive()
            drive_msg.speed = 2.0
            drive_msg.steering_angle = 0.0
            drive_msg.acceleration = 0
            drive_msg.jerk = 0
            drive_msg.steering_angle_velocity = 0
            drive_msg_stamped.drive = drive_msg
            self.pub.publish(drive_msg_stamped)

            # don't spin too fast
            rospy.sleep(1.0/PUBLISH_RATE)

    def apply_control(self):
        while not rospy.is_shutdown():
            if self.control is None:
                print "No control data"
                rospy.sleep(0.5)
            else:
                self.steering_hist.append(self.control[0])
                # smoothed_steering = self.steering_hist.mean()
                smoothed_steering = self.steering_hist.median()

                # print smoothed_steering, self.control[0]
                
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

    # given line parameters cached in the self object, compute the pid control
    def compute_pd_control(self):
        if self.received_data:
            # given the computed wall slope, compute theta, avoid divide by zero error
            if np.abs(self.m) < EPSILON:
                theta = np.pi / 2.0
                x_intercept = 0
            else:
                theta = np.arctan(1.0/self.m)
                # solve for y=0 in y=mx+c
                x_intercept = self.c / self.m

            # x axis is perp. to robot but not perpindicular to wall
            # cosine term solves for minimum distance to wall
            wall_dist = np.abs(np.cos(theta)*x_intercept)

            # control proportional to angular error and distance from wall
            distance_term = self.direction_muliplier * KP * (wall_dist - TARGET_DISTANCE)
            angle_term = KD * theta
            control = angle_term + distance_term
            # avoid turning too sharply
            self.control = (np.clip(control, -0.3, 0.3), SPEED)

    def fit_line(self):
        if self.received_data and self.xs.shape[0] > 0:
            # fit line to euclidean space laser data in the window of interest
            slope, intercept, r_val, p_val, std_err = stats.linregress(self.xs,self.ys)
            self.m = slope
            self.c = intercept
            
    # window the data, compute the line fit and associated control
    def lidarCB(self, msg):
        if not self.received_data:
            rospy.loginfo("success! first message received")

            # populate cached constants
            if self.direction == RIGHT:
                center_angle = -math.pi / 2
                self.direction_muliplier = -1
            else:
                center_angle = math.pi / 2
                self.direction_muliplier = 1

            self.min_angle = center_angle - FAN_ANGLE
            self.max_angle = center_angle + FAN_ANGLE
            self.laser_angles = (np.arange(len(msg.ranges)) * msg.angle_increment) + msg.angle_min

        self.data = msg.ranges
        values = np.array(msg.ranges)

        # remove out of range values
        ranges = values[(values > msg.range_min) & (values < msg.range_max)]
        angles = self.laser_angles[(values > msg.range_min) & (values < msg.range_max)]

        # apply median filter to clean outliers
        filtered_ranges = signal.medfilt(ranges, MEDIAN_FILTER_SIZE)

        # apply a window function to isolate values to the side of the car
        window = (angles > self.min_angle) & (angles < self.max_angle)
        filtered_ranges = filtered_ranges[window]
        filtered_angles = angles[window]

        # convert from polar to euclidean coordinate space
        self.ys = filtered_ranges * np.cos(filtered_angles)
        self.xs = filtered_ranges * np.sin(filtered_angles)

        self.fit_line()
        self.compute_pd_control()

        # filter lidar data to clean it up and remove outlisers
        self.received_data = True

        if PUBLISH_LINE:
            self.publish_line()

        if SHOW_VIS:
            # cache data for development visualization
            self.filtered_ranges = filtered_ranges
            self.filtered_angles = filtered_angles

    def viz_loop(self):
        if self.received_data == True:
            self.viz.laser_angular.set_data(self.laser_angles, self.data)
            self.viz.laser_filtered.set_data(self.filtered_angles, self.filtered_ranges)
            self.viz.laser_euclid.set_data(self.xs, self.ys)
            self.viz.laser_regressed.set_data(self.xs, self.m*self.xs+self.c)
            self.viz.redraw()

if __name__=="__main__":
    rospy.init_node("wall_follow")
    WallFollow(RIGHT)
    rospy.spin()