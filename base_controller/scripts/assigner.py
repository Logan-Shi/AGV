#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, Twist
from std_msgs.msg import UInt8
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

class assigner():
    def __init__(self, arg):
        rospy.init_node('assigner_py')
        self.sim = arg # 1 as sim
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self._shutdown)
        self.goal_states = ('PENDING', 'ACTIVE', 'PREEMPTED',  
                            'SUCCEEDED', 'ABORTED', 'REJECTED', 
                            'PREEMPTING', 'RECALLING', 'RECALLED', 
                            'LOST')
        self.car_states = {'LIGHT'    : 0,
                           'LEFT'     : 1,
                           'RIGHT'    : 2,
                           'EXITTURN' : 3,
                           'STRAIGHT' : 4,
                           'DYNAMIC'  : 5,
                           'PARK'     : 6,
                           'TURNRIGHT': 7,
                           'TURNLEFT' : 8}
        self.state = 0
        if arg:
            self.start_turn_pose = Pose(Point(2,0,0),Quaternion(0,0,0,1)) 
            self.left_turn_pose = Pose(Point(6.68,0.6,0),Quaternion(0,0,0.5287,0.8488))
            self.right_turn_pose = Pose(Point(6.68,-0.6,0),Quaternion(0,0,-0.5287,0.8488))
            self.exit_left_pose = Pose(Point(11.11,1.29,0),Quaternion(0,0,0.56,0.83))
            self.exit_right_pose = Pose(Point(11.11,-1.29,0),Quaternion(0,0,-0.56,0.83))
            self.exit_turn_pose = Pose(Point(12.6,0,0),Quaternion(0,0,0,1))
            self.straight_lane_pose = Pose(Point(13,4,0),Quaternion(0,0,1,0))
            self.straight_exit_pose = Pose(Point(7,4,0),Quaternion(0,0,1,0))
            self.park_one_pose = Pose(Point(1,4,0),Quaternion(0,0,1,0))
            self.park_two_pose = Pose(Point(1,2,0),Quaternion(0,0,1,0))
        else:
            self.start_turn_pose = Pose(Point(2,0,0),Quaternion(0,0,0,1)) 
            self.left_turn_pose = Pose(Point(6.68,0.6,0),Quaternion(0,0,0.5287,0.8488))
            self.right_turn_pose = Pose(Point(6.68,-0.6,0),Quaternion(0,0,-0.5287,0.8488))
            self.exit_left_pose = Pose(Point(11.11,1.29,0),Quaternion(0,0,0.56,0.83))
            self.exit_right_pose = Pose(Point(11.11,-1.29,0),Quaternion(0,0,-0.56,0.83))
            self.exit_turn_pose = Pose(Point(12.6,0,0),Quaternion(0,0,0,1))
            self.straight_lane_pose = Pose(Point(13,4,0),Quaternion(0,0,1,0))
            self.straight_exit_pose = Pose(Point(7,4,0),Quaternion(0,0,1,0))
            self.park_one_pose = Pose(Point(1,4,0),Quaternion(0,0,1,0))
            self.park_two_pose = Pose(Point(1,2,0),Quaternion(0,0,1,0))

        self.pose = Pose()
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.map_base_link_tf_listener = tf.TransformListener()
        self.velPub = rospy.Publisher("/cmd_vel_mux/input/assigner",\
                Twist, queue_size =1 )
        rospy.Subscriber('isFront', UInt8, self.isFrontCallback)
        self.isFrontMsg = UInt8()
        rospy.sleep(1)

    def isFrontCallback(self,msg):
        self.isFrontMsg = msg.data

    def sendGoal(self,pose):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        self.client.send_goal(goal)
        self.stop()

        return self.checkGoalStatus()
            
    def checkGoalStatus(self):
        if not self.client.wait_for_result(rospy.Duration(300)):
            rospy.loginfo("Timed out to achieve goal!")
            return 2
        else:
            state = self.client.get_state()
            if self.goal_states[state] == 'SUCCEEDED': 
                rospy.loginfo("Goal reached!")
                return 0
            else:
                rospy.loginfo("Goal failed:" + str(self.goal_states[state]))
                return 1

    def updatePose(self):
        self.map_base_link_tf_listener.waitForTransform('/base_link','/map',rospy.Time(), rospy.Duration(0.5))
        (trans,rot) = self.map_base_link_tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
        self.pose.position.x = trans[0]
        self.pose.position.y = trans[1]
        self.pose.position.z = trans[2]
        self.pose.orientation.x = rot[0]
        self.pose.orientation.y = rot[1]
        self.pose.orientation.z = rot[2]
        self.pose.orientation.w = rot[3]
        rospy.loginfo("pose = \n" + str(self.pose.position))

    def waitforLight(self):
        if self.state == self.car_states['LIGHT']:
            lightStatus = 1
            if lightStatus == 0:
                rospy.loginfo('waiting for light signal...')
                self.stop()
                self.state = self.car_states['LIGHT']
            else :
                rospy.loginfo('signal received.')
                if lightStatus == 1:
                    rospy.loginfo('turning right')
                    if self.isArrived(self.start_turn_pose):
                        self.state = self.car_states['RIGHT']
                    else:
                        self.state = self.car_states['LIGHT']
                else:
                    rospy.loginfo('turning left')
                    if self.isArrived(self.start_turn_pose):
                        self.state = self.car_states['LEFT']
                    else:
                        self.state = self.car_states['LIGHT']
                
    def turnRight(self):
        if self.state == self.car_states['RIGHT']:
            if not self.sendGoal(self.right_turn_pose):
                self.state = self.car_states['TURNRIGHT']
            else:
                self.state = self.car_states['RIGHT']

    def turnLeft(self):
        if self.state == self.car_states['LEFT']:
            if not self.sendGoal(self.left_turn_pose):
                self.state = self.car_states['TURNLEFT']
            else:
                self.state = self.car_states['LEFT']

    def right(self):
        if self.state == self.car_states['TURNRIGHT']:
            if self.isArrived(self.exit_right_pose):
                self.state = self.car_states['EXITTURN']
            else:
                self.state = self.car_states['TURNRIGHT']

    def left(self):
        if self.state == self.car_states['TURNLEFT']:
            if self.isArrived(self.exit_left_pose):
                self.state = self.car_states['EXITTURN']
            else:
                self.state = self.car_states['TURNLEFT']

    def exitTurn(self):
        if self.state == self.car_states['EXITTURN']:
            rospy.loginfo('exiting turn')
            if not self.sendGoal(self.exit_turn_pose):
                self.state = self.car_states['STRAIGHT']
            else:
                self.state = self.car_states['EXITTURN']

    def goStraight(self):
        if self.state == self.car_states['STRAIGHT']:
            if self.isArrived(self.straight_lane_pose):
                self.state = self.car_states['DYNAMIC']
            else:
                self.state = self.car_states['STRAIGHT']

    def isArrived(self,pose):
        isArrived = 0
        self.updatePose()
        if self.abs(self.pose.position.x - pose.position.x) < 0.2 and \
           self.abs(self.pose.position.y - pose.position.y) < 0.5:
            isArrived = 1
            rospy.loginfo('target reached!')
        else:
            rospy.loginfo('approaching target...')
        return isArrived

    def waitforDynamicObj(self):
        if self.state == self.car_states['DYNAMIC']: 
            rospy.loginfo('waiting for dynamic obj')
            isFront = self.isFrontMsg
            if isFront:
                rospy.loginfo('pedestrian detected, stopping...')
                self.stop()
                self.state = self.car_states['DYNAMIC']
            else:
                if self.isArrived(self.straight_exit_pose):
                    rospy.loginfo('pedestrian crossing passed!')
                    self.state = self.car_states['PARK']
                else:
                    self.state = self.car_states['DYNAMIC']

    def abs(self,a):
        if a>0:
            return a
        else:
            return -a

    def park(self):
        if self.state == self.car_states['PARK']:
            park = 1
            if park == 1:
                if not self.sendGoal(self.park_one_pose):
                    rospy.loginfo('parked into spot one')
                else:
                    self.state == self.car_states['PARK']
            else:
                if not self.sendGoal(self.park_two_pose):
                    rospy.loginfo('parked into spot two')
                else:
                    self.state == self.car_states['PARK']
    
    def _shutdown(self):
        self.client.cancel_goal()
        self.stop()

    def spin(self):
        while not rospy.is_shutdown():
            self.waitforLight()
            self.turnLeft()
            self.left()
            self.turnRight()
            self.right()
            self.exitTurn()
            self.goStraight()
            self.waitforDynamicObj()
            self.park()
            self.rate.sleep()

    def stop(self):
        drive_msg = Twist()
        drive_msg.linear.x = 0.0
        drive_msg.angular.z = 0.0
        self.velPub.publish(drive_msg)

if __name__ == '__main__':
    try:
        mode = rospy.get_param('mode','0')
        assigner = assigner(mode)
        assigner.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation finished.")