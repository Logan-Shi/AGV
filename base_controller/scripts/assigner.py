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
                           'TURNLEFT' : 8,
                           'PARKONE'  : 9,
                           'PARKTWO'  : 10,
                           'FINISH'   : 11}
        self.sim = 1 # 1 as sim
        self.indicator = 1
        self.lightStatusMsg = 0 # 0 as stop, 1 as right
        self.parkMsg = 0 # 1 as spot one, 2 as spot two
        self.state = 0
        if self.sim:
            self.start_turn_pose = Pose(Point(1.6,0,0),Quaternion(0,0,0,1)) 
            self.left_turn_pose = Pose(Point(0.69,-0.3,0),Quaternion(0,0,0.5287,0.8488))
            self.right_turn_pose = Pose(Point(0.69,0.3,0),Quaternion(0,0,-0.5287,0.8488))
            self.exit_left_pose = Pose(Point(-4.9,-0.9,0),Quaternion(0,0,-0.4,0.90))
            self.exit_right_pose = Pose(Point(-4.9,0.9,0),Quaternion(0,0,0.4,0.90))
            self.exit_turn_pose = Pose(Point(-5.3,0,0),Quaternion(0,0,0,1))
            self.straight_lane_pose = Pose(Point(-4.7,-3.5,0),Quaternion(0,0,1,0))
            self.straight_exit_pose = Pose(Point(0.2,-3.5,0),Quaternion(0,0,1,0))
            self.park_one_pose = Pose(Point(1.93,-1.95,0),Quaternion(0,0,0.7,0.7))
            self.park_two_pose = Pose(Point(3,-1.95,0),Quaternion(0,0,0.7,0.7))
        else:
            self.start_turn_pose = Pose(Point(4.9,-3.98,0),Quaternion(0,0,0.04,1)) 
            self.left_turn_pose = Pose(Point(3.75,-3.92,0),Quaternion(0,0,0.59,0.8))
            self.right_turn_pose = Pose(Point(6.68,-0.6,0),Quaternion(0,0,-0.5287,0.8488))
            self.exit_left_pose = Pose(Point(1.8,-2.5,0),Quaternion(0,0,-0.52,0.85))
            self.exit_right_pose = Pose(Point(11.11,-1.29,0),Quaternion(0,0,-0.56,0.83))
            self.exit_turn_pose = Pose(Point(1.27,-1.67,0),Quaternion(0,0,0,1))
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
        rospy.Subscriber('hilensData',UInt8,self.hilenDataCallback)
        self.isFrontMsg = UInt8()
        self.hilenData = UInt8()
        rospy.sleep(1)

    def isFrontCallback(self,msg):
        self.isFrontMsg = msg.data

    def hilenDataCallback(self,msg):
        flag = msg.data
        if flag:
            if flag == 3:
                self.lightStatusMsg = 1
                rospy.loginfo('hilens told me to go right')
            elif flag == 4:
                self.lightStatusMsg = 2
                rospy.loginfo('hilens told me to go left')
            elif flag == 1 or flag == 2:
                self.lightStatusMsg = 0
                rospy.loginfo('hilens told me to stop')
            elif flag == 5:
                self.parkMsg = 1
                rospy.loginfo('hilens told me to park at spot one')
            elif flag == 6:
                self.parkMsg = 2
                rospy.loginfo('hilens told me to park at spot two')

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
        rospy.loginfo('current pose = ' + str(self.pose.position.x) + ', ' + str(self.pose.position.y))
        rospy.loginfo("state = " + str(self.state))

    def waitforLight(self):
        if self.state == self.car_states['LIGHT']:
            lightStatus = self.lightStatusMsg
            if lightStatus == 0:
                rospy.loginfo('waiting for light signal...')
                self.stop()
                self.state = self.car_states['LIGHT']
            else :
                rospy.loginfo('signal received.')
                if lightStatus == 1:
                    rospy.loginfo('entering right')
                    self.indicator = 0
                    if self.isArrivedLineX(self.start_turn_pose):
                        self.state = self.car_states['RIGHT']
                    else:
                        self.state = self.car_states['LIGHT']
                else:
                    self.indicator = 1
                    rospy.loginfo('entering left')
                    if self.isArrivedLineX(self.start_turn_pose):
                        self.state = self.car_states['LEFT']
                    else:
                        self.state = self.car_states['LIGHT']
                
    def turnRight(self):
        if self.state == self.car_states['RIGHT']:
            self.rightCmd()
            rospy.loginfo('turning right')
            if self.isArrivedLineY(self.right_turn_pose):
                self.state = self.car_states['TURNRIGHT']
            else:
                self.state = self.car_states['RIGHT']
            # if not self.sendGoal(self.right_turn_pose):
            #     self.state = self.car_states['TURNRIGHT']
            # else:
            #     self.state = self.car_states['RIGHT']

    def turnLeft(self):
        if self.state == self.car_states['LEFT']:
            self.leftCmd()
            rospy.loginfo('turning left')
            if self.isArrivedLineY(self.left_turn_pose):
                self.state = self.car_states['TURNLEFT']
            else:
                self.state = self.car_states['LEFT']
            # if not self.sendGoal(self.left_turn_pose):
            #     self.state = self.car_states['TURNLEFT']
            # else:
            #     self.state = self.car_states['LEFT']

    def right(self):
        if self.state == self.car_states['TURNRIGHT']:
            rospy.loginfo('on the right')
            if self.isArrivedLineY(self.exit_right_pose):
                self.state = self.car_states['EXITTURN']
            else:
                self.state = self.car_states['TURNRIGHT']

    def left(self):
        if self.state == self.car_states['TURNLEFT']:
            rospy.loginfo('on the left')
            if self.isArrivedLineY(self.exit_left_pose):
                self.state = self.car_states['EXITTURN']
            else:
                self.state = self.car_states['TURNLEFT']

    def exitTurn(self):
        if self.state == self.car_states['EXITTURN']:
            rospy.loginfo('exiting turn')
            if self.indicator:
                self.leftCmd()
            else:
                self.rightCmd()
            if self.isArrivedLineX(self.exit_turn_pose):
                self.state = self.car_states['STRAIGHT']
            else:
                self.state = self.car_states['EXITTURN']
            # if not self.sendGoal(self.exit_turn_pose):
            #     self.state = self.car_states['STRAIGHT']
            # else:
            #     self.state = self.car_states['EXITTURN']

    def goStraight(self):
        if self.state == self.car_states['STRAIGHT']:
            if self.isArrivedLineX(self.straight_lane_pose):
                self.state = self.car_states['DYNAMIC']
            else:
                self.state = self.car_states['STRAIGHT']

    def isArrivedLineX(self,pose):
        isArrivedLineX = 0
        self.updatePose()
        if self.abs(self.pose.position.x - pose.position.x) < 0.1 and \
           self.abs(self.pose.position.y - pose.position.y) < 0.8:
            isArrivedLineX = 1
            rospy.loginfo('line reached!')
        else:
            rospy.loginfo('target x = ' + str(pose.position.x))
            rospy.loginfo('reaching line...')
        return isArrivedLineX

    def isArrivedLineY(self,pose):
        isArrivedLineY = 0
        self.updatePose()
        if self.abs(self.pose.position.y - pose.position.y) < 0.1 and \
           self.abs(self.pose.position.x - pose.position.x) < 0.8:
            isArrivedLineY = 1
            rospy.loginfo('line reached!')
        else:
            rospy.loginfo('target y = ' + str(pose.position.y))
            rospy.loginfo('reaching line...')
        return isArrivedLineY

    def isArrived(self,pose):
        isArrived = 0
        self.updatePose()
        if self.abs(self.pose.position.x - pose.position.x) < 0.2 and \
           self.abs(self.pose.position.y - pose.position.y) < 0.2:
            isArrived = 1
            rospy.loginfo('target reached!')
        else:
            rospy.loginfo('target pose = ' + str(pose.position.x) + ', ' + str(pose.position.y))
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
                if self.isArrivedLineX(self.straight_exit_pose):
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
            rospy.loginfo('parking...')
            park = self.parkMsg
            if park == 1:
                self.state = self.car_states['PARKONE']
            elif park == 2:
                self.state = self.car_states['PARKTWO']
            else:
                self.state = self.car_states['PARK']

    def parkOne(self):
        if self.state == self.car_states['PARKONE']:
            rospy.loginfo('park to spot one')
            parkingStatus = self.sendGoal(self.park_one_pose)
            if parkingStatus == 0:
                self.stop()
                rospy.loginfo('parked into spot one')
                self.state = self.car_states['FINISH']
            elif parkingStatus == 1:
                self.backup()
                rospy.loginfo('goal failed, auto parking')
                self.state = self.car_states['FINISH']
            else:
                self.state = self.car_states['PARKONE']

    def parkTwo(self):
        if self.state == self.car_states['PARKTWO']:
            rospy.loginfo('park to spot two')
            parkingStatus = self.sendGoal(self.park_two_pose)
            if parkingStatus == 0:
                self.stop()
                rospy.loginfo('parked into spot two')
                self.state = self.car_states['FINISH']
            elif parkingStatus == 1:
                self.backup()
                rospy.loginfo('goal failed, auto parking')
                self.state = self.car_states['FINISH']
            else:
                self.state = self.car_states['PARKTWO']

    def finish(self):
        if self.state == self.car_states['FINISH']:
            self.backup()
            rospy.loginfo('FINISH')
    
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
            self.parkOne()
            self.parkTwo()
            self.finish()
            self.rate.sleep()

    def stop(self):
        drive_msg = Twist()
        drive_msg.linear.x = 0.0
        drive_msg.angular.z = 0.0
        self.velPub.publish(drive_msg)

    def leftCmd(self):
        drive_msg = Twist()
        drive_msg.linear.x = 0.5
        drive_msg.angular.z = 0.4
        self.velPub.publish(drive_msg)

    def rightCmd(self):
        drive_msg = Twist()
        drive_msg.linear.x = 0.5
        drive_msg.angular.z = -0.4
        self.velPub.publish(drive_msg)

    def backup(self):
        drive_msg = Twist()
        if self.isFrontMsg == 2:
            drive_msg.linear.x = -0.2
            drive_msg.angular.z = 0.0
        else:
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
