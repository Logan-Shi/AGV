#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, Twist
from std_msgs.msg import UInt8,Float32
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

class assigner():
    def __init__(self, arg):
        rospy.init_node('assigner_py')
        self.rate = rospy.Rate(10)
        self.counter = 0
        self.cnt = 0
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
        self.sim = 0 # 1 as sim
        self.indicator = 0 # 1 as left
        self.lightStatusMsg = 1 # 0 as stop, 1 as right
        self.parkMsg = 0 # 1 as spot one, 2 as spot two
        self.state = 0
        self.exit = 0
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
            self.start_turn_pose = Pose(Point(2.3,0.0,0),Quaternion(0,0,0.04,1)) 
            self.left_turn_pose = Pose(Point(3.5,0.08,0),Quaternion(0,0,0.59,0.8))
            self.right_turn_pose = Pose(Point(3.5,-0.1,0),Quaternion(0,0,-0.5287,0.8488))
            self.exit_left_pose = Pose(Point(6.3,-0.0,0),Quaternion(0,0,-0.52,0.85))
            self.exit_right_pose = Pose(Point(6.3,0,0),Quaternion(0,0,-0.56,0.83))
            self.exit_turn_pose = Pose(Point(5.5,0.0,0),Quaternion(0,0,0,1))
            self.straight_lane_pose = Pose(Point(15,1.3,0),Quaternion(0,0,1,0))
            self.straight_exit_pose = Pose(Point(21,1.2,0),Quaternion(0,0,1,0))
            self.park_one_pose = Pose(Point(1,4,0),Quaternion(0,0,1,0))
            self.park_two_pose = Pose(Point(1,2,0),Quaternion(0,0,1,0))
            self.park_one = Pose(Point(2.57,1.27,0),Quaternion(0,0,1,0))
            self.park_two = Pose(Point(2,1.27,0),Quaternion(0,0,1,0))
            self.exit_pose = Pose(Point(0,0,0),Quaternion(0,0,1,0))
        self.rightClear = 0
        self.leftClear = 0
        self.pose = Pose()
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.map_base_link_tf_listener = tf.TransformListener()
        self.velPub = rospy.Publisher("/cmd_vel_mux/input/assigner",\
                Twist, queue_size =1 )
        self.statePub = rospy.Publisher('assignerState', UInt8, queue_size=1)
        rospy.Subscriber('isFront', Twist, self.isFrontCallback)
        rospy.Subscriber('hilensData',UInt8,self.hilenDataCallback)
        self.isFrontMsg = UInt8()
        self.hilenData = UInt8()
        self.lastx = 0
        rospy.sleep(1)

    def isFrontCallback(self,msg):
        data = min(msg.linear.x, msg.linear.y)
        if data < 0.6: 
            self.isFrontMsg = 2
        elif data < 0.95:
            self.isFrontMsg = 1
        else:
            self.isFrontMsg = 0
        if msg.angular.y > 0.9:
            self.leftClear = 1
        else:
            self.leftClear = 0
        if msg.angular.x > 0.9:
            self.rightClear = 1
        else:
            self.rightClear = 0
        
        # rospy.loginfo('!!!!!!!!'+str(msg))
           

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
        self.map_base_link_tf_listener.waitForTransform('/base_link','/odom',rospy.Time(), rospy.Duration(0.5))
        (trans,rot) = self.map_base_link_tf_listener.lookupTransform('/odom','/base_link',rospy.Time(0))
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
            if self.isArrivedLineX(self.right_turn_pose):
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
            if self.counter > 10:
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
            self.statePublish(0)
            if self.isArrivedLineX(self.exit_right_pose):
                self.state = self.car_states['EXITTURN']
            else:
                self.state = self.car_states['TURNRIGHT']

    def left(self):
        if self.state == self.car_states['TURNLEFT']:
            rospy.loginfo('on the left')
            self.statePublish(0)
            if self.isArrivedLineX(self.exit_left_pose):
                self.state = self.car_states['EXITTURN']
            else:
                self.state = self.car_states['TURNLEFT']

    def exitTurn(self):
        if self.state == self.car_states['EXITTURN']:
            rospy.loginfo('exiting turn')
            rospy.loginfo(str(self.exit))
            if 0: #self.indicator:
                if self.rightClear:
                    self.cnt += 1
                    rospy.loginfo('right clear'+str(self.cnt))
                    if self.cnt > 3:
                        rospy.loginfo('left')
                        self.statePublish(1)
                        if not self.rightClear:
                            self.state = self.car_states['STRAIGHT']
            else:
                if self.leftClear:
                    self.exit = 1 
                    rospy.loginfo('left clear')
                    rospy.loginfo('right')
                    self.statePublish(2)
                # if self.isArrivedLineX(self.exit_turn_pose):
                if (not self.leftClear) and self.exit:
                    self.state = self.car_states['STRAIGHT']
      
            # if not self.sendGoal(self.exit_turn_pose):
            #     self.state = self.car_states['STRAIGHT']
            # else:
            #     self.state = self.car_states['EXITTURN']

    def goStraight(self):
        if self.state == self.car_states['STRAIGHT']:
            self.statePublish(0)
            if self.isArrivedLineX(self.straight_lane_pose):
                self.state = self.car_states['DYNAMIC']
            else:
                self.state = self.car_states['STRAIGHT']

    def isArrivedLineX(self,pose):
        isArrivedLineX = 0
        self.updatePose()
        if self.abs(self.pose.position.x - pose.position.x) < 0.1:
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
            if isArrivedLineX(self.park_one):
                self.leftCmdP()
                if isArrivedLineX(self.exit):
                    self.state = self.car_states['FINISH']
                    
    def parkTwo(self):
        if self.state == self.car_states['PARKTWO']:
            rospy.loginfo('park to spot two')
            if isArrivedLineX(self.park_two):
                self.leftCmdP()
                if isArrivedLineX(self.exit):
                    self.state = self.car_states['FINISH']

    def finish(self):
        if self.state == self.car_states['FINISH']:
            self.finish()
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
            self.rate.sleep()

    def statePublish(self,state):
        msg = UInt8()
        msg.data = state
        self.statePub.publish(msg)

    def stop(self):
        drive_msg = Twist()
        drive_msg.linear.x = 0.0
        drive_msg.angular.z = 0.0
        self.velPub.publish(drive_msg)

    def leftCmdP(self):
        drive_msg = Twist()
        drive_msg.linear.x = 0.7
        drive_msg.angular.z = 0.2
        self.velPub.publish(drive_msg)
        
    def rightCmdP(self): 
        drive_msg = Twist()
        drive_msg.linear.x = 0.7
        drive_msg.angular.z = 0.2
        self.velPub.publish(drive_msg)

    def leftCmd(self):
        self.counter += 1
        drive_msg = Twist()
        drive_msg.linear.x = 0.7
        drive_msg.angular.z = 0.39
        # self.velPub.publish(drive_msg)
        self.statePublish(1)

    def rightCmd(self):
        self.counter += 1
        drive_msg = Twist()
        drive_msg.linear.x = 0.7
        drive_msg.angular.z = -0.39
        # self.velPub.publish(drive_msg)
        self.statePublish(2)
        
    def backup(self):
        drive_msg = Twist()
        if self.isFrontMsg == 2:
            drive_msg.linear.x = -0.2
            drive_msg.angular.z = 0.0
        else:
            drive_msg.linear.x = 0.0
            drive_msg.angular.z = 0.0
        self.velPub.publish(drive_msg)
        
    def finish(self):
        pass
            
if __name__ == '__main__':
    try:
        mode = rospy.get_param('mode','0')
        assigner = assigner(mode)
        
        assigner.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation finished.")
