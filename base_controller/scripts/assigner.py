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
        self.velPub = rospy.Publisher("/cmd_vel_mux/input/assigner", Twist, queue_size =1 )
        self.statePub = rospy.Publisher('assignerState', UInt8, queue_size=1)
        rospy.Subscriber('isFront', Twist, self.laserDataCallback)
        rospy.Subscriber('hilensData',UInt8,self.hilensDataCallback)
        rospy.Subscriber('cmd_vel',Twist,self.servoCallback)

        self.car_states = {'LIGHT'    : 0,
                           'LEFT'     : 1,
                           'RIGHT'    : 2,
                           'ENTERTURN': 3,
                           'EXITTURN' : 4,
                           'STRAIGHT' : 5,
                           'DYNAMIC'  : 6,
                           'PARK'     : 7,
                           'PARKONE'  : 8,
                           'PARKTWO'  : 9,
                           'GORIGHT'  : 10,
                           'GOLEFT'   : 11}
        self.sim = 0 # 1 as sim
        self.indicator = 2 # 1 as left
        self.lightStatusMsg = 1 # 0 as stop, 1 as right
        self.parkMsg = 0 # 0 as spot one, 1 as spot two
        self.sideWayClearWindow = 0
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
            self.start_turn_pose = Pose(Point(1.7,0.0,0),Quaternion(0,0,0.04,1)) 
            self.left_turn_pose = Pose(Point(2.5,0.08,0),Quaternion(0,0,0.59,0.8))
            self.right_turn_pose = Pose(Point(2.5,-0.1,0),Quaternion(0,0,-0.5287,0.8488))
            self.exit_left_pose = Pose(Point(4.8,-0.0,0),Quaternion(0,0,-0.52,0.85))
            self.exit_right_pose = Pose(Point(4.8,0,0),Quaternion(0,0,-0.56,0.83))
            self.exit_turn_pose = Pose(Point(5.5,0.0,0),Quaternion(0,0,0,1))
            self.straight_lane_pose = Pose(Point(15,1.3,0),Quaternion(0,0,1,0))
            self.straight_exit_pose = Pose(Point(21,1.2,0),Quaternion(0,0,1,0))
            self.park_one_pose = Pose(Point(1,4,0),Quaternion(0,0,1,0))
            self.park_two_pose = Pose(Point(1,2,0),Quaternion(0,0,1,0))
            self.park_one = Pose(Point(2.57,1.27,0),Quaternion(0,0,1,0))
            self.park_two = Pose(Point(2,1.27,0),Quaternion(0,0,1,0))
            self.exit_pose = Pose(Point(0,0,0),Quaternion(0,0,1,0))


        self.flag_list=[]
        self.flag_list_laser=[]
        rospy.on_shutdown(self._shutdown)

        self.pose = Pose()
        self.map_base_link_tf_listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.goal_states = ('PENDING', 'ACTIVE', 'PREEMPTED',  
                            'SUCCEEDED', 'ABORTED', 'REJECTED', 
                            'PREEMPTING', 'RECALLING', 'RECALLED', 
                            'LOST')

        self.sideWayClearCnt = 0
        self.rightClearMsg = 0
        self.leftClearMsg = 0
        self.isFrontMsg = 0
        self.headingMsg = 'straight'
        self.straightflag = 0
        self.no_signal_bufsize = 0

    def servoCallback(self,msg):
        turn_reference = 0.01
        
        if (msg.angular.z > turn_reference):
            self.headingMsg = 'left'
        elif (msg.angular.z < - turn_reference):
            self.headingMsg = 'right'
        else:
            self.headingMsg = 'straight'
            
    def laserDataCallback(self,msg):
        frontDisFilterSize = 3
        frontDis = min(msg.linear.x, msg.linear.y)
        self.flag_list_laser.append(frontDis)
        flag_list_reverse = list(reversed(self.flag_list_laser))
        flag_list_front = flag_list_reverse[0:frontDisFilterSize]

        count_num_front = []
        for item in flag_list_front:
            count_num_front.append(flag_list_front.count(item))
            max_pos = count_num_front.index(max(count_num_front))
        frontDis = flag_list_front[max_pos]

        parkingStopDis = 0.45
        dynamicStopDis = 0.5
        
        if frontDis < parkingStopDis: 
            self.isFrontMsg = 2 # less than parkingStopDis
        elif frontDis < dynamicStopDis:
            self.isFrontMsg = 1 # less than dynamicStopDis
        else:
            self.isFrontMsg = 0 # not gonna stop
        
        sideWayClearDis = 0.6
        sideWayFurtherClearDis = 1.0
        
        if msg.angular.y > sideWayClearDis:
            self.leftClearMsg = 1 # 'left clear, more than sideWayClearDis'
            if msg.angular.y > sideWayFurtherClearDis:
                self.leftClearMsg = 2 # 'left clear, more than sideWayFurtherClearDis'
        else:
            self.leftClearMsg = 0 # 'left blocked'

        if msg.angular.x > sideWayClearDis:
            self.rightClearMsg = 1 # 'right clear, more than sideWayClearDis'
            if msg.angular.x > sideWayFurtherClearDis:
                self.rightClearMsg = 2 # 'right clear, more than sideWayFurtherClearDis'
        else:
            self.rightClearMsg = 0 # 'right blocked'

    def hilensDataCallback(self,msg):
        lightFilterSize = 4
        parkFilterSize = 8

        self.flag_list.append(msg.data)
        flag_list_reverse = list(reversed(self.flag_list))
        flag_list_light = flag_list_reverse[0:lightFilterSize]
        flag_list_park = flag_list_reverse[0:parkFilterSize]

        count_num_light = []
        for item in flag_list_light:
            count_num_light.append(flag_list_light.count(item))
            max_pos = count_num_light.index(max(count_num_light))
        flag_light = flag_list_light[max_pos]
        
        count_num_park = []
        for item in flag_list_park:
            count_num_park.append(flag_list_park.count(item))
            max_pos = count_num_park.index(max(count_num_park))
        flag_park = flag_list_park[max_pos]
        
        if flag_light:
            if flag_light == 3:
                self.lightStatusMsg = 1
                rospy.loginfo('hilens told me to go right')
            elif flag_light == 4:
                self.lightStatusMsg = 2
                rospy.loginfo('hilens told me to go left')
            elif flag_light == 1 or flag_light == 2:
                self.lightStatusMsg = 0
                rospy.loginfo('hilens told me to stop')
        
        if self.state == self.car_states['STRAIGHT']:
            if flag_park == 5:
                self.parkMsg = 1
                self.straightflag = 1
                rospy.loginfo('hilens told me to park at spot one')
            elif flag_park == 6:
                self.parkMsg = 2
                self.straightflag = 1
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
        rospy.loginfo('current position = ' + str(self.pose.position.x) + ', ' + str(self.pose.position.y))
    
    def showState(self):
        rospy.loginfo("state = " + str(self.state))

    def isArrived(self,target_pose,x_window,y_window):
        isArrived = 0
        self.updatePose()
        dist = hypot(self.pose.position.x - target_pose.position.x, self.pose.position.y - target_pose.position.y)
        if dist < 0.4:
            isArrived = 1
            rospy.loginfo('target reached!')
        else:
            rospy.loginfo('target position = ' + str(pose.position.x) + ', ' + str(pose.position.y))
            rospy.loginfo('approaching target...')
        return isArrived

    def waitforLight(self):
        if self.state == self.car_states['LIGHT']:
            lightStatus = self.lightStatusMsg
            if lightStatus == 0:
                rospy.loginfo('waiting for light signal...')
                self.stop()
                self.no_signal_bufsize = 0
                self.state = self.car_states['LIGHT']
            else :
                rospy.loginfo('signal received.')
                if lightStatus == 1:
                    rospy.loginfo('entering right')
                    self.no_signal_bufsize = 0
                    self.state = self.car_states['GORIGHT']
                elif lightStatus == 2:
                    rospy.loginfo('entering left')
                    self.no_signal_bufsize = 0
                    self.state = self.car_states['GOLEFT']
                elif (self.no_signal_bufsize < 20):
                    self.no_signal_bufsize += 1
                    self.state = self.car_states['LIGHT']
                else:
                    rospy.logwarn('warning: can not wait')
                    self.state = self.car_states['GOLEFT']
                    self.no_signal_bufsize = 0
                        
    def lightRight(self):
        if self.state == self.car_states['GORIGHT']:
            self.indicator = 0
            if self.leftClearMsg:
                self.state = self.car_states['RIGHT']
            else:
                self.state = self.car_states['GORIGHT']
                
    def lightLeft(self):
        if self.state == self.car_states['GOLEFT']:
            self.indicator = 1
            if self.rightClearMsg:
                self.state = self.car_states['LEFT']
            else:
                self.state = self.car_states['GOLEFT']
                
    def turnRight(self):
        if self.state == self.car_states['RIGHT']:
            self.rightCmd()
            rospy.loginfo('turning right')
            if not self.leftClearMsg:
                self.state = self.car_states['ENTERTURN']
            else:
                self.state = self.car_states['RIGHT']

    def turnLeft(self):
        if self.state == self.car_states['LEFT']:
            self.leftCmd()
            rospy.loginfo('turning left')
            if not self.rightClearMsg:
                self.state = self.car_states['ENTERTURN']
            else:
                self.state = self.car_states['LEFT']

    def enterTurn(self):
        if self.state == self.car_states['ENTERTURN']:
            rospy.loginfo('back to normal avoid mode')
            self.sideWayClearCnt = 0
            self.middleCmd()
            self.state = self.car_states['EXITTURN']

    def exitTurn(self):
        if self.state == self.car_states['EXITTURN']:
            rospy.loginfo('exiting turn')
            rospy.loginfo('sideWayClearCnt ' + str(self.sideWayClearCnt))
            if self.indicator:
                if self.rightClearMsg == 2: # and (not self.headingMsg == 'left'):
                    rospy.loginfo('heading' + str(self.headingMsg))
                    rospy.loginfo('right clear, more than sideWayFurtherClearDis')
                    self.sideWayClearCnt += 1
                    self.leftCmd()
                if (self.sideWayClearCnt > self.sideWayClearWindow) and (self.rightClearMsg == 0):
                    rospy.loginfo('exit!!!!')
                    self.state = self.car_states['STRAIGHT']
            else:
                if self.leftClearMsg == 2: # and (not self.headingMsg == 'right'):
                    rospy.loginfo('heading' + str(self.headingMsg))
                    rospy.loginfo('left clear, more than sideWayFurtherClearDis')
                    self.sideWayClearCnt += 1
                    self.rightCmd()
                if (self.sideWayClearCnt > self.sideWayClearWindow) and (self.leftClearMsg == 0):
                    rospy.loginfo('exit!!!!')
                    self.state = self.car_states['STRAIGHT']

    def goStraight(self):
        if self.state == self.car_states['STRAIGHT']:
            if (not self.leftClearMsg) and (not self.rightClearMsg):
                self.middleCmd()
            if self.straightflag:
                self.state = self.car_states['DYNAMIC']
            else:
                self.state = self.car_states['STRAIGHT']

    def waitforDynamicObj(self):
        if self.state == self.car_states['DYNAMIC']: 
            rospy.loginfo('waiting for dynamic obj')
            isFront = self.isFrontMsg
            if isFront:
                rospy.loginfo('pedestrian detected, stopping...')
                self.stop()
                self.state = self.car_states['DYNAMIC']
            else:
                if self.leftClearMsg == 2:
                    rospy.loginfo('pedestrian crossing passed!')
                    self.state = self.car_states['PARK']
                else:
                    self.state = self.car_states['DYNAMIC']

    def park(self):
        if self.state == self.car_states['PARK']:
            rospy.loginfo('parking...')
            park = self.parkMsg
            if park:
                self.state = self.car_states['PARKONE']
            else:
                self.state = self.car_states['PARKTWO']

    def parkOne(self):
        if self.state == self.car_states['PARKONE']:
            rospy.loginfo('... to spot one')
            if self.isFrontMsg == 2:
                self.stop()
            else:
                self.leftCmd()
                    
    def parkTwo(self):
        if self.state == self.car_states['PARKTWO']:
            rospy.loginfo('... to spot two')
            if self.isFrontMsg == 2:
                self.stop()
    
    def _shutdown(self):
        self.stop()

    def spin(self):
        while not rospy.is_shutdown():
            self.waitforLight()
            self.lightLeft()
            self.lightRight()
            self.turnLeft()
            self.turnRight()
            self.enterTurn()
            self.exitTurn()
            self.goStraight()
            self.waitforDynamicObj()
            self.park()
            self.parkOne()
            self.parkTwo()
            self.rate.sleep()

    def avoidStatePublish(self,state):
        msg = UInt8()
        msg.data = state
        self.statePub.publish(msg)

    def stop(self):
        drive_msg = Twist()
        drive_msg.linear.x = 0.0
        drive_msg.angular.z = 0.0
        self.velPub.publish(drive_msg)
        rospy.loginfo('stop')

    def middleCmd(self):
        rospy.loginfo('back to normal avoid mode')
        self.avoidStatePublish(0)

    def leftCmd(self):
        rospy.loginfo('going along the left wall')
        self.avoidStatePublish(3)
        
    def rightCmd(self): 
        rospy.loginfo('going along the right wall')
        self.avoidStatePublish(4)
            
if __name__ == '__main__':
    try:
        mode = rospy.get_param('mode','0')
        assigner = assigner(mode)
        
        assigner.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation finished.")
