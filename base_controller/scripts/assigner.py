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
        self.rate = rospy.Rate(60)
        self.velPub = rospy.Publisher("/cmd_vel_mux/input/assigner", Twist, queue_size =1)
        self.velAlterPub = rospy.Publisher("/assignerMsg/speed", Twist, queue_size =1)
        self.statePub = rospy.Publisher('/assignerMsg/assignerState', UInt8, queue_size=1)
        rospy.Subscriber('/assignerMsg/isFront', Twist, self.laserDataCallback)
        rospy.Subscriber('/assignerMsg/hilensData',UInt8,self.hilensDataCallback)
        #rospy.Subscriber('cmd_vel',Twist,self.servoCallback)

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
        self.sim = 0 # 0 as in real, 1 as in sim
        self.indicator = 1 # 0 as right, 1 as left
        if self.sim:
            self.lightStatusMsg = 1 # 0 as stop, 1 as right
        else:
            self.lightStatusMsg = 0
        self.parkMsg = 1 # 1 as spot one, 2 as spot two
        self.sideWayClearWindow = 0
        if self.sim:
            self.state = 5
        else:
            self.state = 0

        self.flag_list=[0,0]
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
        self.cnt = 0
        self.exit_cnt = 0
        self.rightClearMsg = 0
        self.leftClearMsg = 0
        self.isFrontMsg = 0
        self.headingMsg = 'straight'
        self.straightflag = 0
        self.turnClear = 0
        self.startParkMsg = 0
        self.no_signal_bufsize = 0

    # def servoCallback(self,msg):
    #     turn_reference = 0.01
        
    #     if (msg.angular.z > turn_reference):
    #         self.headingMsg = 'left'
    #     elif (msg.angular.z < - turn_reference):
    #         self.headingMsg = 'right'
    #     else:
    #         self.headingMsg = 'straight'
            
    def laserDataCallback(self,msg):
        frontDis = min(msg.linear.x, msg.linear.y)

        parkingStopDis = 0.54
        dynamicStopDis = 0.5
        
        if frontDis < parkingStopDis: 
            self.isFrontMsg = 2 # less than parkingStopDis
        elif frontDis < dynamicStopDis:
            self.isFrontMsg = 1 # less than dynamicStopDis
        else:
            self.isFrontMsg = 0 # not gonna stop
        
        parkDis = 1.3
        turnEnterDis = 0.50
        sideWayClearDis = 0.6
        sideWayFurtherClearDis = 1.0
        
        if (msg.angular.y > turnEnterDis) and (msg.angular.x > turnEnterDis):
            self.turnClear = 1
        elif (msg.angular.y < turnEnterDis) and (msg.angular.x < turnEnterDis):
            self.turnClear = 0
        
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

        if msg.linear.y < parkDis:
            self.startParkMsg = 1
        else:
            self.startParkMsg = 0

    def hilensDataCallback(self,msg):
        lightFilterSize = 4
        parkFilterSize = 4

        self.flag_list.append(msg.data)
        if len(self.flag_list) > 100:
            self.flag_list = self.flag_list[90:]
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
        
        
        if flag_list_park[0] == flag_list_park[1]:
            flag_park_indi = flag_list_park[0]
        else:
            flag_park_indi = 0
        
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
            rospy.loginfo('exit double turn')
            if flag_park_indi == 5:
                self.straightflag = 1
                rospy.loginfo('hilens saw spot one')
            elif flag_park_indi == 6:
                self.straightflag = 1
                rospy.loginfo('hilens saw spot two')
            else:
                self.straightflag = 0
        
        if self.state == self.car_states['DYNAMIC']:
            rospy.loginfo('watching hilens')
            if flag_park == 5:
                self.parkMsg = 1
                rospy.loginfo('hilens told me to park at spot one')
            elif flag_park == 6:
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
        rospy.loginfo('current position = ' + str(self.pose.position.x) + ', ' + str(self.pose.position.y))

    def showState(self):
        rospy.loginfo("state = " + str(self.state))

    def waitforLight(self):
        if self.state == self.car_states['LIGHT']:
            self.alterVel(0.7)
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
            self.alterVel(0.9)
            if self.turnClear:
                self.state = self.car_states['RIGHT']
            else:
                self.state = self.car_states['GORIGHT']
                
    def lightLeft(self):
        if self.state == self.car_states['GOLEFT']:
            self.indicator = 1
            self.alterVel(0.9)
            if self.turnClear:
                self.state = self.car_states['LEFT']
            else:
                self.state = self.car_states['GOLEFT']
                
    def turnRight(self):
        if self.state == self.car_states['RIGHT']:
            self.alterVel(0.9)
            self.rightCmd()
            self.cnt += 1
            rospy.loginfo('side way blocked for ' + str(self.cnt))
            rospy.loginfo('turning right')
            if not self.turnClear and self.cnt > 0:
                self.state = self.car_states['ENTERTURN']
            else:
                self.state = self.car_states['RIGHT']

    def turnLeft(self):
        if self.state == self.car_states['LEFT']:
            self.alterVel(0.9)
            self.leftCmd()
            self.cnt += 1
            rospy.loginfo('side way blocked for ' + str(self.cnt))
            rospy.loginfo('turning left')
            if not self.turnClear and self.cnt > 0:
                self.state = self.car_states['ENTERTURN']
            else:
                self.state = self.car_states['LEFT']

    def enterTurn(self):
        if self.state == self.car_states['ENTERTURN']:
            self.alterVel(0.9)
            if self.indicator:
                self.leftCmd()    
            else:
                self.rightCmd()
            self.sideWayClearCnt = 0
            self.state = self.car_states['EXITTURN']

    def exitTurn(self):
        if self.state == self.car_states['EXITTURN']:
            self.alterVel(0.9)
            self.exit_cnt = 0
            rospy.loginfo('exiting turn')
            rospy.loginfo('sideWayClearCnt ' + str(self.sideWayClearCnt))
            if self.indicator:
                self.leftCmd()
                if self.rightClearMsg == 2: # and (not self.headingMsg == 'left'):
                    #rospy.loginfo('heading' + str(self.headingMsg))
                    rospy.loginfo('right clear, more than sideWayFurtherClearDis')
                    self.sideWayClearCnt += 1
                    
                if (self.sideWayClearCnt > self.sideWayClearWindow) and (self.rightClearMsg == 0):
                    rospy.loginfo('exit!!!!')
                    self.state = self.car_states['STRAIGHT']
            else:
                self.rightCmd()
                if self.leftClearMsg == 2: # and (not self.headingMsg == 'right'):
                    #rospy.loginfo('heading' + str(self.headingMsg))
                    rospy.loginfo('left clear, more than sideWayFurtherClearDis')
                    self.sideWayClearCnt += 1
                    
                if (self.sideWayClearCnt > self.sideWayClearWindow) and (self.leftClearMsg == 0):
                    rospy.loginfo('exit!!!!')
                    self.state = self.car_states['STRAIGHT']

    def goStraight(self):
        if self.state == self.car_states['STRAIGHT']:
            self.alterVel(0.7)
            if (not self.leftClearMsg) and (not self.rightClearMsg):
                self.leftCmdTest()
                self.exit_cnt += 1
                rospy.loginfo('side way blocked for straight' + str(self.exit_cnt))
            if (self.exit_cnt > 270) and self.straightflag and (not self.isFrontMsg):
                self.state = self.car_states['DYNAMIC']
            else:
                self.state = self.car_states['STRAIGHT']

    def waitforDynamicObj(self):
        if self.state == self.car_states['DYNAMIC']:
            self.alterVel(1.0)
            self.middleCmd() 
            rospy.loginfo('waiting for dynamic obj')
            isFront = self.isFrontMsg
            if isFront:
                rospy.loginfo('pedestrian detected, stopping...')
                self.stop()
                self.state = self.car_states['DYNAMIC']
            else:
                if self.leftClearMsg:
                    rospy.loginfo('pedestrian crossing passed!')
                    self.state = self.car_states['PARK']
                else:
                    self.state = self.car_states['DYNAMIC']

    def park(self):
        if self.state == self.car_states['PARK']:
            self.alterVel(0.9)
            rospy.loginfo('parking...')
            park = self.parkMsg
            if park == 1:
                self.state = self.car_states['PARKONE']
            else:
                self.state = self.car_states['PARKTWO']

    def parkOne(self):
        if self.state == self.car_states['PARKONE']:
            self.alterVel(0.9)
            rospy.loginfo('... to spot one')
            self.leftCmd()
            if self.isFrontMsg == 2:
                self.stop()
                    
    def parkTwo(self):
        if self.state == self.car_states['PARKTWO']:
            self.alterVel(0.9)
            rospy.loginfo('... to spot two')
            if self.startParkMsg == 1:
                self.leftCmd()
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

    def alterVel(self,v):
        speed_msg = Twist()
        speed_msg.linear.x = v
        self.velAlterPub.publish(speed_msg)
    
    def parkCmd(self):
        rospy.loginfo('park mode')
        self.avoidStatePublish(2)
        
    def middleCmd(self):
        rospy.loginfo('back to normal avoid mode')
        self.avoidStatePublish(0)

    def leftCmd(self):
        rospy.loginfo('going along the left wall')
        self.avoidStatePublish(3)
        
    def rightCmd(self): 
        rospy.loginfo('going along the right wall')
        self.avoidStatePublish(4)

    def leftCmdTest(self): 
        rospy.loginfo('left test')
        self.avoidStatePublish(5)

    def rightCmdTest(self): 
        rospy.loginfo('right test')
        self.avoidStatePublish(6)
            
if __name__ == '__main__':
    try:
        mode = rospy.get_param('mode','0')
        assigner = assigner(mode)
        
        assigner.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation finished.")