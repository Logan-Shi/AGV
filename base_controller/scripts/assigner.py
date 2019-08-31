#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, Twist
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class assigner():
    def __init__(self, arg):
        rospy.init_node('assigner_py')
        self.arg = arg
        self.rate = rospy.Rate(2)
        rospy.on_shutdown(self._shutdown)
        self.goal_states = ('PENDING', 'ACTIVE', 'PREEMPTED',  
                            'SUCCEEDED', 'ABORTED', 'REJECTED', 
                            'PREEMPTING', 'RECALLING', 'RECALLED', 
                            'LOST')
        self.car_states = ('LIGHT','TURN','STRAIGHT','PARK')
        self.state = 0
        self.left_turn_pose = Pose(Point(9,2.3,0),Quaternion(0,0,0,1))
        self.right_turn_pose = Pose(Point(9,-2.3,0),Quaternion(0,0,0,1))
        self.exit_turn_pose = Pose(Point(13,0,0),Quaternion(0,0,0,1))
        self.straight_lane_pose = Pose(Point(13,4,0),Quaternion(0,0,1,0))
        self.park_request_pose = Pose(Point(5,4,0),Quaternion(0,0,1,0))
        self.park_one_pose = Pose(Point(3,4,0),Quaternion(0,0,1,0))
        self.park_two_pose = Pose(Point(3,2,0),Quaternion(0,0,1,0))
        self.pose = Pose()
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.map_base_link_tf_listener = tf.TransformListener()
        self.velPub = rospy.Publisher("/cmd_vel_mux/input/assigner",\
                Twist, queue_size =1 )
        rospy.sleep(1)

    def sendGoal(self,pose):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        self.client.send_goal(goal)
        self.checkGoalStatus()
            
    def checkGoalStatus(self):
        if not self.client.wait_for_result(rospy.Duration(300)):
            rospy.loginfo("Timed out to achieve goal!")
        else:
            state = self.client.get_state()
            if self.goal_states[state] == 'SUCCEEDED': 
                rospy.loginfo("Goal reached!")
            else:
                rospy.loginfo("Goal failed:" + str(self.goal_states[state]))

    def updatePose(self):
        self.map_base_link_tf_listener.waitForTransform('/base_link','/map',rospy.Time(), rospy.Duration(0.5))
        (trans,rot) = self.map_base_link_tf_listener.lookupTransform('/base_link','/map',rospy.Time(0))
        self.pose.position.x = trans[0]
        self.pose.position.y = trans[1]
        self.pose.position.z = trans[2]
        self.pose.orientation.x = rot[0]
        self.pose.orientation.y = rot[1]
        self.pose.orientation.z = rot[2]
        self.pose.orientation.w = rot[3]
        rospy.loginfo("pose = \n" + str(self.pose))

    def waitforLight(self):
        while self.car_states[self.state] == 'LIGHT':
            lightStatus = 1
            if lightStatus == 0:
                rospy.loginfo('waiting for light signal...')
                self.stop()
            else :
                rospy.loginfo('signal received.')
                self.state = 1
                if lightStatus == 1:
                    rospy.loginfo('turning right')
                    self.sendGoal(self.right_turn_pose)
                else:
                    rospy.loginfo('turning left')
                    self.sendGoal(self.left_turn_pose)
            return lightStatus

    def goStraight(self):
        pass

    def park(self):
        pass
        
    def _shutdown(self):
        self.client.cancel_goal()
        self.stop()

    def spin(self):
        while not rospy.is_shutdown():
            self.updatePose()
            while self.waitforLight():
                if self.car_states[self.state] == 'TURN':
                    rospy.loginfo('exiting turn')
                    self.sendGoal(self.exit_turn_pose)
                    self.state = 2
                else:
                    self.state = 0
                self.goStraight()
                self.park()
            self.rate.sleep()

    def stop(self):
        drive_msg = Twist()
        drive_msg.linear.x = 0.0
        drive_msg.angular.z = 0.0
        self.velPub.publish(drive_msg)

if __name__ == '__main__':
    try:
        assigner = assigner(1)
        assigner.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation finished.")