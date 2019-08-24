#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
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
        self.pose = Pose()
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.map_base_link_tf_listener = tf.TransformListener()
        rospy.sleep(1)

    def sendGoal(self,pose):
        rospy.loginfo("send new goal:\n" + str(pose))
        
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

    def parser(self,msg):
        goal_pose = Pose(Point(0,0,0),Quaternion(0,0,0,1))
        end_pose = Pose(Point(-1.7,-3.9,0),Quaternion(0,0,1,0))
        if (self.pose.position.x - end_pose.position.x < 0.1) and (self.pose.position.y - end_pose.position.y < 0.1):
            self.sendGoal(goal_pose)
        else:
            pass
        
    def _shutdown(self):
        self.client.cancel_goal()

    def spin(self):
        while not rospy.is_shutdown():
            self.updatePose()
            self.parser(0)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        assigner = assigner(1)
        assigner.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation finished.")