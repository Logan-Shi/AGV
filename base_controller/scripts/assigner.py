#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
import actionlib
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

    def sendGoal(self,pose):
        rospy.loginfo("send new goal:\n" + str(pose))
        
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        self.client.send_goal(goal)
            
    def checkGoalStatus(self):
        if not self.client.wait_for_result(rospy.Duration(300)):
            rospy.loginfo("Timed out to achieve goal!")
        else:
            state = self.client.get_state()
            if self.goal_states[state] == 'SUCCEEDED': 
                rospy.loginfo("Goal reached!")
            else:
                rospy.loginfo("Goal failed:" + str(self.goal_states[state]))

    def parser(self,msg):
        self.pose.position.x = 0.5
        self.pose.orientation.w = 1

    def _shutdown(self):
        self.client.cancel_goal()

    def spin(self):
        while not rospy.is_shutdown():
            self.parser(0)
            self.sendGoal(self.pose)
            self.checkGoalStatus()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        assigner = assigner(1)
        assigner.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation finished.")