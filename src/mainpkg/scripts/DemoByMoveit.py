#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from mainpkg.msg import MyGoal
from sensor_msgs.msg import JointState
import actionlib, time
from control_msgs.msg import FollowJointTrajectoryAction
from std_srvs.srv import SetBool, SetBoolResponse
import sys
import my_move_group
from math import pi

class DemoByMoveit():
    def __init__(self):
        self.rate = rospy.Rate(20)
        self.GoalPos=MyGoal()
        self.pub = rospy.Publisher('ManualPosCmd', MyGoal, queue_size=10)
    
    def GoToOnePoint(self,GoalPoint,HighAccuracy):
        self.GoalPos.Position=GoalPoint # Moveit order
        self.GoalPos.HighAccuracy=HighAccuracy # Moveit order
        for i in range(3):
            self.pub.publish(self.GoalPos)
            self.rate.sleep()
        print ('Published the goal')
    
if __name__ == '__main__':
    try:
        rospy.init_node('mainpkg')
        MyDemoByMoveit = DemoByMoveit()
        my_move_group=my_move_group.my_move_group()
        MyDemoByMoveit.GoToOnePoint([0,0,0,0,0, -0.05,0], 1) # Home, m and rad
        # my_move_group.go_to_pose_goal(0,0,0,1,0.5,0.56,0.4)
        # my_move_group.go_to_joint_state(0,0,0,0,0) # Home, m and rad
        
    except rospy.ROSInterruptException:
        pass
