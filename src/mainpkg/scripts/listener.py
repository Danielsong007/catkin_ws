#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mainpkg.msg import MyGoal
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
import actionlib

def on_goal(self, goal_handle):
    points=goal_handle.get_goal().trajectory.points
    traj = goal_handle.get_goal().trajectory
    goal_handle.set_accepted()

def on_cancel(self, goal_handle):
    rospy.loginfo('it is cancelled')

def TopicCallback(msg):
    rospy.loginfo(msg.effort[5])

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('joint_states', JointState, TopicCallback)
    # myserver = actionlib.ActionServer("robot/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction, on_goal, on_cancel, auto_start=False)
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
