#! /usr/bin/env python
import rospy
from std_msgs.msg import String
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class curi_ros_trajectory_action():
    def __init__(self):
        self.server = actionlib.ActionServer("palletizer_arm/follow_joint_trajectory", FollowJointTrajectoryAction, self.on_goal, self.on_cancel, auto_start=False)
        self.goalP=[0.0] * 5
        self.goalV=[0.0] * 5
        self.RecieveG=0

    def start(self):
        self.server.start()
        print("Started")
    def on_goal(self, goal_handle):
        goal_handle.set_accepted()
        print('ON goal')
        print('All states:')
        print(goal_handle.get_goal().trajectory.points)
        print('Start state:')
        print(goal_handle.get_goal().trajectory.points[0].positions)
        print('Final state:')
        print(goal_handle.get_goal().trajectory.points[-1].positions)
        self.RecieveG=1
        num=len(goal_handle.get_goal().trajectory.points)
        print('Num is:', num)
        for i in range(0,num):
            self.goalP=goal_handle.get_goal().trajectory.points[i].positions
            self.goalV=goal_handle.get_goal().trajectory.points[i].velocities
            time.sleep(0.15)
        goal_handle.set_succeeded()

    def on_cancel(self, goal_handle):
        print('ON cancel')

def talker():
    rospy.init_node('Myserver', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    action_server = curi_ros_trajectory_action()
    action_server.start()

    pub = rospy.Publisher('joint_states', JointState, queue_size=5)
    joint_states = JointState()
    joint_states.header = Header()
    joint_states.header.stamp = rospy.Time.now()
    joint_states.position = [0.0] * 7
    joint_states.velocity = [0.0] * 7
    joint_states.effort = [0.0] * 7
    joint_states.name = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "Joint7"]
    pub.publish(joint_states)
    while not rospy.is_shutdown():
        if action_server.RecieveG == 0:
            joint_states.header.stamp = rospy.Time.now()
            pub.publish(joint_states)
        else:
            joint_states.header.stamp = rospy.Time.now()
            joint_states.position[0:5] = action_server.goalP
            joint_states.velocity[0:5] = action_server.goalV
            pub.publish(joint_states)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
