#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from beginner_tutorials.msg import Num
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction

def talker():
    pub = rospy.Publisher('chatter', Num, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    Mymsg=Num()
    while not rospy.is_shutdown():
        Mymsg.num=[0,0,0,0,0,0]
        rospy.loginfo(Mymsg.num)
        pub.publish(Mymsg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
