#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from MainNode.msg import Num
from sensor_msgs.msg import JointState
import actionlib, time
from control_msgs.msg import FollowJointTrajectoryAction
from std_srvs.srv import SetBool, SetBoolResponse


class Talker():
    def __init__(self):
        rospy.init_node('talker')
        self.rate = rospy.Rate(10)
        self.GoalPos=Num()
        self.JointSize=7
        self.CurPos=[1000,0,0,0,0,0,0]

        self.pub = rospy.Publisher('chatter', Num, queue_size=10)
        self.Sub_Robotstate=rospy.Subscriber('robot/joint_states', JointState, self.Sub_Robotstate_Handle)
        rospy.wait_for_service('/gripper/run')
        self.Gripper_Client = rospy.ServiceProxy('/gripper/run', SetBool)

    def Sub_Robotstate_Handle(self,msg):
        self.CurPos=msg.position
        self.CurVelocity=msg.velocity
        # print(self.CurPos)
    
    def ArriveCheck(self,HighAccuracy):
        Flag_Arrive=1 # 0 means arrived
        if HighAccuracy == 0:
            Err_limit_T=0.01
            Err_limit_R=0.1
        else:
            Err_limit_T=0.001
            Err_limit_R=0.01
        for i in range(self.JointSize-1): 
            if i in range(3): # Translate
                if abs(self.GoalPos.num[i]-self.CurPos[i])>=Err_limit_T:
                    Flag_Arrive=0
            else: # Rotate
                if abs(self.GoalPos.num[i]-self.CurPos[i])>=Err_limit_R:
                    Flag_Arrive=0
        return Flag_Arrive
    
    def GoToOnePoint(self,GoalPoint,HighAccuracy):
        self.GoalPos.num=GoalPoint # Go to a Point
        print('Go to',self.GoalPos)
        while self.ArriveCheck(HighAccuracy)==0:
            self.pub.publish(self.GoalPos)
            # print('Go-go-go!')
            self.rate.sleep()
        print('It arrived:',self.GoalPos)
        self.pub.publish(self.CurPos)
    
    def PickOneBox(self,BoxPose):
        self.GoToOnePoint(BoxPose,0) # Middle Point
        self.GoToOnePoint([0.45,0,0, 20,-20,0,0],1) # Face Point
        self.GoToOnePoint([0.45,0.05,0, 20,-20,0,0],1) # Suck
        print('Open gripper:')
        # self.ripper_Client(1)
        time.sleep(1)
        self.GoToOnePoint([0.45,-0.05,0, 20,-20,0,0],1) # Pull
        self.GoToOnePoint([0.45,0,0, -20,20,0,0],0) # Middle Point
        self.GoToOnePoint([0.45,0,0, -20,-20,0,0],0) # Transmit
        self.GoToOnePoint([0.45,0.05,0, -20,-20,0,0],1) # Push
        self.GoToOnePoint([0.45,0,0, -20,20,0,0],0) # Middle Point
        print('Close gripper:')
        # self.Gripper_Client(0)
        time.sleep(0.05)
        print('Bye-Bye')
    
    def PickMultiBoxes(self):
        self.PickOneBox([0.45,0,0, -20,20,0,0])
        self.PickOneBox([0.45,0,0, -20,20,0,0])

if __name__ == '__main__':
    # while not rospy.is_shutdown():
    try:
        MyTalker = Talker()
        MyTalker.GoToOnePoint([0,0,0, 0,0,0,0],1) # Home
        # MyTalker.PickOneBox()
        # MyTalker.PickMultiBoxes()
    except rospy.ROSInterruptException:
        pass
