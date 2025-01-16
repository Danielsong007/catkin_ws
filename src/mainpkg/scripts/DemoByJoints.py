#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from mainpkg.msg import MyGoal
from sensor_msgs.msg import JointState
import actionlib, time
from control_msgs.msg import FollowJointTrajectoryAction
from std_srvs.srv import SetBool, SetBoolResponse
import sys, math
import matplotlib.pyplot as plt



class DemoByJoints():
    def __init__(self):
        self.rate = rospy.Rate(200)
        self.GoalPos=MyGoal()
        self.JointSize=7
        self.LM1Torque=0
        self.JointT=[0,0,0,0,0,0,0]
        self.JointP=[0,0,0,0,0,0,0]
        self.Tlist=[0,0,0,0,0,0]
        self.TAverage=0
        self.recorddata=0

        self.pub = rospy.Publisher('ManualPosCmd', MyGoal, queue_size=10)
        rospy.Subscriber('joint_states', JointState, self.JSCallback)
        rospy.wait_for_service('/gripper/run')
        self.Gripper_Client = rospy.ServiceProxy('/gripper/run', SetBool)

    def JSCallback(self,msg):
        self.JointP=msg.position
        self.JointT=msg.effort
        self.LM1Torque=self.JointT[5]

        self.Tlist[5]=self.Tlist[4]
        self.Tlist[4]=self.Tlist[3]
        self.Tlist[3]=self.Tlist[2]
        self.Tlist[2]=self.Tlist[1]
        self.Tlist[1]=self.Tlist[0]
        self.Tlist[0]=self.LM1Torque
        self.TAverage=(self.Tlist[0]+self.Tlist[1]+self.Tlist[2]+self.Tlist[3]+self.Tlist[4]+self.Tlist[5])/6
        # print(self.Tlist,self.TAverage)
        # print(self.JointT)

    def GoToOnePoint(self,GoalPoint,HighAccuracy,Vlimit):
        self.GoalPos.Position=GoalPoint # Moveit order
        self.GoalPos.HighAccuracy=HighAccuracy # Moveit order
        self.GoalPos.JfiveV=Vlimit[0] # Joint Velocity limit
        self.GoalPos.JsixV=Vlimit[1] # Joint Velocity limit
        for i in range(20):
            self.pub.publish(self.GoalPos)
            time.sleep(0.005)

    def DirectPick(self):
        print('Suck')
        time.sleep(5)
        self.GoToOnePoint([0,0,0,0,0, 0.3,0], 1)
        now=time.time()
        while self.TAverage<90 and (time.time()-now)<10:
            rospy.sleep(0.01)
        print ('Pull')
        self.GoToOnePoint([0,0,0,0,0, -0.19,0], 1)
    
    def PickofLP_NTO(self):
        time.sleep(10)
        print ('Lift')
        self.GoToOnePoint([0,0,0,0,0, -0.06,0.13], 1, [0.025,0.06])
        time.sleep(5.5)
        print ('pull')
        self.GoToOnePoint([0,0,0,0,0, -0.28,0.13], 1, [0.05,0.05])
        time.sleep(3.1)
        print ('Down')
        self.GoToOnePoint([0,0,0,0,0, -0.35,0.05], 1, [0.055,0.035])
        time.sleep(5)

    def PickofHome(self):
        time.sleep(5)
        print ('Back')
        self.GoToOnePoint([0,0,0,0,0, -0,0.05], 1, [0.1,0.1])
        time.sleep(3)
        print ('Down')
        self.GoToOnePoint([0,0,0,0,0, 0,0], 1, [0.1,0.1])
        time.sleep(2)
    
    def Ctest(self):
        time.sleep(8)
        print ('go')
        self.GoToOnePoint([0,0,0,0,0, 0.325,0], 1, [0.1,0.1])
        time.sleep(4)
        print ('back')
        self.GoToOnePoint([0,0,0,0,0, 0.1,0], 1, [0.1,0.1])
        time.sleep(2)

if __name__ == '__main__':
    try:
        print("Start")
        rospy.init_node('mainpkg')
        mydemo = DemoByJoints()

        # mydemo.GoToOnePoint([0,0,0,0,0, 0.366,-0.035], 1) # Test Point, m and rad
        # mydemo.PickofLP_NTO()
        # mydemo.PickofHome()
        mydemo.GoToOnePoint([1.5,0,0,0,0, 0.2,0], 1, [0.1,0.1])
        # mydemo.Ctest()
        # mydemo.DirectPick()
        # rospy.spin() # spin() simply keeps python from exiting until this node is stopped
        
    except rospy.ROSInterruptException:
        pass
