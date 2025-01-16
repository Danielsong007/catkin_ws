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
        if self.recorddata == 1:
            self.dt=time.time()-self.now
            self.dt_list.append(self.dt)
            self.JointP_list0.append(self.JointP[0])
            self.JointT_list0.append(self.JointT[0])
            self.JointP_list1.append(self.JointP[1])
            self.JointT_list1.append(self.JointT[1])
            self.JointP_list2.append(self.JointP[2])
            self.JointT_list2.append(self.JointT[2])

    def GoToOnePoint(self,GoalPoint,HighAccuracy):
        self.GoalPos.Position=GoalPoint # Moveit order
        self.GoalPos.HighAccuracy=HighAccuracy # Moveit order
        for i in range(20):
            self.pub.publish(self.GoalPos)
            # self.rate.sleep()
            time.sleep(0.005)

    
    def PickOneBox(self,Height,ClosePoseI,MidPoseI,FarPoseI,D_suck):
        D_Pull=0.1
        ClosePose=[Height,ClosePoseI[1],ClosePoseI[2], ClosePoseI[3],ClosePoseI[4], ClosePoseI[5],ClosePoseI[6]]
        MidPose=[Height,MidPoseI[1],MidPoseI[2], MidPoseI[3],MidPoseI[4], MidPoseI[5],MidPoseI[6]]
        FarPose=[Height,FarPoseI[1],FarPoseI[2], FarPoseI[3],FarPoseI[4], FarPoseI[5],FarPoseI[6]]
        self.GoToOnePoint(FarPose,0)
        self.GoToOnePoint(MidPose,0)
        self.GoToOnePoint(ClosePose,1)
        print('Open gripper:')
        self.Gripper_Client(1)
        self.GoToOnePoint([ClosePose[0],ClosePose[1],ClosePose[2],ClosePose[3],ClosePose[4], ClosePose[5]+D_suck,ClosePose[6]],1) # Suck
        time.sleep(0.5)
        self.GoToOnePoint([ClosePose[0],ClosePose[1],ClosePose[2],ClosePose[3],ClosePose[4], ClosePose[5]-D_Pull,ClosePose[6]],1) # Pull
        self.GoToOnePoint([MidPose[0],MidPose[1],MidPose[2],MidPose[3],MidPose[4], MidPose[5]-D_Pull,MidPose[6]],0) # Mid Pose
        self.GoToOnePoint([FarPose[0],FarPose[1],FarPose[2],FarPose[3],FarPose[4], FarPose[5]-D_Pull,FarPose[6]],0) # Far Pose
        self.GoToOnePoint([FarPose[0],FarPose[1],FarPose[2],FarPose[3]-90,FarPose[4], FarPose[5]-D_Pull,FarPose[6]],1) # Final Pose
        # self.GoToOnePoint([FarPose[0],FarPose[1],FarPose[2],FarPose[3]-90,FarPose[4], FarPose[5]+0.08,FarPose[6]],1) # Push
        print('Close gripper:')
        self.Gripper_Client(0)
        time.sleep(1)
        self.GoToOnePoint(FarPose,0) # Far Pose
        print('Complete one box')
    
    def PickMultiBoxes(self):
        Height=[1.505,0.93,0.395]
        ClosePose1=[0,33,-15,-18,0, 0,0]
        MidPose1=[0,15,7.5,-22.5,0, 0,0]
        FarPose1=[0,0,30,-30,0, 0,0]
        ClosePose2=[0,30,-35,5,0, 0,0]
        MidPose2=[0,10,-10,0,0, 0,0]
        FarPose2=[0,-10,15,-5,0, 0,0]
        ClosePose3=[0,45,-90,45,0, 0,0]
        MidPose3=[0,28,-70,43,0, 0,0]
        FarPose3=[0,10,-50,40,0, 0,0]
        self.PickOneBox(Height[0],ClosePose1,MidPose1,FarPose1,0.26) # Up1
        self.PickOneBox(Height[0],ClosePose2,MidPose2,FarPose2,0.255) # Up2
        self.PickOneBox(Height[0],ClosePose3,MidPose3,FarPose3,0.265) # Up3
        self.PickOneBox(Height[1],ClosePose1,MidPose1,FarPose1,0.23) # Mid1
        self.PickOneBox(Height[1],ClosePose2,MidPose2,FarPose2,0.25) # Mid2
        self.PickOneBox(Height[1],ClosePose3,MidPose3,FarPose3,0.266) # Mid3
        self.PickOneBox(Height[2],ClosePose1,MidPose1,FarPose1,0.23) # Down1
        self.PickOneBox(Height[2],ClosePose2,MidPose2,FarPose2,0.23) # Down2
        self.PickOneBox(Height[2],ClosePose3,MidPose3,FarPose3,0.245) # Down3
    
    def PickWithC(self):
        print('Suck')
        self.GoToOnePoint([0,0,0,0,0, 0.15,0], 1)
        while self.TAverage<110:
            rospy.sleep(0.01)
        self.GoToOnePoint([0,0,0,0,0, self.JointP[5],0], 1)
        time.sleep(1)
        print ('Lift')
        self.GoToOnePoint([0,0,0,0,0, self.JointP[5],0.12], 1)
        time.sleep(2.5)
        print ('pull')
        self.GoToOnePoint([0,0,0,0,0, -0.05,0.12], 1)
        time.sleep(2.5)
        print ('Down')
        self.GoToOnePoint([0,0,0,0,0, -0.08,0], 1)
        time.sleep(1)

    def DirectPick(self):
        print('Suck')
        time.sleep(5)
        self.GoToOnePoint([0,0,0,0,0, 0.3,0], 1)
        now=time.time()
        while self.TAverage<90 and (time.time()-now)<10:
            rospy.sleep(0.01)
        print ('Pull')
        self.GoToOnePoint([0,0,0,0,0, -0.19,0], 1)
    
    def Cyclemotion(self):
        time.sleep(10)
        dt_list=[]
        JointP_list=[]
        JointT_list=[]
        now=time.time()
        dt=0
        while dt < 30:
            A=0.3 # J1 0.6/2; J2/3
            b=0 # J1 A; J2/3 0
            omega=2*math.pi/10
            JP=A*math.sin(omega*dt)+b
            self.GoToOnePoint([0,JP,0,0,0, 0,0], 1)
            dt=time.time()-now
            dt_list.append(dt)
            JointP_list.append(self.JointP[1])
            JointT_list.append(self.JointT[1])
        f=open('test.txt','w')
        f.write(str(dt_list))
        f.write('\n')
        f.write(str(JointP_list))
        f.write('\n')
        f.write(str(JointT_list))
        f.close
        plt.subplot(1,2,1)
        plt.plot(dt_list,JointP_list)
        plt.subplot(1,2,2)
        plt.plot(dt_list,JointT_list)
        plt.show()
    
    def transport(self):
        print ('Goal')
        self.GoToOnePoint([0.3,-0.28,0.7,0.7,0, 0,0], 1)
        time.sleep(4)
        time.sleep(4.5)
        print ('Back')
        self.GoToOnePoint([-0.25,0.1,0.1,-0.45,0, 0,0], 1)
        time.sleep(5)
        time.sleep(2)
        self.GoToOnePoint([-0,-0,-0,0,0, 0.0,0], 1)
        time.sleep(2)
    
    def PickWithC_TO(self):
        print('Suck')
        time.sleep(1)
        self.GoToOnePoint([0,0,0,0,0, 0.12,0], 1)
        now=time.time()
        while self.TAverage<110 and (time.time()-now)<3:
            rospy.sleep(0.01)
        print ('Lift')
        self.GoToOnePoint([0,0,0,0,0, self.JointP[5]-0.01,0.11], 1)
        time.sleep(2.5)
        print ('pull')
        self.GoToOnePoint([0,0,0,0,0, -0.1,0.11], 1)
        time.sleep(2)
        print ('Down')
        self.GoToOnePoint([0,0,0,0,0, -0.19,0.03], 1)
        time.sleep(2)
    
    def PickofLP_NTO(self):
        time.sleep(10)
        print ('Lift')
        self.GoToOnePoint([0,0,0,0,0, -0.02,0.165], 1)
        time.sleep(5.5)
        print ('pull')
        self.GoToOnePoint([0,0,0,0,0, -0.28,0.165], 1)
        time.sleep(6)
        print ('Down')
        self.GoToOnePoint([0,0,0,0,0, -0.34,0.1], 1)
        time.sleep(8)

    def PickofHome(self):
        print ('Back')
        self.GoToOnePoint([0,0,0,0,0, 0,0.1], 1)
        time.sleep(20)
        print ('Down')
        self.GoToOnePoint([0,0,0,0,0, 0,0], 1)
        time.sleep(3)
    
    def PickofLP_TO(self):
        print ('Lift')
        self.GoToOnePoint([0,0,0,0,0, -0.15,0.11], 1)
        time.sleep(2.5)
        print ('Pull and Down')
        self.GoToOnePoint([0,0,0,0,0, -0.15,0.03], 1)
        time.sleep(2)

if __name__ == '__main__':
    try:
        print("Start")
        rospy.init_node('mainpkg')
        mydemo = DemoByJoints()

        # mydemo.GoToOnePoint([0,0,0,0,0, 0.366,-0.035], 1) # Test Point, m and rad
        mydemo.PickofLP_NTO()
        mydemo.PickofHome()
        # mydemo.GoToOnePoint([0,0,0,0,0, -0.34,0.09], 1)
        # mydemo.DirectPick()
        # mydemo.PickWithC()
        # mydemo.PickWithC_TO()
        # mydemo.transport()
        # mydemo.Cyclemotion()
        # mydemo.PickMultiBoxes() # !!!Need to change to rad from deg
        # rospy.spin() # spin() simply keeps python from exiting until this node is stopped
        
    except rospy.ROSInterruptException:
        pass
