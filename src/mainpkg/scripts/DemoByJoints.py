#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from mainpkg.msg import MyGoal
from sensor_msgs.msg import JointState
import actionlib, time
from control_msgs.msg import FollowJointTrajectoryAction
from std_srvs.srv import SetBool, SetBoolResponse
import sys


class DemoByJoints():
    def __init__(self):
        self.rate = rospy.Rate(20)
        self.GoalPos=MyGoal()
        self.JointSize=7

        self.pub = rospy.Publisher('ManualPosCmd', MyGoal, queue_size=10)
        rospy.wait_for_service('/gripper/run')
        self.Gripper_Client = rospy.ServiceProxy('/gripper/run', SetBool)
    
    def GoToOnePoint(self,GoalPoint,HighAccuracy):
        self.GoalPos.Position=GoalPoint # Moveit order
        self.GoalPos.HighAccuracy=HighAccuracy # Moveit order
        for i in range(3):
            self.pub.publish(self.GoalPos)
            self.rate.sleep()
        print ('Published the goal')
    
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
    
if __name__ == '__main__':
    try:
        print("Start")
        rospy.init_node('mainpkg')
        MyDemoByJoints = DemoByJoints()
        # MyDemoByJoints.GoToOnePoint([0,0,0,0,0, 0,0],1) # Home
        MyDemoByJoints.GoToOnePoint([0, 0, 0, 0, 0, 0, 0], 1) # Test Point, m and rad
        # MyDemoByJoints.PickMultiBoxes() # !!!Need to change to rad from deg
        
    except rospy.ROSInterruptException:
        pass
