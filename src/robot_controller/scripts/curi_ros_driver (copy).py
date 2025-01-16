#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolResponse
import serial
import struct
from mainpkg.msg import MyGoal
import re, json, sys, time
sys.path.append("..")
from communication.curi_communication_socket import curi_communication_socket
from base.curi_robot_control import curi_robot_control, robot, ROBOT_STATE, CONTROL_SPACE, CONTROL_MODE
from communication.curi_ros_trajectory_action import curi_ros_trajectory_action
import numpy as np
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction


s_open = [0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A]
s_close = [0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA]
# ser = serial.Serial(port='/dev/usb_0', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

class curi_ros_driver(robot):
    def __init__(self):
        self.JointSize = 7
        self.rate = rospy.Rate(20)
        self.HighAccuracy=1
        self.GoalPos=[0]*self.JointSize
        self.joint_states = JointState()
        self.joint_states.header = Header()
        self.joint_states.name = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "Joint7"]
        robot.__init__(self, self.JointSize, np.array([0.0] * self.JointSize))
        self.socket_communication = curi_communication_socket(self.JointSize, "192.168.0.1", 11230, "192.168.0.34", 11231)
        self.socket_communication.open()

        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)
        self.sub_new=rospy.Subscriber('ManualPosCmd', MyGoal, self.ManualPosCmd_handle)
        self.gripper_srv_ = rospy.Service('/gripper/run', SetBool, self.gripper_handle)

        self.armserver = actionlib.ActionServer("palletizer_arm/follow_joint_trajectory", FollowJointTrajectoryAction, self.on_goal_arm, self.on_cancel_arm, auto_start=True)
        self.handserver = actionlib.ActionServer("palletizer_hand/follow_joint_trajectory", FollowJointTrajectoryAction, self.on_goal_hand, self.on_cancel_hand, auto_start=True)

    def gripper_handle(self,req):
        resp = SetBoolResponse()
        if req.data:
            data = struct.pack("%dB"%(len(s_open)),*s_open)
            ser.write(data) 
            resp.success = True
            resp.message = 'openGripper'
        else:
            data = struct.pack("%dB"%(len(s_close)),*s_close)
            ser.write(data) 
            resp.success = True
            resp.message = 'closeGripper'
        return resp
    
    def ArriveCheck(self):
        Flag_Arrive=1 # 1 means arrived
        if self.HighAccuracy == 0:
            Err_limit_T=0.005
            Err_limit_R=0.01
        else:
            Err_limit_T=0.0002
            Err_limit_R=0.001
        for i in range(self.JointSize):
            if i==0 or i==5 or i==6: # Translate
                if abs(self.GoalPos[i]-self.joint_states.position[i])>=Err_limit_T:
                    Flag_Arrive=0
            else: # Rotate
                if abs(self.GoalPos[i]-self.joint_states.position[i])>=Err_limit_R:
                    Flag_Arrive=0
        return Flag_Arrive

    def SendPosCmd(self):
        if self.ArriveCheck()==1:
            self.JointCmdVel=[0,0,0,0,0,0,0]
            message = self.packRobotCommand()
            self.socket_communication.send(message)
        else:
            print('It has not arrived!')
            for i in range(self.JointSize):
                if i==0: # Translate
                    Vel_limit=0.1
                elif i==5: # Translate
                    Vel_limit=0.04
                elif i==6: # Translate
                    Vel_limit=0.04
                else: # Rotate
                    Vel_limit=0.7
                self.JointCmdVel[i] = 2*(self.GoalPos[i]-self.joint_states.position[i])
                if self.JointCmdVel[i] > Vel_limit:
                    self.JointCmdVel[i] = Vel_limit
                elif self.JointCmdVel[i] < -Vel_limit:
                    self.JointCmdVel[i] = -Vel_limit
                self.JointCmdMod[i] = CONTROL_MODE.CONTROL_MODE_VELOCITY
            vel=self.JointCmdVel # Moveit order
            self.JointCmdVel=[vel[0],vel[5],vel[6],vel[1]*180/np.pi,vel[2]*180/np.pi,vel[3]*180/np.pi,vel[4]*180/np.pi] # Low machine order
            self.ControlSpace = CONTROL_SPACE.CONTROL_SPACE_JOINT
            self.Command = ROBOT_STATE.RUNNING_STATE_ONLINE
            message = self.packRobotCommand()
            self.socket_communication.send(message)
    
    def ManualPosCmd_handle(self, msg): # The unit is m,rad
        print('Manual mode')
        tempP=msg.Position
        self.GoalPos=[tempP[0],tempP[1],tempP[2],tempP[3],tempP[4],tempP[5],tempP[6]]
        self.HighAccuracy=msg.HighAccuracy
        
    def on_goal_arm(self, goal_handle): # Moveit setting: 240 374.5 175
        print('Arm: on goal')
        goal_handle.set_accepted()
        goal_handle.set_succeeded()
        for i in range(len(goal_handle.get_goal().trajectory.points)):
            self.GoalPos[0:5]=goal_handle.get_goal().trajectory.points[i].positions
            time.sleep(0.2)
    
    def on_goal_hand(self, goal_handle):
        print('Hand: on goal')
        goal_handle.set_accepted()
        goal_handle.set_succeeded()
        for i in range(len(goal_handle.get_goal().trajectory.points)):
            self.GoalPos[5:7]=goal_handle.get_goal().trajectory.points[i].positions
            time.sleep(0.2)
    
    def on_cancel_arm(self, goal_handle):
        print('Arm: on cancel')

    def on_cancel_hand(self, goal_handle):
        print('Hand: on cancel')
    
    def pub_joint_states(self):
        data = self.socket_communication.recieve(flag=1)
        if data:
            self.unpackRobotState(data.strip("b'")) 
            pos = self.JointCurPos[:] # Low machine order
            self.joint_states.position = [pos[0]*180/np.pi,pos[3],pos[4],pos[5],pos[6],pos[1]*180/np.pi,pos[2]*180/np.pi] # Moveit order
            vel = self.JointCurVel[:]
            self.joint_states.velocity = [vel[0],vel[3],vel[4],vel[5],vel[6],vel[1],vel[2]]
            eff = self.JointCurTor[:]
            self.joint_states.effort   = [eff[0],eff[3],eff[4],eff[5],eff[6],eff[1],eff[2]]
        self.joint_states.header.stamp = rospy.Time.now()
        self.pub.publish(self.joint_states)
    
if __name__ == '__main__':
    try:
        rospy.init_node('curi_ros_driver')
        myclass = curi_ros_driver()
        while not rospy.is_shutdown():
            myclass.pub_joint_states()
            myclass.SendPosCmd()
            myclass.rate.sleep()
    except rospy.ROSInterruptException:
        myclass.Command = ROBOT_STATE.RUNNING_STATE_HOLDON
        message = myclass.packRobotCommand()
        myclass.socket_communication.send(message)
        myclass.socket_communication.close()