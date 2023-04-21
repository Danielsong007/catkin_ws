#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolResponse
import serial
import struct
from beginner_tutorials.msg import Num

import re, json, sys, time
sys.path.append("..")
from communication.curi_communication_socket import curi_communication_socket
from base.curi_robot_control import curi_robot_control, robot, ROBOT_STATE, CONTROL_SPACE, CONTROL_MODE
from communication.curi_ros_trajectory_action import curi_ros_trajectory_action
import numpy as np

t = 0
s_open = [0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A]
s_close = [0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA]

ser = serial.Serial(
        port='/dev/usb_0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
        )

class curi_ros_driver(robot):
    def __init__(self, config_file = 'defualt.json'):
        with open(config_file, 'r') as fpcfg:
            config = json.load(fpcfg)

        rospy.init_node('curi_ros_driver')
        self.dt = 0.05
        self.rate = rospy.Rate(int(1.0/self.dt))
        self.JointSize = 7
        
        self.joint_states = JointState()
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.position = [0.0] * self.JointSize
        self.joint_states.velocity = [0.0] * self.JointSize
        self.joint_states.effort = [0] * self.JointSize
        self.joint_states.name = config['robot'][0]['joint_name']

        robot_ip    = config['robot'][0]['robot_ip']
        robot_port  = config['robot'][0]['robot_port']
        robot_ip2   = config['robot'][0]['robot_ip2']
        robot_port2 = config['robot'][0]['robot_port2']
        self.socket_communication = curi_communication_socket(self.JointSize, robot_ip, robot_port, robot_ip2, robot_port2) #SOCK_DGRAM udp model 
        
        robot.__init__(self, self.JointSize, np.array([0.0] * self.JointSize))
        self.ControlSpace = CONTROL_SPACE.CONTROL_SPACE_NONE
        self.JointTarPos = [0.0] * self.JointSize
        self.JointLasPos = [0.0] * self.JointSize
        self.JointLasVel = [0.0] * self.JointSize

        self.is_braked_ = False
        self.pub = rospy.Publisher('robot/joint_states', JointState, queue_size=5)
        self.sub_new=rospy.Subscriber('chatter', Num, self.recieve_script_new)
        self.gripper_srv_ = rospy.Service('/gripper/run', SetBool, self._gripper_handle)

    def _gripper_handle(self,req):
        resp = SetBoolResponse()
        if req.data:
            data = struct.pack("%dB"%(len(s_open)),*s_open)
            ser.write(data) 
            self.is_braked_ = True
            resp.success = True
            resp.message = 'openGripper'
            rospy.logwarn('openVaccumGripper Grasp')
        else:
            data = struct.pack("%dB"%(len(s_close)),*s_close)
            ser.write(data) 
            self.is_braked_ = False
            resp.success = True
            resp.message = 'closeGripper'
            rospy.logwarn('closeVaccumGripper Drop')           
        return resp

    def recieve_script_new(self, msg):
        GoalPos=msg.num # The unit is /m,/deg; no matter Pos or Vel; eg.[0.1,0,0,-90,90,-90,0]
        # GoalPos=[-0.37,0.08,0,0,0,90,0]
        for i in range(self.JointSize): 
            if i in range(3): # Translate
                Vel_limit=0.04
                Err_limit=0.003
                print(i)
            else: # Rotate
                Vel_limit=10
                Err_limit=0.3
            self.JointCmdVel[i] = GoalPos[i]-self.JointCurPos[i]*180/np.pi
            if self.JointCmdVel[i] > Vel_limit:
                self.JointCmdVel[i] = Vel_limit
            elif self.JointCmdVel[i] < -Vel_limit:
                self.JointCmdVel[i] = -Vel_limit
            if abs(GoalPos[i]-self.JointCurPos[i]*180/np.pi) < Err_limit:
                self.JointCmdVel[i]=0
            # print('Cur_Pos_Err: i=', i, GoalPos[0]-self.JointCurPos[0]*180/np.pi)
            self.JointCmdMod[i] = CONTROL_MODE.CONTROL_MODE_VELOCITY
        self.ControlSpace = CONTROL_SPACE.CONTROL_SPACE_JOINT
        self.Command = ROBOT_STATE.RUNNING_STATE_ONLINE
        message = self.packRobotCommand()
        self.socket_communication.send(message)

    def run(self):
            self.socket_communication.open()
            self.action_server = None
            while not rospy.is_shutdown():
                data = self.socket_communication.recieve(flag=1)
                if data:
                    self.unpackRobotState(data.strip("b'"))
                    self.joint_states.header.stamp = rospy.Time.now()
                    self.joint_states.position = self.JointCurPos[:]
                    self.joint_states.velocity = self.JointCurVel[:]
                    self.joint_states.effort = self.JointCurTor[:]
                self.pub.publish(self.joint_states)
                self.rate.sleep()
            self.Command = ROBOT_STATE.RUNNING_STATE_HOLDON
            message = self.packRobotCommand()
            self.socket_communication.send(message)
            self.socket_communication.close()
    
if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            node = curi_ros_driver(sys.argv[1])
        else:
            node = curi_ros_driver()
        node.run()
    except rospy.ROSInterruptException:
        pass