#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolResponse
import serial
import struct
from beginner_tutorials.msg import Num
# import thread


import re, json, sys, time, numpy
sys.path.append("..")
from communication.curi_communication_socket import curi_communication_socket
from base.curi_robot_control import curi_robot_control, robot, ROBOT_STATE, CONTROL_SPACE, CONTROL_MODE
from communication.curi_ros_trajectory_action import curi_ros_trajectory_action
import numpy as np
from pykalman import KalmanFilter

kf = KalmanFilter(transition_matrices=np.array([[1,0,1,0], [0,1,0,1],[0,0,1,0], [0,0,0,1]]),
                  observation_matrices =np.array([[1,0,0,0],[0,1,0,0]]),
                  transition_covariance= 0.01*np.eye(4))

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
        # read configuration file to get robot parameters and display parameters
        with open(config_file, 'r') as fpcfg:
            config = json.load(fpcfg)

        rospy.init_node('curi_ros_driver')
        self.dt = 0.05
        self.rate = rospy.Rate(int(1.0/self.dt))
        
        self.JointSize = config['robot'][0]['joint_size']
        
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
        
        robot.__init__(self, self.JointSize, numpy.array([0.0] * self.JointSize))
        self.ControlSpace = CONTROL_SPACE.CONTROL_SPACE_NONE
        self.JointTarPos = [0.0] * self.JointSize
        self.JointLasPos = [0.0] * self.JointSize
        self.JointLasVel = [0.0] * self.JointSize

        self.is_braked_ = False
        self.pub = rospy.Publisher('robot/joint_states', JointState, queue_size=5)
        self.sub = rospy.Subscriber('CURIScript', String, self.recieve_script)
        self.sub_new=rospy.Subscriber('chatter', Num, self.recieve_script_new)
        self.gripper_srv_ = rospy.Service('/gripper/run', SetBool, self._gripper_handle)
    #     thread.start_new_thread(command_thread,())


    # def command_thread(self):
    #     self.gripper_srv_ = rospy.Service('/gripper/run', SetBool, self._gripper_handle)
    #     rospy.spin()

    def _gripper_handle(self,req):
        resp = SetBoolResponse()
        if req.data:
            # activate vacuum gripper
            data = struct.pack("%dB"%(len(s_open)),*s_open)
            ser.write(data) 
            #gripper_.lock()
            # activate brake
            #brake.lock()
            self.is_braked_ = True
            resp.success = True
            resp.message = 'openGripper'
            rospy.logwarn('openVaccumGripper Grasp')
        else:
            # unactivate vacuum gripper
            data = struct.pack("%dB"%(len(s_close)),*s_close)
            ser.write(data) 
            #gripper_.unlock()
            # unactivate brake
            #brake.lock()
            self.is_braked_ = False
            resp.success = True
            resp.message = 'closeGripper'
            rospy.logwarn('closeVaccumGripper Drop')           
        return resp

    def recieve_script_new(self, msg):
        KKKK=[0,0,0,0,0,0.05]
        rospy.loginfo(KKKK)

        self.ControlSpace = CONTROL_SPACE.CONTROL_SPACE_JOINT
        for i in range(self.JointSize):
            self.JointCmdVel[i] = float(KKKK[i])
            self.JointCmdMod[i] = CONTROL_MODE.CONTROL_MODE_VELOCITY
            # self.JointCmdPos[i] = float(KKKK[i])
            # self.JointCmdMod[i] = CONTROL_MODE.CONTROL_MODE_POSITION

        self.Command = ROBOT_STATE.RUNNING_STATE_ONLINE
        message = self.packRobotCommand()
        self.socket_communication.send(message)

    def recieve_script(self, curi_script1):
        curi_script = curi_script1
        cmd = curi_script.data.split('(')
        data = re.findall(r"\d+\.?\d*", curi_script.data)
        if self.JointSize > len(data):
            print('data size error')
            return
        if cmd[0] == 'speedj':
            self.ControlSpace = CONTROL_SPACE.CONTROL_SPACE_JOINT
            for i in range(self.JointSize):
                self.JointCmdVel[i] = float(data[i])
                self.JointCmdMod[i] = CONTROL_MODE.CONTROL_MODE_VELOCITY
        elif cmd[0] == 'movej':
            self.ControlSpace = CONTROL_SPACE.CONTROL_SPACE_JOINT
            for i in range(self.JointSize):
                self.JointCmdPos[i] = float(data[i])
                self.JointCmdMod[i] = CONTROL_MODE.CONTROL_MODE_POSITION
        elif cmd[0] == 'speeda':
            return
        elif cmd[0] == 'movea':
            return
        self.Command = ROBOT_STATE.RUNNING_STATE_ONLINE
        message = self.packRobotCommand()
        self.socket_communication.send(message)

    def run(self):
            self.socket_communication.open()
            self.action_server = None
            t = 0
            flag = 0
            flag_1 = 0
            a = numpy.array([[1.0,1.0,1.0,1.0,1.0]])
            time_now = 0.0
            time_las = time_now
            time_dt = 0.0
            while not rospy.is_shutdown():
                # get robot state
                data = self.socket_communication.recieve(flag=1)
                if data:
                    self.unpackRobotState(data.strip("b'"))
                    self.joint_states.header.stamp = rospy.Time.now()
                    self.joint_states.position = self.JointCurPos[:]
                    self.joint_states.velocity = self.JointCurVel[:]
                    self.joint_states.effort = self.JointCurTor[:]
                    self.ConnectRobot = True 
                
                if self.ConnectRobot:
                    if not self.action_server:
                        self.Command = ROBOT_STATE.RUNNING_STATE_ONLINE
                        message = self.packRobotCommand()
                        self.socket_communication.send(message)
                        self.action_server = curi_ros_trajectory_action(self.JointSize, self.joint_states.name, self.dt)
                        self.action_server.ConnectRobot = self.ConnectRobot
                        self.action_server.start()
                        self.JointLasPos = self.JointCurPos.copy()
                        self.JointCmdPos = self.JointCurPos.copy()
                    else:
                        self.Command = ROBOT_STATE.RUNNING_STATE_ONLINE
                        self.JointCmdPos = self.action_server.JointCmdPos.copy()
                        self.action_server.JointCurPos = self.JointCurPos
                        self.action_server.JointCurVel = self.JointCurVel
                        time_now = self.action_server.tx
                        time_dt = time_now - time_las
                        if time_dt == 0:
                            flag_1 +=1
                            if flag_1 >1:
                                self.JointCmdVel = (self.JointCmdPos - self.JointLasPos) / self.dt + 0.01 * (self.JointCmdPos - self.JointCurPos) / self.dt
                            else:
                                self.JointCmdVel = self.JointLasVel
                        elif time_dt > 0.07 or time_dt < -0.3:
                            self.JointCmdVel = self.JointLasVel
                        else:
                            flag_1 = 0
                            self.JointCmdVel = (self.JointCmdPos - self.JointLasPos) / self.dt + 0.01 * (self.JointCmdPos - self.JointCurPos) / self.dt
                        #self.JointCmdVel = (self.JointCmdPos - self.JointLasPos) / time_dt    #+ 0.01 * (self.JointCmdPos - self.JointCurPos) / self.dt
                        
                        #print(type(b))

                        global filtered_state_means0,filtered_state_covariances0,lmx,lmy,lpx,lpy
                        current_measurement = np.array([np.float32(self.JointCurVel[0]),np.float32(self.JointCurVel[1])])
                        cmx, cmy = current_measurement[0], current_measurement[1]
                        if t == 0:
                            filtered_state_means0=np.array([0.0,0.0,0.0,0.0])
                            filtered_state_covariances0=np.eye(4)
                            lmx, lmy = 0.0, 0.0
                            lpx, lpy = 0.0, 0.0
                        filtered_state_means, filtered_state_covariances= (kf.filter_update( filtered_state_means0,filtered_state_covariances0,current_measurement))
                        cpx,cpy= filtered_state_means[0], filtered_state_means[1]
                        filtered_state_means0, filtered_state_covariances0=filtered_state_means, filtered_state_covariances
                        t=t+1
                        lpx, lpy = filtered_state_means[0], filtered_state_means[1]
                        lmx, lmy =current_measurement[0], current_measurement[1]

                        a = numpy.append(a,[[flag, self.JointCmdPos[0], self.JointCurVel[0], lpx, time_now]], axis=0)
                        #if lpx != 0.0:
                        #    if abs(lpx - self.JointCmdVel[0])/abs(lpx) >= 80:
                        #        self.JointCmdVel[0] = lpx

                        #a = numpy.append(a,[[flag, self.JointCmdPos[0], self.JointLasPos[0], self.JointCmdVel[0], time_now]], axis=0)
                        flag+=1
                        #if flag == 3000:
                        #    print("write!!!!!!!!!!!!!")
                        #    f = open("a.txt",'wb')
                        #    numpy.savetxt("a.txt", a, fmt='%f',delimiter=' ')
                        #    f.close()
                        self.JointLasPos = self.JointCmdPos.copy()
                        self.JointLasVel = self.JointCmdVel
                        time_las = time_now
                        #if self.is_braked_:
                        #    self.JointCurVel[4] = 0.0
                        #    self.JointCurVel[5] = 0.0
                        message = self.packRobotCommand()

                        # self.socket_communication.send(message)

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