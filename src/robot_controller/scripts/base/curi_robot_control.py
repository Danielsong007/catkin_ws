from base.interpolation import interpolation_1_cos
from base.curi_robotics import curi_robotics
import numpy

class ROBOT_STATE():
    RUNNING_STATE_POWEROFF      = 0x00
    RUNNING_STATE_POWERON       = 0x01
    RUNNING_STATE_INITIAL       = 0x02
    RUNNING_STATE_SELFTEST      = 0x03
    RUNNING_STATE_SETUP         = 0x04
    RUNNING_STATE_BACKDRIVE     = 0x04
    RUNNING_STATE_HOLDON        = 0x05
    RUNNING_STATE_ONLINE        = 0x06
    RUNNING_STATE_REMOTE        = 0x07
    RUNNING_STATE_REACTIVE      = 0x08
    RUNNING_STATE_INTERPOLATION = 0x09

    RUNNING_STATE_SHUTDOWN      = 0xF0
    RUNNING_STATE_EMERGENCY     = 0xF1
    RUNNING_STATE_PROTECED      = 0xF3

class CONTROL_MODE():
    CONTROL_MODE_INTERPOLATION  = 0x07
    CONTROL_MODE_POSITION       = 0x08
    CONTROL_MODE_VELOCITY       = 0x09
    CONTROL_MODE_TORQUE         = 0x0A

class CONTROL_SPACE():
    CONTROL_SPACE_NONE          = 0x00
    CONTROL_SPACE_JOINT         = 0x01
    CONTROL_SPACE_TASK          = 0x02
    CONTROL_SPACE_RCM           = 0x03

class robot(): 
    def __init__(self, joint_size, q_init):
        self.JointSize = joint_size
        self.RunningState = ROBOT_STATE.RUNNING_STATE_HOLDON
        self.Command = ROBOT_STATE.RUNNING_STATE_HOLDON
        self.ConnectRobot = False
        self.CommunicationOpen = False
        
        self.JointCurPos = numpy.array(q_init)
        self.JointCurVel = numpy.array([0.0] * self.JointSize)
        self.JointCurTor = numpy.array([0.0] * self.JointSize)
        self.JointCurMod = numpy.array([CONTROL_MODE.CONTROL_MODE_VELOCITY] * self.JointSize)
        
        self.JointCmdPos = self.JointCurPos.copy()
        self.JointCmdVel = self.JointCurVel.copy()
        self.JointCmdTor = self.JointCurTor.copy()
        self.JointCmdMod = self.JointCurMod.copy()
        self.tx = 0

    def packRobotCommand(self):
        Message = str(self.Command) + "#"
        Message += str(self.JointSize) + "#"
        for i in range (self.JointSize):
            Message += str(self.JointCmdPos[i] / numpy.pi * 180.0) + "#"
        for i in range (self.JointSize):
            Message += str(self.JointCmdVel[i] / numpy.pi * 180.0) + "#"
        for i in range (self.JointSize):
            Message += str(self.JointCmdTor[i]) + "#"
        for i in range (self.JointSize):
            Message += str(self.JointCmdMod[i]) + "#"
        return Message

    def unpackRobotState(self, Message):
        x = Message.split("#", Message.count('#'))
        if int(x[0]) > 128 or int(x[0]) < 0:
            return
        self.RunningState = int(x[0])
        self.JointSize = int(x[1])
        index = 2
        for i in range (self.JointSize):
            self.JointCurPos[i] = float(x[index]) * numpy.pi / 180.0
            index += 1
        for i in range (self.JointSize):
            self.JointCurVel[i] = float(x[index]) * numpy.pi / 180.0
            index += 1
        for i in range (self.JointSize):
            self.JointCurTor[i] = float(x[index])
            index += 1
        for i in range (self.JointSize):
            self.JointCurMod[i] = float(x[index])
            index += 1
        '''
        print('Robot RunningState:', self.RunningState)
        print('Robot JOINT_SIZE:', self.JointSize)
        print('Robot JointCurPos:', self.JointCurPos)
        print('Robot JointCurVel:', self.JointCurVel)
        print('Robot JointCurTor:', self.JointCurTor)
        print('Robot JointCurMod:', self.JointCurMod)
        print('Robot Message:', x[index])
        '''

class curi_robot_control(curi_robotics, robot):
    def __init__(self, joint_size, joint_type, a, alpha, d, theta, q_init, dt, communication):
        curi_robotics.__init__(self, joint_size, joint_type, a, alpha, d, theta)
        robot.__init__(self, joint_size, numpy.array([0.0] * joint_size))

        self.RunningState = ROBOT_STATE.RUNNING_STATE_HOLDON
        self.Communication = communication

        self.JointTarPos = self.JointCurPos.copy()
        self.JointLasPos = self.JointCurPos.copy()
        self.JointZerPos = self.JointCurPos.copy()
        
        self.TaskCurPos = numpy.array([0.0] * 7)
        self.TaskBasPos = numpy.array([0.0] * 7)
        self.TaskCmdPos = numpy.array([0.0] * 7)
        self.TaskTarPos = numpy.array([0.0] * 7)

        self.LastRemoteData = numpy.array([0.0] * self.JointSize)

        # initial joint position for uterus manipulation
        self.dt = dt
        self.ConnectRobot = False
        self.ControlWithRCM = False

        # interpolation
        self.OnTrajectory = False
        self.InterpolationSumT = 3 # 5 s
        self.InterpolationNowT = 0
        self.Interpolation = [None] * self.JointSize
        
        self.demoRunning = False
        self.demoIndex = 0
        self.demoSize = 0
    
    def __del__(self):
        if self.Communication and self.ConnectRobot:
            if self.Communication.name == 'udp':
                self.Command = ROBOT_STATE.RUNNING_STATE_ONLINE
                message = self.packRobotCommand()
                self.Communication.send(message)
        if self.Communication and self.CommunicationOpen:
            self.Communication.close()
    
    def connectRobot(self):
        self.ConnectRobot = True
        self.Command = ROBOT_STATE.RUNNING_STATE_ONLINE
        if self.Communication:
            print('connection name', self.Communication.name)
            if not self.CommunicationOpen:
                self.Communication.open()
                self.CommunicationOpen = True
            if self.Communication.name == 'udp':
                message = self.packRobotCommand()
                self.Communication.send(message)
        self.JointCmdVel = numpy.array([0.0] * self.JointSize)
        self.JointCmdTor = numpy.array([0.0] * self.JointSize)
        self.ControlSpace = CONTROL_SPACE.CONTROL_SPACE_NONE

    def disconnectRobot(self):
        self.ConnectRobot = False
        self.Command = ROBOT_STATE.RUNNING_STATE_HOLDON
        if self.Communication:
            message = self.packRobotCommand()
            self.Communication.send(message)
        self.Command = ROBOT_STATE.RUNNING_STATE_SHUTDOWN
        if self.Communication:
            message = self.packRobotCommand()
            self.Communication.send(message)

    def switchJointControl(self):
        if self.OnTrajectory:
            return
        self.JointCmdPos = self.JointCurPos.copy()
        self.JointLasPos = self.JointCurPos.copy()
        self.ControlSpace = CONTROL_SPACE.CONTROL_SPACE_JOINT
        self.JointTarPos = numpy.array([0.0] * self.JointSize)
        
    def switchTaskControl(self):
        if self.OnTrajectory:
            return
        self.ControlSpace = CONTROL_SPACE.CONTROL_SPACE_TASK
        self.TaskTarPos = numpy.array([0.0] * 7) # needed for interpolation
        self.TaskCmdPos = self.TaskCurPos.copy()

    def switchRCM(self, bTr, eTr):
        if self.OnTrajectory:
            return
        self.bTr = bTr
        self.eTr = eTr
        self.TaskTarPos = numpy.array([0.0] * 7)
        self.ControlWithRCM = not self.ControlWithRCM
    
    def startDemo(self, demo):
        self.demoTrajectory = demo
        self.demoRunning = True
        self.demoIndex = 0
        self.demoSize = len(demo)
    
    def startInterpolation(self, TarPos):
        samePosition = True
        if CONTROL_SPACE.CONTROL_SPACE_JOINT == self.ControlSpace:
            self.JointTarPos = TarPos
            for i in range(self.JointSize):
                self.Interpolation[i] = interpolation_1_cos(self.InterpolationSumT, self.JointCurPos[i], self.JointTarPos[i])
                if abs(self.JointCurPos[i] - self.JointTarPos[i]) > 1e-3:
                    samePosition = False
        elif CONTROL_SPACE.CONTROL_SPACE_TASK == self.ControlSpace:
            if not self.ControlWithRCM:
                for i in range(3):
                    self.TaskBasPos[i] = self.bTe[i, 3]
                self.TaskBasPos[3:6] = self.Mat2RPY(self.bTe[0:3, 0:3])
            else:
                self.TaskBasPos = self.TaskTarPos.copy()
            self.TaskTarPos = TarPos.copy()
            for i in range(6):
                self.Interpolation[i] = interpolation_1_cos(self.InterpolationSumT, self.TaskBasPos[i], self.TaskTarPos[i])
                if i < 3 and abs(self.TaskBasPos[i] - self.TaskTarPos[i]) > 1e-3 or i >= 3 and abs(self.TaskBasPos[i] - self.TaskTarPos[i]) > 1e-1:
                    samePosition = False
        print('Tar', TarPos, self.JointCurPos, self.JointSize, samePosition)
        if False == samePosition:
            self.OnTrajectory = True
        self.InterpolationNowT = 0
    
    def run(self, RemoteControlFlag, recieveData):
        # get data
        if self.ConnectRobot and self.Communication:
            if self.Communication.name == 'udp':
                message = self.Communication.recieve(flag=1)
                if len(message) > self.JointSize:
                    self.unpackRobotState(message.strip("b'"))
                    print('message', message)
            else:
                self.JointCurPos = numpy.array(self.Communication.get_position()) # get data from remote robot (can be simulation)
        else:
            self.JointCurPos = self.JointCmdPos.copy() # simulation
        self.bTe = self.MFK(self.JointCurPos)
        self.TaskCurPos[0:3] = self.bTe[0:3, 3]
        self.TaskCurPos[3:6] = self.Mat2RPY(self.bTe[0:3, 0:3])
        #print('bTe', self.bTe)
        
        if self.Communication and self.Communication.get_adimitance_control_state():
            J = self.MIK(self.JointCurPos)
            if self.ControlWithRCM:
                J = J * Jrcm
            JointCmdVel = numpy.dot(numpy.linal.pinv(J), self.get_adimitance_control_state(recieveData))
            self.JointCmdPos = self.JointLasPos + JointCmdVel * self.dt
        elif RemoteControlFlag:
            # self.JointCmdPos = self.JointLasPos + JointCmdVel * self.dt
            self.JointCmdPos = self.JointCurPos.copy()
            if CONTROL_MODE.CONTROL_MODE_VELOCITY == self.JointCurMod[0]:
                if [] == recieveData:
                    JointCmdVel = self.LastRemoteData
                else:
                    JointCmdVel = numpy.array(recieveData)
                    self.LastRemoteData = JointCmdVel
            else:
                self.JointCmdPos = numpy.array(recieveData)
        else:
            JointCmdVel = numpy.array([0.0] * self.JointSize)
            # run demo
            if self.demoRunning:
                if self.demoIndex >= self.demoSize: # finish
                    self.demoRunning = False
                elif False == self.OnTrajectory:
                    self.startInterpolation(self.demoTrajectory[self.demoIndex])
                    self.demoIndex += 1
            
            # control
            if self.OnTrajectory:
                if self.InterpolationNowT < self.InterpolationSumT:
                    self.InterpolationNowT += self.dt
                    if CONTROL_SPACE.CONTROL_SPACE_JOINT == self.ControlSpace: # joint-space
                        for i in range(self.JointSize):
                            self.JointCmdPos[i] = self.Interpolation[i].get_position(self.InterpolationNowT)
                    elif rCONTROL_SPACE.CONTROL_SPACE_TASK == self.ControlSpace: # task-space
                        for i in range(6):
                            self.TaskCmdPos[i] = self.Interpolation[i].get_position(self.InterpolationNowT)
                        TaskCmdR = self.RPY2Mat(self.TaskCmdPos[3:6])
                        print('TaskCmdR', TaskCmdR)
                        if not self.ControlWithRCM:
                            self.JointCmdPos = self.MIK(TaskCmdR, self.TaskCmdPos[0:3], self.JointZerPos)
                        else:
                            rTe = numpy.linalg.pinv(self.eTr)
                            rTe[2, 3] = rTe[2, 3] - self.TaskCmdPos[2]
                            T = numpy.eye(4)
                            T[0:3, 0:3] = TaskCmdR
                            bTe = numpy.dot(numpy.dot(self.bTr, T), rTe)
                            self.JointCmdPos = self.MIK(bTe[0:3, 0:3], bTe[0:3, 3], self.JointZerPos)
                else:
                    self.InterpolationNowT = 0
                    self.OnTrajectory = False
                JointCmdVel = (self.JointCmdPos - self.JointLasPos) / self.dt
            else:
                self.JointCmdPos = self.JointLasPos.copy()
            
            # velocity boundary
            max_velocity = 0.0
            for i in range(self.JointSize):
                if max_velocity < abs(JointCmdVel[i]):
                    max_velocity = abs(JointCmdVel[i])
            if max_velocity < 0.01: # or max_velocity > 2.0:
                JointCmdVel = numpy.array([0.0] * self.JointSize)

        if self.ConnectRobot and self.Communication:
            if self.Communication.name == 'udp':
                if CONTROL_MODE.CONTROL_MODE_VELOCITY == self.JointCurMod[0]:
                    self.JointCmdVel = JointCmdVel + 0.01 * (self.JointCmdPos - self.JointCurPos) / self.dt # position feedback
                if CONTROL_SPACE.CONTROL_SPACE_JOINT == self.ControlSpace: 
                    message = self.packRobotCommand()
                    self.Communication.send(message)
            elif CONTROL_MODE.CONTROL_MODE_POSITION == self.JointCurMod[0]:
                self.Communication.set_position(self.JointCmdPos)
            elif CONTROL_MODE.CONTROL_MODE_VELOCITY == self.JointCurMod[0]:
                JointCmdVel = JointCmdVel + 0.3 * (self.JointCmdPos - self.JointCurPos) / self.dt # position feedback
                self.Communication.set_velocity(JointCmdVel)
            # TODO: add torque control
        
        self.JointLasPos = self.JointCmdPos.copy()
        self.ShowInfo = self.showData(RemoteControlFlag)
    
    def showData(self, recieve_data_flag):
        Info = '    Joint data  :   position      velocity       torque\n'
        for i in range(self.JointSize):
            Info = Info + '       |_ Joint' + str(i+1) + ':' + "{:10.4f}".format(self.JointCurPos[i] * 180.0 / numpy.pi)
            Info = Info + '  |' + "{:10.4f}".format(self.JointCurVel[i] * 180.0 / numpy.pi) + '  |' + "{:10.4f}".format(self.JointCurTor[i]) + '\n'
        Info = Info + '\n    End Effector data:\n'
        Info = Info + '       |_ X:    ' + "{:10.4f}".format(self.TaskCurPos[0]) + '\n'
        Info = Info + '       |_ Y:    ' + "{:10.4f}".format(self.TaskCurPos[1]) + '\n'
        Info = Info + '       |_ Z:    ' + "{:10.4f}".format(self.TaskCurPos[2]) + '\n'
        Info = Info + '       |_ Yaw:  ' + "{:10.2f}".format(self.TaskCurPos[3] * 180.0 / numpy.pi) + '\n'
        Info = Info + '       |_ Pitch:' + "{:10.2f}".format(self.TaskCurPos[4] * 180.0 / numpy.pi) + '\n'
        Info = Info + '       |_ Roll: ' + "{:10.2f}".format(self.TaskCurPos[5] * 180.0 / numpy.pi) + '\n'
        if self.OnTrajectory:
            Info = Info + '\n\n        On Trajectery Planning: \n' + "            {:10.4f}".format(self.InterpolationNowT) + "/" +  "{:10.4f}".format(self.InterpolationSumT) + "s \n"
        if recieve_data_flag:
            Info = Info + '\n\n        Get Remote Control Command\n'

        return Info