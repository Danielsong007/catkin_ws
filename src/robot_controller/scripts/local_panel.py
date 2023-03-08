import sys
from base.curi_robot_control import curi_robot_control
from communication.curi_communication_socket import curi_communication_socket
from base.ui_block import *
import numpy
import time
import datetime
import json
import importlib

class robot_ui_v2(QWidget):
    def __init__(self, config_file = 'defualt.json'):
        super().__init__()
        self.initUI(config_file)
    
    def initUI(self, config_file):
        # read configuration file to get robot parameters and display parameters
        with open(config_file, 'r') as fpcfg:
            config = json.load(fpcfg)

        self.UI = config['display'][0]['ui']
        self.setGeometry(self.UI[1], self.UI[2], self.UI[3], self.UI[4])
        self.setWindowTitle(self.UI[0])
        dt = config['robot'][0]['dt']
        
        # robot parameter
        joint_size = config['robot'][0]['joint_size']
        joint_type = numpy.array(config['robot'][0]['joint_type'])
        a          = numpy.array(config['robot'][0]['DH_a'])
        alpha      = numpy.array(config['robot'][0]['DH_alpha']) * numpy.pi / 180
        d_base     = numpy.array(config['robot'][0]['DH_d_base'])
        q_base     = numpy.array(config['robot'][0]['DH_q_base']) * numpy.pi / 180
        q_init     = numpy.array(config['robot'][0]['q_init']) * numpy.pi / 180
        joint_name = config['robot'][0]['joint_name']
        robot_type = config['robot'][0]['robot_type']
        robot_ip   = config['robot'][0]['robot_ip']
        robot_port = config['robot'][0]['robot_port']
        
        robot_communication = None
        if robot_type == 'sim':
            from communication.curi_communication_sim import curi_communication_sim
            robot_communication = curi_communication_sim(robot_ip, robot_port, joint_name)
        elif robot_type == 'ads':
            import curi_communication_ads
            robot_communication = curi_communication_ads(joint_size, robot_ip, robot_port)
        elif robot_type == 'udp':
            robot_ip2   = config['robot'][0]['robot_ip2']
            robot_port2 = config['robot'][0]['robot_port2']
            from communication.curi_communication_socket import curi_communication_socket
            robot_communication = curi_communication_socket(joint_size, robot_ip, robot_port, robot_ip2, robot_port2) #SOCK_DGRAM udp model 
        else:
            robot_communication = None
        self.RobotControl = curi_robot_control(joint_size, joint_type, a, alpha, d_base, q_base, q_init, dt, robot_communication)

        #communication
        local_ip    = config['robot'][0]['local_ip']
        local_port  = config['robot'][0]['local_port']
        remote_ip   = config['robot'][0]['remote_ip']
        remote_port = config['robot'][0]['remote_port']
        self.socket_communication = curi_communication_socket(joint_size, local_ip, local_port, remote_ip, remote_port) #SOCK_DGRAM udp model 
        self.RemoteControl = False
        self.GetRemoteCommand = False
        
        '''
        UI Parameter of CR2 
        '''
        # joint group
        JointName = config['display'][0]['joint_name']
        JointLBoundary = config['display'][0]['joint_L_limit']
        JointRBoundary = config['display'][0]['joint_R_limit']
        JointPosition = config['display'][0]['joint_position']
        self.GroupJoint = groupV(self, JointPosition[0], JointPosition[1], self.RobotControl.JOINT_SIZE, JointName, JointLBoundary, JointRBoundary, self.enableJointControl)
        
        # task group
        TaskName = config['display'][0]['task_name']
        TaskLBoundary = config['display'][0]['task_L_limit']
        TaskRBoundary = config['display'][0]['task_R_limit']
        TaskPosition = config['display'][0]['task_position']
        self.GroupTask = groupV(self, TaskPosition[0], TaskPosition[1], 7, TaskName, TaskLBoundary, TaskRBoundary, self.enableTaskControl) 
        
        # others
        info_show = config['display'][0]['info_show_geometry']
        self.InfoShow = QLabel(self)
        self.InfoShow.setFrameStyle(QFrame.Panel | QFrame.Plain)
        self.InfoShow.setAlignment(Qt.AlignTop   | Qt.AlignLeft)
        self.InfoShow.setGeometry(info_show[0], info_show[1], info_show[2], info_show[3])
        
        control_block = config['display'][0]['control_block_geometry']
        self.ControlBlock = QLabel(self)
        self.ControlBlock.setFrameStyle(QFrame.Panel | QFrame.Plain)
        self.ControlBlock.setAlignment(Qt.AlignTop   | Qt.AlignLeft)
        self.ControlBlock.setGeometry(control_block[0], control_block[1], control_block[2], control_block[3])
        
        connect = config['display'][0]['connect_name_position']
        self.CheckBoxConnect = QCheckBox(connect[0], self)
        self.CheckBoxConnect.move(connect[1], connect[2])
        self.CheckBoxConnect.stateChanged.connect(self.connectRobot)
        
        rcm = config['display'][0]['rcm_name_position']
        self.CheckBoxRCM = QCheckBox(rcm[0], self)
        self.CheckBoxRCM.move(rcm[1], rcm[2])
        self.CheckBoxRCM.stateChanged.connect(self.rcm)
        
        run = config['display'][0]['run_name_position']
        self.CheckBoxRun = QCheckBox(run[0], self)
        self.CheckBoxRun.move(run[1], run[2])
        self.CheckBoxRun.stateChanged.connect(self.run)
        
        demo = config['display'][0]['demo_name_position']
        self.CheckBoxDemo = QCheckBox(demo[0], self)
        self.CheckBoxDemo.move(demo[1], demo[2])
        self.CheckBoxDemo.stateChanged.connect(self.demo)
        
        window_pale = QtGui.QPalette()
        self.setPalette(window_pale)

        self.timer = QTimer(self, timerType = Qt.PreciseTimer)
        self.timer.timeout.connect(self.timerEvent)
        self.timer.start(int(1000 * dt)) # 50 ms
        
        self.show()
    
    def connectRobot(self, state):
        if Qt.Checked == state:
            self.RobotControl.connectRobot()
            self.setWindowTitle(self.UI[0] + ': connected')
        else:
            self.RobotControl.disconnectRobot()
            self.setWindowTitle(self.UI[0] + ': unconnected')
    
    def enableJointControl(self):
        if False == self.GroupJoint.CheckBox_Enable.isChecked():
            return
        self.GroupTask.CheckBox_Enable.setChecked(False)
        self.RobotControl.switchJointControl()
        JointCurPos = self.RobotControl.JointCurPos * 180.0 / numpy.pi
        self.GroupJoint.itemValue[0:self.RobotControl.JOINT_SIZE] = JointCurPos.tolist()
        self.GroupJoint.update()
    
    def enableTaskControl(self):
        if False == self.GroupTask.CheckBox_Enable.isChecked():
            return
        self.GroupJoint.CheckBox_Enable.setChecked(False)
        self.RobotControl.switchTaskControl()
        TaskCurPos = self.RobotControl.TaskCurPos.copy()
        TaskCurPos[3:6] = TaskCurPos[3:6] * 180.0 / numpy.pi
        self.GroupTask.itemValue = TaskCurPos.tolist()
        self.GroupTask.update()
    
    def rcm(self):
        d_rcm = self.GroupTask.itemValue[6]
        eTr = self.RobotControl.MDH(0, 0, d_rcm, 0)
        bTr = numpy.dot(self.RobotControl.bTe, eTr)
        self.RobotControl.switchRCM(bTr, eTr)
        self.GroupTask.zero()
        self.GroupTask.itemValue[6] = d_rcm
        self.GroupTask.update()
    
    def demo(self):
        if False == self.CheckBoxDemo.isChecked():
            return
        # predefine trajectory
        demoBase = [[0, 0, 0, -1, -1, 0], [0, 0, 0, -1, 1, 0], [0, 0, 0, 1, 1, 0], [0, 0, 0, 1, -1, 0], [0, 0, 0, -1, -1, 0]]
        demoTraj = []
        for i in range(30, 40, 10):
            for a in demoBase:
                newYPRD = list(map(lambda x: x * i * numpy.pi / 180.0, a))
                #newYPRD[1] -= 10
                demoTraj.append(newYPRD)
        demoTraj.append([0, 0, 0, 0, 0, 0.0])
        self.RobotControl.startDemo(demoTraj)
        self.CheckBoxDemo.setEnabled(False)
        self.CheckBoxDemo.setChecked(False)
    
    def run(self):
        if False == self.CheckBoxRun.isChecked():
            return
        if self.GroupJoint.CheckBox_Enable.isChecked():
            JointTarPos = numpy.array(self.GroupJoint.itemValue[0:self.RobotControl.JOINT_SIZE]) * numpy.pi / 180.0
            self.RobotControl.startInterpolation(JointTarPos)
        elif self.GroupTask.CheckBox_Enable.isChecked():
            TaskTarPos = numpy.array(self.GroupTask.itemValue[0:7])
            TaskTarPos[3:6] = numpy.array(self.GroupTask.itemValue[3:6]) * numpy.pi / 180.0         
            self.RobotControl.startInterpolation(TaskTarPos)
        self.CheckBoxRun.setEnabled(False)
        self.CheckBoxRun.setChecked(False)
    
    def timerEvent(self):
        t0 = time.time()
        
        # get data
        if self.RemoteControl:
            recieve_data = self.socket_communication.recieve()
            self.GetRemoteCommand = recieve_data[0]
        else:
            self.socket_communication.open()
            self.RemoteControl = True
        
        # robot control and send data
        if self.RobotControl.OnTrajectory or self.RobotControl.demoRunning:
            self.CheckBoxRCM.setEnabled(False)
            self.CheckBoxRun.setEnabled(False)
            self.CheckBoxDemo.setEnabled(False)
        else:
            self.CheckBoxRCM.setEnabled(True)
            self.CheckBoxRun.setEnabled(True)
            self.CheckBoxDemo.setEnabled(True)

        if self.RemoteControl and self.GetRemoteCommand:
            self.RobotControl.run(True, recieve_data[1])
        else:
            self.RobotControl.run(False, [])

        # show data
        self.InfoShow.setText(self.RobotControl.ShowInfo)
        t4 = time.time()
        print('total time cost ', "%f"%(t4 - t0))
    
    def closeEvent(self, event):
        if self.CheckBoxConnect.checkState():
            self.CheckBoxConnect.setChecked(False)
        # self.fplog.close()
        self.socket_communication.close()
        event.accept()

if __name__ == '__main__':
    try:
        sys.path.append('./communication')
        app = QApplication(sys.argv)
        if len(sys.argv) > 1:
            ex_ = robot_ui_v2(sys.argv[1])
        else:
            ex_ = robot_ui_v2()
    except:
        print('exit')
        sys.exit(ex_.exec_())
    sys.exit(app.exec_())
