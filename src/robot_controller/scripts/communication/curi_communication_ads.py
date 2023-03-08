import pyads
from communication.curi_communication import *
import numpy 

class curi_communication_ads(curi_communication):
    NAME = 'ads'
    Plc = None

    def __init__(self, joint_size, IP, Port):
        super().__init__(joint_size)
        if self.Plc ==  None:
            self.Plc = pyads.Connection(IP, Port)
        return
    
    def open(self):
        if self.Plc == None:
            return False
        self.Plc.open()
        if self.Plc.read_by_name('MAIN.RunningState', pyads.PLCTYPE_DINT) == 5:
            self.Plc.write_by_name('MAIN.RunningState', 6, pyads.PLCTYPE_DINT)
            return True
        return False
    
    def close(self):
        if self.Plc == None:
            return False
        if self.Plc.read_by_name('MAIN.RunningState', pyads.PLCTYPE_DINT) == 6:
            self.Plc.write_by_name('MAIN.RunningState', 5, pyads.PLCTYPE_DINT)
            return True
        self.Plc.close()
        return False
        
    def set_position_mode():
        self.Plc.write_by_name('MAIN.DriverCmdMod', [7] * 7, pyads.PLCTYPE_SINT * 7)
    
    def set_velocity_mode():
        self.Plc.write_by_name('MAIN.DriverCmdMod', [8] * 7, pyads.PLCTYPE_SINT * 7)
    
    def get_position(self):
        JointCurPos = self.Plc.read_by_name('MAIN.JointCurPos', pyads.PLCTYPE_LREAL * 7) # fast read and write
        return numpy.array(JointCurPos) * numpy.pi / 180.0
    
    def get_velocity(self):
        JointCurVel = self.Plc.read_by_name('MAIN.JointCurVel', pyads.PLCTYPE_LREAL * 7)
        return numpy.array(JointCurVel) * numpy.pi / 180.0
    
    def get_torque(self):
        JointCurTor = self.Plc.read_by_name('MAIN.JointCurTor', pyads.PLCTYPE_LREAL * 7)
        return numpy.array(JointCurTor) * numpy.pi / 180.0
    
    def set_position(self, JointCmdPos):
        self.Plc.write_by_name('MAIN.JointSetPos', list(JointCmdPos * 180.0 / numpy.pi), pyads.PLCTYPE_LREAL * 7)
        return
    
    def set_velocity(self, JointCmdVel):
        # self.Plc.write_by_name('MAIN.JointCmdPos', list(JointCmdPos * 180.0 / numpy.pi), pyads.PLCTYPE_LREAL * 7)
        self.Plc.write_by_name('MAIN.JointCmdVel', list(JointCmdVel * 180.0 / numpy.pi), pyads.PLCTYPE_LREAL * 7)
        return
    
    def set_torque(self, JointCmdTor):
        self.Plc.write_by_name('MAIN.JointSetTor', list(JointCmdTor), pyads.PLCTYPE_LREAL * 7)
        return