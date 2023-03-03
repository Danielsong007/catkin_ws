import pyads, time
from numpy import *

# connect to the PLC


JointCurPos = array([0, 0, 0, 0, 0, 0]);

count = 0;
max_c = 100;
try:
    while True:
        RunningState = plc.read_by_name('MAIN.RunningState', pyads.PLCTYPE_DINT);
        if RunningState == 5:
            plc.write_by_name('MAIN.RunningState', 6, pyads.PLCTYPE_DINT);
        elif RunningState == 6:
            for i in range(0, 5):
                JointCurPos[i] = 
            print('JointCurPos', JointCurPos);
            count = count + 1;
            if count >= max_c:
                count = 0;
            for i in range(0, 5):
                plc.write_by_name('MAIN.JointCmdPos[' + str(i) + ']', 100*sin(2.0*pi*count/max_c), pyads.PLCTYPE_LREAL);
        # sleep for 1 seconds
        time.sleep(0.02);
except:
    plc.write_by_name('MAIN.RunningState', 5, pyads.PLCTYPE_DINT);
    plc.close();