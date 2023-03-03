import sys
import time
import numpy
from interpolation import *
from curi_communication_socket import *

JointTarPos = numpy.array([0, 0, 0, 0, 0, 0, 0.0])
JointCurPos = numpy.array([0, 0, 0, 0, 0, 0, 0.0])
JointCmdPos = numpy.array([0, 0, 0, 0, 0, 0, 0.0])
JointCmdVel = numpy.array([0, 0, 0, 0, 0, 0, 0.0])
JOINT_SIZE = 7

socket_communication = curi_communication_socket(JOINT_SIZE, "172.16.253.38", 10087, "172.16.253.38", 10086) #SOCK_DGRAM udp model
socket_communication.open()

dt = 0.05
t = 0
T = 3

# joint targets position
if len(sys.argv) < 8:
    exit()
else:
    for i in range(JOINT_SIZE):
        JointTarPos[i] = float(sys.argv[i+1]) * numpy.pi / 180.0
print('JointTarPos: ', JointTarPos)

# interpolation
Interpolation = [None] * JOINT_SIZE
for i in range(JOINT_SIZE):
    Interpolation[i] = interpolation_1_cos(T, JointCurPos[i], JointTarPos[i])

# 
JointLasPos = JointCurPos.copy()
socket_communication.set_start()
while t < T:
    data = socket_communication.recieve()
    print(data)
    if data[0]:
        print('JointCurPos: ', numpy.array(data[1]) * 180 / numpy.pi)

    for i in range(JOINT_SIZE):
        JointCmdPos[i] = Interpolation[i].get_position(t)
    JointCmdVel = (JointCmdPos - JointLasPos) / dt

    print('JointCmdVel: ', JointCmdVel)
    socket_communication.set_velocity(JointCmdVel)
    
    JointLasPos = JointCmdPos.copy()
    t += dt
    time.sleep(dt)

# socket_communication.set_velocity(JointTarPos)
# t = 0
# while t < T:
#     t += dt
#     time.sleep(dt)

socket_communication.set_stop()
time.sleep(5)
socket_communication.close()