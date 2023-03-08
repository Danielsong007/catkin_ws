# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

from communication.curi_communication import curi_communication

class curi_communication_sim(curi_communication):
    # static varable for global parameter, this can achieve different nodes in the same sim software/pc or in different software/pc
    serverSize = 0
    clientID = []
    IP = []
    Port = []
    counter = []
    
    def __init__(self, ip, port, joint_name):
        self.JointSize = len(joint_name)
        curi_communication.__init__(self.JointSize)
        # ['Revolute_joint1#', 'Revolute_joint2#', 'Revolute_joint3#', 'Revolute_joint4#', 'Revolute_joint5#', 'Revolute_joint6#', 'Revolute_joint7#']
        self.JointName = joint_name
        self.jointHandle = [0] * self.JointSize
        find_id = -1
        if curi_communication_sim.serverSize:
            for i in range(curi_communication_sim.serverSize):
                if ip == curi_communication_sim.IP[i] and port == curi_communication_sim.Port[i]:
                    find_id = i
                    break
        if -1 == find_id:
            curi_communication_sim.clientID.append(-1)
            curi_communication_sim.IP.append(ip)
            curi_communication_sim.Port.append(port)
            curi_communication_sim.counter.append(0)
            find_id = curi_communication_sim.serverSize
            curi_communication_sim.serverSize += 1
        self.index = find_id
        return
    
    def open(self):
        print ('sim communication started')
        # Now retrieve streaming data (i.e. in a non-blocking fashion):
        print(self.index, curi_communication_sim.counter)
        if 0 >= curi_communication_sim.counter[self.index]:
            # sim.simxFinish(-1)
            curi_communication_sim.clientID[self.index] = sim.simxStart(curi_communication_sim.IP[self.index], \
                                                                          curi_communication_sim.Port[self.index], \
                                                                          True, True, 5000, 5) # Connect to V-R
        curi_communication_sim.counter[self.index] += 1
        print('sim connection number', self.index, curi_communication_sim.counter[self.index])
        for i in range(self.JointSize):
            res, self.jointHandle[i] = sim.simxGetObjectHandle(curi_communication_sim.clientID[self.index], \
                                                                self.JointName[i], sim.simx_opmode_oneshot_wait)
        print('self.jointHandle', self.jointHandle)
        #sim.simxSynchronous(self.CR2_clientID, True)
    
    def close(self):
        # Now send some data to V-REP in a non-blocking fashion:
        sim.simxAddStatusbarMessage(curi_communication_sim.clientID[self.index], 'close V-REP!', sim.simx_opmode_oneshot_split)
        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(curi_communication_sim.clientID[self.index])
        # Now close the connection to V-REP:
        curi_communication_sim.counter[self.index] -= 1
        if 0 >= curi_communication_sim.counter[self.index]:
            sim.simxFinish(curi_communication_sim.clientID[self.index])
        print ('sim communication ended')
        return
    
    def set_position(self, JointPos):
        #returnCode,data=sim.simxGetJointPosition(CR2_clientID, target1, sim.simx_opmode_oneshot_split) # Try to retrieve the streamed data
        #if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        returnCode = [0, 0, 0, 0, 0, 0, 0]
        print(self.index, self.jointHandle, 'JointPos', JointPos)
        for i in range(self.JointSize):
            returnCode[i] = sim.simxSetJointPosition(curi_communication_sim.clientID[self.index], self.jointHandle[i], JointPos[i], sim.simx_opmode_streaming)
        # print(returnCode)
        #sim.simxSynchronousTrigger(self.CR2_clientID)
        return

    def get_position(self):
        returnValue = [0, 0, 0, 0, 0, 0, 0]
        for i in range(self.JointSize):
            res, returnValue[i] = sim.simxGetJointPosition(curi_communication_sim.clientID[self.index], self.jointHandle[i], sim.simx_opmode_streaming)
        return returnValue
