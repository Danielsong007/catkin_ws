import sys
import select
import socket
from communication.curi_communication import curi_communication

class curi_communication_socket(curi_communication):
    name = 'udp'
    def __init__(self, joint_size, selfIP, selfPort, targetIP, targetPort):
        curi_communication.__init__(self)
        self.JointSize = joint_size
        self.self_IP = selfIP
        self.self_Port = selfPort
        self.target_Address = (targetIP, targetPort)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
        return
    
    def open(self):
        # self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((self.self_IP, self.self_Port))
        print('open socket')

    def close(self):
        print('close socket')
        self.s.close()

    def send(self, massage):
        self.s.sendto(massage.encode("utf-8"), self.target_Address)
        
    def recieve(self, flag = 0):
        if flag == 1:
            readable = select.select([self.s], [], [], 0.001)[0]
            buf = ""
            if readable:
                for a in readable:
                    buf = str(a.recvfrom(256)[0])
            return buf
        
        readable = select.select([self.s], [], [], 0.001)[0]
        if readable:
            buf = ""
            for a in readable:
                buf = str(a.recvfrom(256)[0])
                print('buf[]', buf)
            JointValue = []
            buf = buf.strip("b'")
            cnt = buf.count("$")
            if cnt:
                x = buf.split("$", cnt)
                print(cnt, x, buf)
                for i in range(0, cnt, 1):
                    JointValue.append(float(x[i+1]))
                return [x[0], JointValue]
            else:
                return [buf, []]
        else:
            return [0, []]

    def set_position(self, JointPos):
        massage = "movej"
        for i in range(self.JointSize):
            massage += "$" + str(JointPos[i]) 
        # print('str_JointPos', massage)
        self.send(massage)

    def set_velocity(self, JointVel):
        massage = "speedj"
        for i in range(self.JointSize):
            massage += "$" + str(JointVel[i]) 
        # print('str_JointPos', massage)
        self.send(massage)

    def get_position(self):
        # print('get joint pos', self.recieve()[1])
        return self.recieve()[1]

    def set_start(self):
        self.send("start")

    def set_stop(self):
        self.send("stop")

'''
import time
if __name__ == '__main__':
    try:
        CS = curi_socket("", 10086)
        CS.open()
        for i in range(10):
            print('i', CS.recieve())
            time.sleep(0.05)
    except:
        print('exit')
    CS.close()
'''
