<<<<<<< HEAD
import sys
import time
from communication.curi_communication_socket import *

test_size = 20
output = []
for i in range(test_size):
    communication = curi_communication_socket(7, "127.0.0.1", 10200 + i, "127.0.0.1", 10100 + i) #SOCK_DGRAM udp model
    output.append(communication)

for i in range(test_size):
    output[i].open()

test_cases = 100
for i in range(test_cases):
    t0 = time.time()
    cnt = 0
    for i in range(test_size):
        data = output[i].recieve()
        cnt += data[0]
    t1 = time.time()
    print('get ', cnt, ' total time cost ', "%f"%(t1 - t0))
    time.sleep(0.05)

for i in range(test_size):
    output[i].close()
=======
import sys
import time
from communication.curi_communication_socket import *

test_size = 20
output = []
for i in range(test_size):
    communication = curi_communication_socket(7, "127.0.0.1", 10200 + i, "127.0.0.1", 10100 + i) #SOCK_DGRAM udp model
    output.append(communication)

for i in range(test_size):
    output[i].open()

test_cases = 100
for i in range(test_cases):
    t0 = time.time()
    cnt = 0
    for i in range(test_size):
        data = output[i].recieve()
        cnt += data[0]
    t1 = time.time()
    print('get ', cnt, ' total time cost ', "%f"%(t1 - t0))
    time.sleep(0.05)

for i in range(test_size):
    output[i].close()
>>>>>>> 9778bfa7beef6029b1ea946117aaae9e3d6c49e5
