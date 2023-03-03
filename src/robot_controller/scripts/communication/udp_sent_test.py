import sys
import time
from communication.curi_communication_socket import *
import numpy

test_size = 20
output = []
for i in range(test_size):
    communication = curi_communication_socket(7, "127.0.0.1", 10100 + i, "127.0.0.1", 10200 + i) #SOCK_DGRAM udp model
    output.append(communication)

for i in range(test_size):
    output[i].open()

test_cases = 100
for i in range(test_cases):
    for j in range(test_size):
        output[j].set_position([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] * i * 10 + j)
    time.sleep(0.05)

for i in range(test_size):
    output[i].close()
