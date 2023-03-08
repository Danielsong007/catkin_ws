#!/usr/bin/env python
from __future__ import print_function
import numpy as np
from rotools.utility import transform

def deg_to_rad(deg):
    return(print(deg*np.pi/180))

if __name__ == "__main__":
    # Rx90 = np.array([[1,0,0,0],
    #                 [0,-1,0,0],
    #                 [0,0,-1,0],
    #                 [0,0,0,1]])
    # print(Rx90[2])                
    # Xaxis = np.array([1, 0, 0])
    # # Xaxis = np.array([[1], [0], [0]])
    # print(Xaxis)
    # dot_prod = np.dot(Xaxis,Rx90[0:3,2])
    # print(Rx90[0:3,2])
    # print(dot_prod)

    # print(np.zeros((4,4)))

    # print(Rx90[0:3,2].shape)
# # left grasp
#     matrix = [[0, 0, 1, 0],
#               [1, 0, 0, 0],
#               [0, 1, 0, 0],
#               [0, 0, 0, 1]]

# # left grasp quaternion w,x,y,z
#     # quat = [0.5,0.5,0.5,0.5]

# # right grasp
#     # matrix = [[0, 0, 1, 0],
#     #           [-1, 0, 0, 0],
#     #           [0, -1, 0, 0],
#     #           [0, 0, 0, 1]]

#     Rx90 = np.array([[1,0,0,0],
#                     [0,-1,0,0],
#                     [0,0,-1,0],
#                     [0,0,0,1]])

#     Rz90 = np.array([[-1,0,0,0],
#                     [0,-1,0,0],
#                     [0,0,1,0],
#                     [0,0,0,1]])
# # right grasp quaternion w,x,y,z
#     quat = [-0.5,0.5,-0.5,0.5]

#     # print(np.around(transform.quaternion_from_matrix(matrix), decimals=16))

#     # print(np.around(transform.euler_from_quaternion(quat), decimals=16))
#     # print(np.around(transform.quaternion_matrix(quat)))
#     # q = np.array(quat[:4], dtype=np.float64, copy=True)
#     # q *= 2.0 / 0.5 * 0.1

#     # print(np.dot(Rx90, matrix))
#     # quat = [0.5,-0.5,0.5,-0.5]
#     # print(transform.quaternion_from_matrix(np.dot(Rx90, matrix)))
#     # print(np.around(transform.quaternion_matrix(quat)))
#     # print(Rz90[0:3,0:3])

#     # print(transform.rotation_matrix(-2.09991,[0.57908,-0.577581,-0.575384]))
#     rTw = [ [1, 0, 0],
#             [0, 0, -1],
#             [0, 1, 0]] 
#     # rTw = [ [0, 1, 0],
#     #         [0, 0, -1],
#     #         [-1, 0, 0]] 

#     wTb = [ [0, -1, 0],
#             [0, 0, 1],
#             [-1, 0, 0]]
#     # # rTb = [ [0, 0, 1],
#     # #         [1, 0, 0],
#     # #         [0, 1, 0]]
#     # # print(np.dot(rTb,np.linalg.inv(wTb)))
#     # rTb = transform.identity_matrix()
#     # rTb[0:3,0:3] = np.dot(rTw,wTb)
#     # print(rTb)
#     # print(transform.quaternion_from_matrix(rTb))
#     print(Rx90[0,1])

    # print(transform.euler_from_matrix(awTc))


    # aubo eye in hand joints
    # joints = [-50.375118, 46.028324, 127.969673, -69.304242, 20.802938, 124.811107]
    # for i in range(len(joints)):
    #     deg_to_rad(joints[i])
    # [-0.8792116701806609, 0.8033458029747264, 2.233492136550489, -1.20958720849894, 0.3630797621882441, 2.178364760208942]

    # from qingzheng and yanshu we have
    abTc = [[-0.92870 ,-0.03257, 0.36941, -0.27700490], 
             [-0.33276, 0.51287, -0.79135, -0.43478580], 
             [-0.16368, -0.85785 ,-0.48714, 0.62455730],
             [0,0,0,1]]

    # because
    abPaw = [-0.127715,-0.299169,0.590675]
    abQaw = [0.717229,-0.266464,-0.481602,0.427363]
    # so transform quaternion to rotation matrix we got
    abTaw = [[ 0.39411297 , 0.02940633 ,-0.91859144, -0.127715       ],
             [-0.79386907 ,-0.49271567 ,-0.35637504, -0.299169      ],
             [-0.46308408 ,0.86969336 ,-0.17084086 , 0.590675        ],
             [ 0.     ,     0.      ,    0.     ,     1.        ]]
    # take the inverse matrix
    awTab = np.linalg.inv(abTaw)

    # we get the ralationship of camera wrt. aubo wrist3 by following
    awTc = np.dot(awTab,abTc)
    awTc = [[-0.02604724 ,-0.02273121 , 0.99940434  ,0.03313454],
            [-0.005705   ,-0.99972329 ,-0.02288888  ,0.09189767],
            [ 0.99964646 ,-0.00629971 , 0.02590394  ,0.17967838],
            [ 0.         , 0.         , 0.          ,1.        ]]

    # get euler from awTc, this is the resualt we want for eye in hand case
    awEc = [-0.2385638215371723, -1.5441285291951534, -2.9259724078202893]
    awPc = [0.03313454, 0.09189767, 0.17967838]



    # target joint for robot to aubo calibration
    joints2 = [-116.528601, -96.394829, 28.621979, 114.499488, 108.668105, -43.367876]
    for i in range(len(joints2)):
        deg_to_rad(joints2[i])
    # [-2.0338077601927567, -1.682407147946913, 0.49954777198667405, 1.9983930574477369, 1.8966162241529123, -0.7569122257966283]

     # because
    abP2c = [-0.648207,-1.22888,-0.151586]
    abQ2c = [-0.565469,0.712259,-0.263318,0.321863]
    # print(transform.quaternion_matrix(abQ2c))

    # so transform quaternion to rotation matrix we got
    abT2c = [[-0.15329824 ,-0.63601598 , 0.75629579  ,-0.648207        ],
             [-0.97502518 , 0.22181706 ,-0.01109413  ,-1.22888        ],
             [-0.16070327 ,-0.73910816 ,-0.65413576  ,-0.151586        ],
             [ 0.         , 0.         , 0.          ,1.        ]]

    # from qingzheng and yanshu we have
    rbT2c = [[0.97293  ,-0.22953 ,0.02697  ,0.16010320 ],
             [-0.16818 ,-0.62310 ,0.76384  ,-0.65318080],
             [-0.15852 ,-0.74770 ,-0.64484 ,1.06365900 ],
             [0.       , 0.      ,0.       ,1.         ]]

    # take the inverse matrix
    cT2ab = np.linalg.inv(abT2c)    
    rbT2ab = np.dot(rbT2c,cT2ab)
    rbT2ab = [[ 0.01723359 ,-0.99984414 ,-0.00434758 ,-1.05807336],
              [ 0.99977223 , 0.01729138 ,-0.0120897  , 0.01429496],
              [ 0.0121602  ,-0.00413769 , 0.99991875 , 1.21803029],
              [ 0.         , 0.         , 0.         , 1.        ]]
    
    # rotate aubo base wrt itself for 180 degrees in z-axis
    Rz180 = np.array([[-1,  0, 0, 0],
                      [ 0, -1, 0, 0],
                      [ 0,  0, 1, 0],
                      [ 0,  0, 0, 1]])
    rbT2ab_RZ180 =  np.dot(rbT2ab, Rz180)
    rbT2ab_RZ180 = [[-0.01723359,  0.99984414, -0.00434758, -1.05807336],
                    [-0.99977223, -0.01729138, -0.0120897 ,  0.01429496],
                    [-0.0121602 ,  0.00413769,  0.99991875,  1.21803029],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]

    # A
    jointsA = [-125.137699, -10.777975, 58.020387, -14.342332, -83.660863, -55.270430]
    print("A!!!!!!!!!!!!!!!!!")
    for i in range(len(jointsA)):
        deg_to_rad(jointsA[i])
    # -2.1840648659196154
    # -0.1881111504476358
    # 1.01264678643131510
    # -0.2503209158141434
    # -1.4601575144099008
    # -0.9646509824930495

    # B
    jointsB = [120.158109, 26.638522, -36.913614, 19.267281, -95.174199, -59.028134]
    print("B!!!!!!!!!!!!!!!!!")
    for i in range(len(jointsB)):
        deg_to_rad(jointsB[i])
    # 2.09715462502023100
    # 0.46492991676494490
    # -0.6442641031102742
    # 0.33627749135694557
    # -1.6611031354982948
    # -1.0302352896084106

    # C
    jointsC = [-123.927171, -12.829371, 57.370966, 3.657669, -73.543857, -59.323629]
    print("C!!!!!!!!!!!!!!!!!")
    for i in range(len(jointsC)):
        deg_to_rad(jointsC[i])
    # -2.1629371666320334
    # -0.2239147649098774
    # 1.00131225174972130
    # 0.06383836699812848
    # -1.2835824492658794
    # -1.0353926502815910

    # D
    jointsD = [121.011158, 23.139315, -45.748365, -1.187208, -103.855383, -60.457454]
    print("D!!!!!!!!!!!!!!!!!")
    for i in range(len(jointsD)):
        deg_to_rad(jointsD[i])
    # 2.11204313875107600
    # 0.40385723340611170
    # -0.7984595966541357
    # -0.0207206885060168
    # -1.8126183792697463
    # -1.0551816296730159