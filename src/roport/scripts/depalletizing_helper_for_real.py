#!/usr/bin/env python
from __future__ import print_function
import threading

import rospy
import tf2_ros

import numpy as np

from geometry_msgs.msg import Pose, Quaternion, Vector3, PoseArray

from std_srvs.srv import SetBool, Trigger
from robot_webots.srv import *

from roport.srv import *
from smarteye.srv import *

from rotools.utility.common import sd_pose, to_ros_pose, get_param, offset_ros_pose, set_param, all_close
from rotools.utility import transform

# maybe in need in the future
# class BoxPickingHelper(object):
#     def __init__(self):
#         super(BoxPickingHelper, self).__init__()


class NaiveDepalletizingPlanner(object):

    def __init__(self):
        super(NaiveDepalletizingPlanner, self).__init__()
        self.pick_from_left = True

    def picking_plan(self, obj_pose):
        # reach the limit in z-axis
        # MaxZ_buffer = 2.245
        # MaxZ = 2.3449
        MaxZ = 2.245
        # MinZ_buffer = 0.366
        # MinZ = 0.266
        MinZ = 0.366
        # if obj_pose[2, 3] > MaxZ_buffer:
        #     # when the gripper is on the limit of side pick, we need to rotate it 90 degree, to make the girpper upwards and increase Z limit
        #     Rz90 = np.array([[0, -1, 0],
        #                       [1, 0, 0],
        #                       [0, 0, 1]])
        #     obj_pose[0:3, 0:3] = np.dot(obj_pose[0:3, 0:3], Rz90)
        if obj_pose[2, 3] > MaxZ:
            rospy.logerr(
                'Target box is too high to reach and I can only reach the MaxZ at %f m' % MaxZ)
            obj_pose[2, 3] = MaxZ

        # if obj_pose[2, 3] < MinZ_buffer:
        #     # when the gripper is on the limit of side pick, we need to rotate it 90 degree, to make the girpper downwards and decrease Z limit
        #     RzMinus90 = np.array([[0, 1, 0],
        #                       [-1, 0, 0],
        #                       [0, 0, 1]])
        #     obj_pose[0:3, 0:3] = np.dot(obj_pose[0:3, 0:3], RzMinus90)
        if obj_pose[2, 3] < MinZ:
            rospy.logerr(
                'Target box is too low to reach and I can only reach the MinZ at %f m' % MinZ)
            obj_pose[2, 3] = MinZ

        # the box comes from whether the left or right side
        MaxY = 0.95
        if obj_pose[1, 3] >= 0:
            self.pick_from_left = True
            # reach the limit in y-axis
            if obj_pose[1, 3] > MaxY:
                rospy.logerr(
                    'Target box is too letf to reach and I can only reach the MaxY at %f m' % MaxY)
                obj_pose[1, 3] = MaxY
            tcp_pose = obj_pose
        else:
            # reach the limit in y-axis
            if obj_pose[1, 3] < -MaxY:
                rospy.logerr(
                    'Target box is too right to reach and I can only reach the MinY at -%f m' % MaxY)
                obj_pose[1, 3] = -MaxY
            # when on the right we need to rotate tcp wrt itself in z-axis for 180 degree
            tcp_pose = obj_pose
            Rz180 = np.array([[-1, 0, 0],
                              [0, -1, 0],
                              [0, 0, 1]])
            tcp_pose[0:3, 0:3] = np.dot(tcp_pose[0:3, 0:3], Rz180)
            self.pick_from_left = False

        # pick
        pick_tcp_pose = to_ros_pose(tcp_pose)
        # rospy.logwarn('pick_tcp_pose is ' + str(pick_tcp_pose) + '!!!!!')
        # pre-pick
        pre_pick_offset = get_param('pre_pick_offset', [-0.1, 0, 0])
        pre_pick_tcp_pose = offset_ros_pose(pick_tcp_pose, pre_pick_offset)
        # pull out planning
        post_pick_offset = get_param('post_pick_offset', [-0.43, 0, 0.3])

        # can not be raised up any more cause it will collide with the roof
        if obj_pose[2, 3] >= MaxZ:
            post_pick_offset[2] = 0
        elif obj_pose[2, 3] < MaxZ and obj_pose[2, 3] >= (MaxZ - post_pick_offset[2]):
            post_pick_offset[2] = MaxZ - obj_pose[2, 3]

        post_pick_edge_offset = [0, 0, 0]
        # 0.38 is the mimimum x coordinate of tcp after the box pulled out, that is to say the max box thickness in x direction is 0.83-0.38 = 0.45m
        post_pick_offset[0] = 0.40 - obj_pose[0, 3]
        post_pick_edge_offset[0] = post_pick_offset[0]

        BestY = 0.65
        # first pull out action "post_pick_tcp_pose"
        if abs(obj_pose[1, 3]) >= BestY:
            # raise and pull out
            post_pick_tcp_pose = offset_ros_pose(
                pick_tcp_pose, post_pick_offset)
        else:
            # only raise
            post_pick_offset[0] = 0
            post_pick_tcp_pose = offset_ros_pose(
                pick_tcp_pose, post_pick_offset)

        # second pull ou action "post_pick_edge_offset"
        # if box is far from origin in y direction, it has to come back a little in case of collision with side wall.
        if abs(obj_pose[1, 3]) >= BestY:
            # if y > BestY, we have pull it out before, and we pull out a little more for it to get close to center
            post_pick_edge_offset[0] = -0.02
            if self.pick_from_left:
                post_pick_edge_offset[1] -= (abs(obj_pose[1, 3])-BestY)
            else:
                post_pick_edge_offset[1] += (abs(obj_pose[1, 3])-BestY)

        # if box is close from origin in y direction, it has to come out a little because of workspace limitation.
        # if y < BestY, we still did not pull out the box, and we pull out here and add a small offset
        elif abs(obj_pose[1, 3]) < BestY:
            post_pick_edge_offset[0] -= 0.02
            if self.pick_from_left:
                post_pick_edge_offset[1] += (BestY-abs(obj_pose[1, 3]))
            else:
                post_pick_edge_offset[1] -= (BestY-abs(obj_pose[1, 3]))

        post_pick_tcp_pose_edge = offset_ros_pose(
            post_pick_tcp_pose, post_pick_edge_offset)
        return pick_tcp_pose, pre_pick_tcp_pose, post_pick_tcp_pose, post_pick_tcp_pose_edge

    def middle_plan(self, obj_pose):
        # if pick from left side, we'd better to place the box from right side.
        if self.pick_from_left:
            pre_middle_pose = get_param(
                'pre_middle_left', [0.0829, -1.7568, 2.5506, 0.7786, -1.5734, -0.0038])
            post_middle_pose = get_param(
                'post_middle_left', [0.7015, 0.4237, 1.7017, -0.0747, -0.0040, -0.0806])
        # vise versa
        else:
            pre_middle_pose = get_param(
                'pre_middle_right', [0.0777, 1.8904, -2.4741, -0.9875, 1.5718, -0.0037])
            post_middle_pose = get_param(
                'post_middle_right', [0.7015, -0.4631, -1.6642, -1.0472, 0.0081, -0.0615])
        # if obj_pose[2, 3] >= 1.3:
        #     pre_middle_pose[0] += 0.8
        return pre_middle_pose, post_middle_pose

    def placing_plan(self):
        # if pick from left side, we'd better to place the box from right side.
        if self.pick_from_left:
            place_pose = get_param(
                'place_left', [0.7005, 1.8997, 2.0341, 0.8173, 0.0009, -0.0041])
        # vise versa
        else:
            place_pose = get_param(
                'place_right', [0.7015, -1.8999, -2.0660, -0.7844, 0.0080, -0.0615])
        return place_pose


class DepalletizingHelper(object):

    def __init__(self, ):
        super(DepalletizingHelper, self).__init__()

        # Unchanged. transformation matrix of aubo (base_link) frame wrt. robot base_link frame (robot_link0). x = -1.05807336; x-= 0.04
        self.rbTab_A =  [[0,  1, 0, -1.05807336],
                       [-1, 0, 0 ,  0.01429496],
                       [0 ,  0,  1,  1.41803029],
                       [ 0.        ,  0.        ,  0.        ,  1.        ]]

        self.rbTab_B =  [[-0.01723359,  0.99984414, -0.00434758, -1.05807336],
                       [-0.99977223, -0.01729138, -0.0120897 ,  0.01429496],
                       [-0.0121602 ,  0.00413769,  0.99991875,  1.21803029],
                       [ 0.        ,  0.        ,  0.        ,  1.        ]]

        self.rbTab_C =  [[0,  1, 0, -0.94807336],
                       [-1, 0, 0 ,  0.01429496],
                       [0 ,  0,  1,  1.41803029],
                       [ 0.        ,  0.        ,  0.        ,  1.        ]]

        self.rbTab_D =  [[-0.01723359,  0.99984414, -0.00434758, -1.05807336],
                        [-0.99977223, -0.01729138, -0.0120897 ,  0.01429496],
                        [-0.0121602 ,  0.00413769,  0.99991875,  1.21803029],
                        [ 0.        ,  0.        ,  0.        ,  1.        ]]

        self.testX = [[1, 0   ,0   ,0],
                    [0, 0.99 ,0.08,0],
                    [0, -0.08,0.99,0],
                    [0, 0    ,0   ,1]]
        self.testY = [[0.99, 0   ,-0.08   ,0],
                    [0,    1 ,  0,     0],
                    [0.08, 0,   0.99,  0],
                    [0, 0    ,  0   , 1]]
        self.testZ = [[0.99, 0.08   ,0   ,0],
                    [-0.08, 0.99   ,0,    0],
                    [0,    0,   1,       0],
                    [0,    0    ,0   ,1]]
        self.test = np.dot(self.testX,self.testY)
        self.test = np.dot(self.testX,self.testZ)
        self.rbTab_A = np.dot(self.rbTab_A, self.test)
        self.rbTab_C = np.dot(self.rbTab_C, self.test)

        # Changing. initialzie the relation between aubo planning frame (pedestal_link) to on-hand camera frame, and waiting for updating by tree node later
        self.aTc = [0., 0., 0., 0., 0., 0., 1.]

        # Unchanged. relation between aubo (base_link) frame to aubo planning frame (pedestal_link)
        self.abTa =[0., 0., -0.5, 0., 0., 0.707108, 0.707105]

        # Changing. initialzie the relation between aubo (base_link) to on-hand camera frame, and waiting for updating by tree node later
        self.abTc = np.dot(sd_pose(self.abTa), sd_pose(self.aTc))

        # Changing. everytime when update the abTc, the rbTc needs to be updated as well
        self.rbTc = np.dot(self.rbTab_A, self.abTc)

        # Unchanged. relation between robot planning frame (robot_bottom) to (robot_link0) frame
        self.rTrb = [0., 0., 0.07 , 0., 0., 0., 1.]

        # Changing. the transformation matrix of camera frame wrt. robot planning frame (robot_bottom)
        self.rTc = np.dot(sd_pose(self.rTrb),self.rbTc)

        # planner for pick and place pose
        self.planner = NaiveDepalletizingPlanner()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # flag for decide whether the aubo has changed position just ago
        self.aubo_pose_ = 'A'

        # Assign goal points here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # Important Note!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # x has to be in 0.83 to 0.91m
        # |y| has to be smaller than 0.95m, if y >= 0, is on left; if y<0, is on right.
        # z has to be within 0.366m to 2.245m
        ## self.goalpoints = [[x1,y1,z1],[x2,y2,z2],...,...,[xn,yn,zn]]
        # self.goalpoints = [[0.83,0.95,0.7],[0.83,0.65,0.7],[0.83,0.35,0.7],[0.83,0.05,0.7],[0.91,0.95,0.7],[0.91,0.65,0.7],[0.91,0.35,0.7],[0.91,0.05,0.7]]
        # self.goalpoints = [[0.91,0.95,0.366],[0.91,0.95,2.245],[0.91,-0.95,0.366],[0.91,-0.95,2.245]]
        self.goalpoints = [[0.862076, 0.438042, 0.614171]]#, [
        #     0.9, 0.2, 0.7], [0.9, 0.2, 0.7], [0.9, 0.2, 0.7]]


        self.goalpointcnt = 0

        # enabled_clients
        self.enabled_clients = []

        # Client for ask aubo to change photoing pose
        self.move_aubo_client = self._create_client(
            '/aubo/execute_group_named_states', ExecuteGroupNamedStates, False)
        # Client for ask robot to change pre middle pose
        self.move_robot_client = self._create_client(
            '/robot/execute_group_named_states', ExecuteGroupNamedStates, False)

        # Clients for querying services provided by real devices
        self.get_pointcloud_client = self._create_client(
            '/hv1000/get_pointcloud', GetPointCloud, False)
        self.get_object_pose_client = self._create_client(
            '/box/get_object_pose', GetObjectPose, False)

        self.run_gripper_client = self._create_client(
            '/gripper/run', SetBool, False)


        self.get_eef_pose_client = self._create_client(
            'aubo/get_group_pose', GetGroupPose, False)

        # Service servers towards task scheduler
        self._execute_planning_srv = rospy.Service(
            'execute_planning', ExecutePlanning, self._execute_planning_handle
        )
        self._type_in_pose_srv = rospy.Service(
            'type_in_pose', TypeInPose, self._type_in_pose_handle
        )
        self._connect_waypoints_srv = rospy.Service(
            'connect_waypoints', ConnectWaypoints, self._connect_waypoints_handle
        )
        self._execute_suction_srv = rospy.Service(
            'execute_suction', ExecuteSuction, self._execute_suction_handle
        )
        self._sense_object_pose_srv = rospy.Service(
            'sense_object_pose', SenseObjectPose, self._sense_object_pose_handle
        )

    def _create_client(self, srv_id, srv_type, simulation=True, timeout=1):
        """Create service client for querying simulated or real devices

        :param srv_id: str ID of the service provided by device interface
        :param srv_type: rossrv
        :param simulation: bool If true, create for simulated device, otherwise false
        "param timeout: int Timeout in sec
        """
        if simulation:
            full_id = '/simulation' + srv_id
        else:
            full_id = srv_id
        try:
            rospy.wait_for_service(full_id, timeout)
            client = rospy.ServiceProxy(full_id, srv_type)
            self.enabled_clients.append(client)
            return client
        except rospy.ROSException:
            rospy.logwarn('Service ' + full_id + ' not available')
            return None

    def _execute_suction_handle(self, req):
        resp = ExecuteSuctionResponse()
        if self.run_gripper_client in self.enabled_clients:
            run_resp = self.run_gripper_client(req.enable)
            if req.enable:
                rospy.loginfo('Vaccum gripper grasping')
            else:
                rospy.loginfo('Vaccum gripper droping')
            if run_resp.success:
                resp.result_status = resp.SUCCEEDED
            else:
                resp.result_status = resp.FAILED
            rospy.sleep(0.5)
            return resp
        else:
            if req.enable:
                rospy.loginfo('Vaccum gripper grasping')
            else:
                rospy.loginfo('Vaccum gripper droping')
            resp.result_status = resp.SUCCEEDED
            rospy.sleep(1.0)
            return resp

    def _type_in_pose_handle(self, req):
        # used to transform typein information into blackboard
        resp = TypeInPoseResponse()
        resp.pose_on_blackboard = req.type_in_pose
        return resp

    def _connect_waypoints_handle(self, req):
        # used to connect multiple waypoints in one path
        resp = ConnectWaypointsResponse()
        connected_waypoints = PoseArray()
        connected_waypoints.header.frame_id = 'robot_link0'
        if not req.waypoint1.position.x == 0 and not req.waypoint1.position.y == 0:
            connected_waypoints.poses.append(req.waypoint1)
        if not req.waypoint2.position.x == 0 and not req.waypoint2.position.y == 0:
            connected_waypoints.poses.append(req.waypoint2)
        if not req.waypoint3.position.x == 0 and not req.waypoint3.position.y == 0:
            connected_waypoints.poses.append(req.waypoint3)
        if not req.waypoint4.position.x == 0 and not req.waypoint4.position.y == 0:
            connected_waypoints.poses.append(req.waypoint4)
        if not req.waypoint5.position.x == 0 and not req.waypoint5.position.y == 0:
            connected_waypoints.poses.append(req.waypoint5)
        resp.connected_waypoints = connected_waypoints
        return resp

    def _execute_planning_handle(self, req):
        """Plan pick and place pose_util for given box pose_util and its category.
        All poses are relevant to the robot base frame
        """
        rospy.logwarn('The ongoing target pose wrt robot_bottom is %f, %f, %f, %f, %f, %f, %f' % 
                    (req.pose.position.x, req.pose.position.y, req.pose.position.z, 
                    req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w))
        # decide whether to move forward the base before next pick
        MaxX = 0.91
        OptimalX = 0.87
        MinX = 0.83
        if req.pose.position.x <= MaxX:
            obj_pose = sd_pose(req.pose)
        else:
            # this is for future moving base module
            # if self.sim_move_base_client in self.enabled_clients:
            #     move_forward = req.pose.position.x - OptimalX
            #     self.sim_move_base_client(move_forward)
            #     req.pose.position.x -= move_forward
            #     obj_pose = sd_pose(req.pose)
            # else:
            # we don have moving base now
            resp = ExecutePlanningResponse()
            rospy.logerr(
                'Target box is too far to reach and I can only reach the MaxX at %f m and I can not move forward at present' % MaxX)
            resp.result_status = resp.FAILED
            return resp

        # plan pick
        pick_tcp_pose, pre_pick_tcp_pose, post_pick_tcp_pose, post_pick_tcp_pose_edge = self.planner.picking_plan(
            obj_pose)

        # plan middle
        pre_middle_pose, post_middle_pose = self.planner.middle_plan(obj_pose)

        # plan place
        place_pose = self.planner.placing_plan()
        resp = ExecutePlanningResponse()
        # Double array
        resp.pre_middle_pose = pre_middle_pose
        # Pose
        resp.pre_pick_pose = pre_pick_tcp_pose
        resp.pick_pose = pick_tcp_pose
        resp.post_pick_pose = post_pick_tcp_pose
        resp.post_pick_pose_edge = post_pick_tcp_pose_edge
        # Double array
        resp.post_middle_pose = post_middle_pose
        resp.place_pose = place_pose
        return resp

    def _sense_object_pose_handle(self, req):
        # 'Get pointcloud' and 'Get object pose' can be intergrated together. However, they are separeted in the code below.
        """Get the box pose in robot base frame.

        :param req: no need to give
        """
        resp = SenseObjectPoseResponse()
        obj_pose = Pose()
        # Important Note!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # do not motifiy the orientation.
        obj_pose.orientation.x = 0.5
        obj_pose.orientation.y = 0.5
        obj_pose.orientation.z = 0.5
        obj_pose.orientation.w = 0.5

        # demo with vision if the smarteye node is running
        if self.get_pointcloud_client in self.enabled_clients:
            pointcloud_req = GetPointCloudRequest()
            pointcloud_resp = self.get_pointcloud_client(pointcloud_req)
            if pointcloud_resp.result_status == pointcloud_resp.FAILED:
                rospy.logerr('Get point cloud failed')
                resp.result_status = resp.FAILED
                return resp
            else:
                rospy.logwarn('Get point cloud successed')
                # update hand eye matrix
                if not self.update_camera_frame():
                    resp.result_status = resp.FAILED
                    rospy.logerr('Failed to update camera frame')
                    return resp     
                # do pose detection 
                if self.get_object_pose_client in self.enabled_clients:
                    get_object_pose_req = GetObjectPoseRequest()
                    get_object_pose_resp = self.get_object_pose_client(get_object_pose_req)                
                    if get_object_pose_resp.result_status == get_object_pose_resp.FAILED:
                        rospy.logerr('Get pose failed')
                        resp.result_status = resp.FAILED
                        return resp
                    else:
                        if get_object_pose_resp.pose_amount == get_object_pose_resp.SINGLE:
                            rospy.logwarn('Get single pose successed')
                            # do hand eye transform
                            pose_detected_transformed_matrix = self.hand_eye_transform(get_object_pose_resp.pose)
                            pose_detected_transformed = to_ros_pose(pose_detected_transformed_matrix)
                            obj_pose.position.x = pose_detected_transformed.position.x
                            obj_pose.position.y = pose_detected_transformed.position.y
                            obj_pose.position.z = pose_detected_transformed.position.z
                        else:
                            rospy.logwarn('Get multi poses successed')
                            poses_1d_array = np.array(get_object_pose_resp.poses_list) 
                            poses_2d_array = np.reshape(poses_1d_array,(-1,7))  
                            # do hand eye transform and choose the optimal one from all to grasp
                            pose_detected_transformed_matrix_optimal = np.zeros((4,4))
                            for cnt in range(np.size(poses_2d_array,0)):
                                # transform pose from camera to robot coordinate one by one
                                pose_detected_transformed_matrix = self.hand_eye_transform(poses_2d_array[cnt])
                                # Three conditions to decide whether it is the optimal one:
                                # 1. whether the Z axis of the pose is parallel enough with the X axis of the robot. here arccos(0.86603) is 30 degree
                                if np.dot(np.array([1, 0, 0]),pose_detected_transformed_matrix[0:3,2]) > 0.86603:
                                    # 2. whether the x coordinate is in the workable space of the robot. here the workable is 0.83 to 0.91m
                                    if pose_detected_transformed_matrix[0,3] > 0.75 and pose_detected_transformed_matrix[0,3] < 0.95:
                                        # 3. whether the pose is in current picking district e.g. A, B, C or D?
                                        if self.is_pose_in_area(pose_detected_transformed_matrix[1,3],pose_detected_transformed_matrix[2,3]):
                                            # 4. whether the z coordinate is the biggest in the district, namely highest one will be choosed.
                                            if pose_detected_transformed_matrix[2,3] > pose_detected_transformed_matrix_optimal[2,3]:
                                                pose_detected_transformed_matrix_optimal = pose_detected_transformed_matrix
                            # if no optimal one, change pose to next district
                            if np.linalg.det(pose_detected_transformed_matrix_optimal) == 0:
                                rospy.logwarn('No optimal one from detected multi poses, need change Robot and Aubo pose to get new poses')
                                ### currently, if aubo reach the C district and detect nothing, the system will stop after a few retries on detection.
                                if self.aubo_pose_ == 'C':
                                    rospy.logwarn('C district has no boxes been detected and need to move forward to pick next box layer')
                                    ### in the futher, we will move the moving base forward to get close to next box layer
                                    resp.result_status = resp.FAILED
                                    return resp
                                move_aubo_req_state_name = self.next_area()
                                if self.aubo_pose_ == 'A' or self.aubo_pose_ == 'C':
                                    move_robot_req_state_name = 'left_pre'
                                elif self.aubo_pose_ == 'B' or self.aubo_pose_ == 'D':
                                    move_robot_req_state_name = 'right_pre'
                                # move robot
                                rospy.logwarn('Need to change Robot pre midlle pose.')
                                thread1 = threading.Thread(target=self.move_robot_proccess, args=(move_robot_req_state_name,))
                                thread1.start()
                                # move aubo
                                rospy.logwarn('Need to change Aubo photoing pose.')
                                thread2 = threading.Thread(target=self.move_aubo_proccess, args=(move_aubo_req_state_name,))
                                thread2.start()
                                # wait both finish moving
                                thread2.join()
                                thread1.join()
                                rospy.logwarn('Finish moving both robots and will detect box pose for new district again.')
                                resp.result_status = resp.FAILED
                                return resp
                            else:
                                # transfer the optimal one from matrix to ros pose
                                pose_detected_transformed = to_ros_pose(pose_detected_transformed_matrix_optimal)
                                rospy.logwarn('Get the optimal one from multi poses')
                                obj_pose.position.x = pose_detected_transformed.position.x
                                obj_pose.position.y = pose_detected_transformed.position.y
                                obj_pose.position.z = pose_detected_transformed.position.z                
                        # output the result
                        resp.result_status = resp.SUCCEEDED
                        resp.pose = obj_pose
                        return resp
                        
                else:
                    rospy.logerr('Get object pose client is not enabled, please make sure the box node is running')
                    resp.result_status = resp.FAILED
                    return resp

        # demo with pre-defined position if the smarteye node is not running
        else:
            rospy.logwarn('Get point cloud client is not enabled, if you want to use vision, please make sure smart eye node is running')
            # Important Note!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            # x has to be in 0.83 to 0.91m
            # |y| has to be smaller than 0.95m, if y >= 0, is on left; if y<0, is on right.
            # z has to be within 0.366m to 2.245m
            if self.goalpointcnt < len(self.goalpoints):
                obj_pose.position.x = self.goalpoints[self.goalpointcnt][0]
                obj_pose.position.y = self.goalpoints[self.goalpointcnt][1]
                obj_pose.position.z = self.goalpoints[self.goalpointcnt][2]
                resp.result_status = resp.SUCCEEDED
                resp.pose = obj_pose
                self.goalpointcnt += 1
                rospy.logwarn(
                    'We are going to execute number %d goal point!!!!!' % self.goalpointcnt)
            else:
                resp.result_status = resp.FAILED
                rospy.logerr(
                    'All assigned goal points has been finished, I have to stop!!!!!!!!!')
                return resp

            # decide whether to change aubo's pose to A, B, C, D districts
            # move_aubo_req = ExecuteGroupNamedStatesRequest()
            # move_aubo_req.group_name = 'aubo'

        y_value = obj_pose.position.y
        z_value = obj_pose.position.z
        # if self.move_aubo_client in self.enabled_clients:
        move_aubo_req = ExecuteGroupNamedStatesRequest()
        if y_value >= 0 and z_value >= 1.35:
            move_aubo_req.state_name = 'A'
        elif y_value < 0 and z_value >= 1.35:
            move_aubo_req.state_name = 'B'
        elif y_value >= 0 and z_value < 1.35:
            move_aubo_req.state_name = 'C'
        elif y_value < 0 and z_value < 1.35:
            move_aubo_req.state_name = 'D'

        if self.aubo_pose_ == move_aubo_req.state_name:
            rospy.logwarn(
                'No need to change Aubo pose, which is in Pose %s !!!!!' % self.aubo_pose_)
        else:
            self.aubo_pose_ = move_aubo_req.state_name
            if self.move_robot_client in self.enabled_clients:
                move_robot_req = ExecuteGroupNamedStatesRequest()
                move_robot_req.group_name = 'arm'
                if self.aubo_pose_ == 'A' or self.aubo_pose_ == 'C':
                    move_robot_req.state_name = 'left_pre'
                elif self.aubo_pose_ == 'B' or self.aubo_pose_ == 'D':
                    move_robot_req.state_name = 'right_pre'
            # move robot
            rospy.logwarn('Need to change Robot pre midlle pose.')
            thread1 = threading.Thread(
                target=self.move_robot_proccess, args=(move_robot_req.state_name,))
            thread1.start()
            # move aubo
            rospy.logwarn('Need to change Aubo photoing pose.')
            # thread2 = threading.Thread(target=self.move_aubo_proccess, args=(move_aubo_req.state_name,))
            # thread2.start()
            # wait both finish moving
            # thread2.join()
            thread1.join()
            # capture point cloud one more time after changing pose
            # rospy.logwarn('Recapturing photo after changing pose.')
            # obj_re_pose = Pose()
            # get_pose_resp = self.sim_get_pose_client(0)
            # obj_re_pose = get_pose_resp.pose
            # resp.result_status = resp.SUCCEEDED
            # resp.pose = obj_re_pose
        return resp

    def move_robot_proccess(self, state_name):
        move_robot_req1 = ExecuteGroupNamedStatesRequest()
        move_robot_req1.group_name = 'arm'
        move_robot_req1.state_name = state_name
        move_robot_resp1 = self.move_robot_client(move_robot_req1)
        rospy.logwarn('Robot has been successfully moved to Pose %s !!!!!. Then We Move Aubo!!!' %
                      move_robot_req1.state_name)

    def move_aubo_proccess(self, state_name):
        move_aubo_req1 = ExecuteGroupNamedStatesRequest()
        move_aubo_req1.group_name = 'aubo'
        move_aubo_req1.state_name = state_name
        move_aubo_resp1 = self.move_aubo_client(move_aubo_req1)
        rospy.logwarn('Aubo has been successfully moved to Pose %s !!!!!. Recapture pointcloud once again!!!' % move_aubo_req1.state_name)

    def hand_eye_transform(self, pose_detected):
        # transformation matrix of target box wrt. camera frame
        cTb = sd_pose(pose_detected)
        # transformation matrix of target box wrt. robot frame
        rTb = np.dot(self.rTc, cTb)
        return rTb

    def update_camera_frame(self):
        getPose_req = GetGroupPoseRequest()
        getPose_req.group_name = 'aubo'
        getPose_req.eef_name = 'hv1000_Link'
        getPose_resp = self.get_eef_pose_client(getPose_req)
        if getPose_resp.result_status == getPose_resp.FAILED:
            rospy.logerr('Get aubo camera pose failed')
            return False
        else:
            rospy.logwarn('Get aubo camera pose successed')
            # Update the relation between aubo planning frame (pedestal_link) to on-hand camera frame
            self.aTc = getPose_resp.pose

            # Update the relation between aubo (base_link) to on-hand camera frame
            self.abTc = np.dot(sd_pose(self.abTa), sd_pose(self.aTc))

            # update rbTc needs to be updated as well
            if self.aubo_pose_ == 'A':
                    rbTab = self.rbTab_A
            elif self.aubo_pose_ == 'B':
                    rbTab = self.rbTab_B
            elif self.aubo_pose_ == 'C':
                    rbTab = self.rbTab_C
            elif self.aubo_pose_ == 'D':
                    rbTab = self.rbTab_D
            self.rbTc = np.dot(rbTab, self.abTc)

            # Update the transformation matrix of camera frame wrt. robot planning frame (robot_bottom)
            self.rTc = np.dot(sd_pose(self.rTrb),self.rbTc)
            rospy.logwarn('Update relation of robot planning frame to camera frame successed')
            return True

    def is_pose_in_area(self, y_value, z_value):
        if self.aubo_pose_ == 'A':
            if y_value >= 0 and y_value <= 1.2 and z_value >= 1.35 and z_value <= 2.7:
                return True
        elif self.aubo_pose_ == 'B':
            if y_value < 0 and y_value >= -1.2 and z_value >= 1.35 and z_value <= 2.7:
                return True
        elif self.aubo_pose_ == 'C':
            if y_value >= 0 and y_value <= 1.2 and z_value < 1.35 and z_value >= 0:
                return True
        elif self.aubo_pose_ == 'D':
            if y_value < 0 and y_value >= -1.2 and z_value < 1.35 and z_value >= 0:
                return True
        return False

    def next_area(self):
        if self.aubo_pose_ == 'A':
            self.aubo_pose_ = 'C'
        elif self.aubo_pose_ == 'B':
            self.aubo_pose_ = 'D'
        elif self.aubo_pose_ == 'C':
            self.aubo_pose_ = 'B'
        elif self.aubo_pose_ == 'D':
            self.aubo_pose_ = 'A'
        return self.aubo_pose_


if __name__ == "__main__":
    try:
        rospy.init_node('roport_depalletizing_helper')
        helper = DepalletizingHelper()
        rospy.loginfo("Roport: Depalletizing helper ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
