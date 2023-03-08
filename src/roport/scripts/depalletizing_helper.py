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
        # self.picking_helper = BoxPickingHelper()
        rospy.loginfo('Created helpers for picking box')
        self.pick_from_left = True

    def picking_plan(self, obj_pose):
        # reach the limit in z-axis
        MaxZ = 2.424
        MinZ = 0.49
        if obj_pose[2, 3] > MaxZ:
            obj_pose[2, 3] = MaxZ
        if obj_pose[2, 3] < MinZ:
            obj_pose[2, 3] = MinZ

        # the box comes from whether the left or right side
        MaxY = 0.95
        if obj_pose[1, 3] >= 0:
            self.pick_from_left = True
            # reach the limit in y-axis
            if obj_pose[1, 3] > MaxY:
                obj_pose[1, 3] = MaxY
            tcp_pose = obj_pose
        else:
            # reach the limit in y-axis
            if obj_pose[1, 3] < -MaxY:
                obj_pose[1, 3] = -MaxY
            # when on the right we need to rotate tcp wrt itself in z-axis for 180 degree
            tcp_pose = obj_pose
            Rz180 = np.array([[-1,0,0],
                              [0,-1,0],
                              [0,0,1]])
            tcp_pose[0:3,0:3] = np.dot(tcp_pose[0:3,0:3],Rz180)
            self.pick_from_left = False
            print(tcp_pose)

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
        elif obj_pose[2, 3] <MaxZ and obj_pose[2, 3]>= (MaxZ - post_pick_offset[2]):
            post_pick_offset[2] = MaxZ - obj_pose[2, 3]

        post_pick_edge_offset = [0, 0, 0]
        post_pick_edge_offset[0] = post_pick_offset[0]

        BestY = 0.65
        # first pull out action "post_pick_tcp_pose"
        if abs(obj_pose[1, 3]) >= BestY:
            # raise and pull out
            post_pick_tcp_pose = offset_ros_pose(pick_tcp_pose, post_pick_offset)
        else:
            # only raise
            post_pick_offset[0] = 0
            post_pick_tcp_pose = offset_ros_pose(pick_tcp_pose, post_pick_offset)

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
            post_pick_edge_offset[0] -= 0.03
            if self.pick_from_left:
                post_pick_edge_offset[1] += (BestY-abs(obj_pose[1, 3]))
            else:
                post_pick_edge_offset[1] -= (BestY-abs(obj_pose[1, 3]))

        post_pick_tcp_pose_edge = offset_ros_pose(post_pick_tcp_pose, post_pick_edge_offset)
        return pick_tcp_pose, pre_pick_tcp_pose, post_pick_tcp_pose, post_pick_tcp_pose_edge

    def middle_plan(self, obj_pose):
        # if pick from left side, we'd better to place the box from right side.
        if self.pick_from_left:
            pre_middle_pose = get_param('pre_middle_left', [0.0829, -1.7568, 2.5506, 0.7786, -1.5734, -0.0038])
            post_middle_pose = get_param('post_middle_left', [0.7015,0.4237,1.7017,-0.0747,-0.0040,-0.0806])
        # vise versa
        else:
            pre_middle_pose = get_param('pre_middle_right', [0.0777, 1.8904, -2.4741, -0.9875, 1.5718, -0.0037])
            post_middle_pose = get_param('post_middle_right', [0.7015,-0.4631,-1.6642, -1.0472, 0.0081, -0.0615])
        # if obj_pose[2, 3] >= 1.3:
        #     pre_middle_pose[0] += 0.8
        return pre_middle_pose, post_middle_pose
                                    

    def placing_plan(self):
        # if pick from left side, we'd better to place the box from right side.
        if self.pick_from_left:
            place_pose = get_param('place_left', [0.7005, 1.8997, 2.0341, 0.8173, 0.0009, -0.0041])
        # vise versa
        else:
            place_pose = get_param('place_right', [0.7015, -1.8999, -2.0660, -0.7844, 0.0080, -0.0615])
        return place_pose


class DepalletizingHelper(object):

    def __init__(self, ):
        super(DepalletizingHelper, self).__init__()
        
        # planner for pick and place pose
        self.planner = NaiveDepalletizingPlanner()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # flag for decide whether the aubo has changed position just ago
        self.aubo_pose = 'A'

        # switch for Parallel Running
        self.store_DI_state = False
        self.fetch_DI_state = False
        
        # switch for Parallel Running
        self.enabled_clients = []

        # Clients for querying services provided by simulated devices
        self.sim_delete_box_client = self._create_client(
            '/supervisor/delete_box', Trigger)
        self.sim_get_pose_client = self._create_client(
            '/supervisor/get_pose', NodeGetPose)
        # self.sim_get_position_client = self._create_client(
        #     '/supervisor/get_position', NodeGetPosition)
        # self.sim_get_orientation_client = self._create_client(
        #     '/supervisor/get_orientation', NodeGetOrientation)
        self.sim_set_position_client = self._create_client(
            '/supervisor/set_position', FieldSetVec3f)
        self.sim_set_rotation_client = self._create_client(
            '/supervisor/set_orientation', FieldSetRotation)
        self.sim_run_gripper_client = self._create_client(
            '/gripper/run', SetBool)
        self.sim_move_base_client = self._create_client(
            '/supervisor/move_base', SetFloat)
        # Client for ask aubo to change photoing pose           
        self.move_aubo_client = self._create_client(
            '/aubo/execute_group_named_states', ExecuteGroupNamedStates, False) 
        # Client for ask robot to change pre middle pose 
        self.move_robot_client = self._create_client(
            '/robot/execute_group_named_states', ExecuteGroupNamedStates, False) 
        # Clients for querying services provided by real devices
        self.get_pointcloud_client = self._create_client(
            '/hv1000/get_pointcloud', GetPointCloud, False)
        # self.get_object_info_client = self._create_client(
        #     '/vision/get_object_info', GetObjectPose, False)
        self.run_gripper_client = self._create_client(
            '/gripper/run', SetBool, False)
        self.get_tcp_pose_client = self._create_client(
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
            'sense_object_pose', SenseObjectPose, self.sense_object_pose_handle
        )
        self._store_detected_info_srv = rospy.Service(
            'store_detected_info', StoreDetectedInfo, self._store_detected_info_handle
        )
        self._fetch_detected_info_srv = rospy.Service(
            'fetch_detected_info', FetchDetectedInfo, self._fetch_detected_info_handle
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
            if run_resp.success:
                resp.result_status = resp.SUCCEEDED
            else:
                resp.result_status = resp.FAILED
                return resp
        if self.sim_run_gripper_client in self.enabled_clients:
            run_resp = self.sim_run_gripper_client(req.enable)
            # delete box after drop
            if not req.enable:
                rospy.sleep(0.5)
                delete_box_resp = self.sim_delete_box_client()
            if run_resp.success:
                resp.result_status = resp.SUCCEEDED
            else:
                resp.result_status = resp.FAILED
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
        # decide whether to move forward the base before next pick
        if req.pose.position.x <= 0.95:
            obj_pose = sd_pose(req.pose)
        else:
            if self.sim_move_base_client in self.enabled_clients:
                move_forward = req.pose.position.x - 0.90
                self.sim_move_base_client(move_forward)
            req.pose.position.x -= move_forward
            obj_pose = sd_pose(req.pose)

        # plan pick
        pick_tcp_pose, pre_pick_tcp_pose, post_pick_tcp_pose, post_pick_tcp_pose_edge = self.planner.picking_plan(obj_pose)

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

    def sense_object_pose_handle(self, req):
        # 'Get pointcloud' and 'Get object pose' can be intergrated together. However, they are separeted in the code below.
        """Get the box pose in robot base frame.

        :param req: no need to give
        """        
        resp = SenseObjectPoseResponse()
        if self.get_pointcloud_client in self.enabled_clients:
            pointcloud_req = GetPointCloudRequest()
            pointcloud_resp = self.get_pointcloud_client(pointcloud_req)
            if pointcloud_resp.result_status == pointcloud_resp.FAILED:
                rospy.logerr('Get point cloud failed')
                resp.result_status = resp.FAILED
                return resp

            if self.get_object_info_client in self.enabled_clients:
                obj_info_req = GetObjectPoseRequest()
                obj_info_req.points = pointcloud_resp.points               
                # obj pose in obj info is in robot base frame
                obj_info_resp = self.get_object_info_client(obj_info_req)
                resp.pose = obj_info_resp.pose
                resp.result_status = resp.SUCCEEDED
                return resp
            else:
                rospy.logwarn('Get object info client is not enabled')
        else:
            rospy.logdebug('Get point cloud client is not enabled')

        if self.sim_get_pose_client in self.enabled_clients:
        # self.sim_get_position_client in self.enabled_clients and \
        #         self.sim_get_orientation_client in self.enabled_clients
            obj_pose = Pose()
            # get_position_resp = self.sim_get_position_client(0)
            # obj_pose.position = get_position_resp.position
            # get_orientation_resp = self.sim_get_orientation_client(0)
            # obj_pose.orientation = get_orientation_resp.orientation
            get_pose_resp = self.sim_get_pose_client(0)
            obj_pose = get_pose_resp.pose

            # for sim, we omit translation between base frame to camera frame (hand-eye calibration)
            resp.result_status = resp.SUCCEEDED
            resp.pose = obj_pose

            # decide whether to change aubo's pose to A, B, C, D districts
            move_aubo_req = ExecuteGroupNamedStatesRequest()
            move_aubo_req.group_name = 'aubo'
            y_value = obj_pose.position.y
            z_value = obj_pose.position.z
            if self.move_aubo_client in self.enabled_clients:
                if y_value >= 0 and z_value >= 1.35:
                    move_aubo_req.state_name = 'A'
                elif y_value < 0 and z_value >= 1.35:
                    move_aubo_req.state_name = 'B'
                elif y_value >= 0 and z_value < 1.35:
                    move_aubo_req.state_name = 'C'
                elif y_value < 0 and z_value < 1.35:
                    move_aubo_req.state_name = 'D' 
            if self.aubo_pose == move_aubo_req.state_name:
                rospy.logwarn('No need to change Aubo pose, which is in Pose %s !!!!!' % self.aubo_pose)
            else:
                self.aubo_pose = move_aubo_req.state_name
                if self.move_robot_client in self.enabled_clients:
                    move_robot_req = ExecuteGroupNamedStatesRequest()
                    move_robot_req.group_name = 'arm'
                    if self.aubo_pose == 'A' or self.aubo_pose == 'C':
                        move_robot_req.state_name = 'left_pre'
                    elif self.aubo_pose == 'B' or self.aubo_pose == 'D':
                        move_robot_req.state_name = 'right_pre'
                # move robot
                rospy.logwarn('Need to change Robot pre midlle pose.')
                thread1 = threading.Thread(target=self.move_robot_proccess, args=(move_robot_req.state_name,))
                thread1.start()
                # move aubo
                rospy.logwarn('Need to change Aubo photoing pose.')
                thread2 = threading.Thread(target=self.move_aubo_proccess, args=(move_aubo_req.state_name,))
                thread2.start()
                # wait both finish moving
                thread2.join()
                thread1.join()
                # capture point cloud one more time after changing pose
                rospy.logwarn('Recapturing photo after changing pose.')                
                obj_re_pose = Pose()
                get_pose_resp = self.sim_get_pose_client(0)
                obj_re_pose = get_pose_resp.pose
                resp.result_status = resp.SUCCEEDED
                resp.pose = obj_re_pose
        return resp

    def move_robot_proccess(self, state_name):
        move_robot_req1 = ExecuteGroupNamedStatesRequest()
        move_robot_req1.group_name = 'arm'
        move_robot_req1.state_name = state_name
        move_robot_resp1 = self.move_robot_client(move_robot_req1)       
        rospy.logwarn('Robot has been successfully moved to Pose %s !!!!!. Then We Move Aubo!!!' % move_robot_req1.state_name)

    def move_aubo_proccess(self, state_name):
        move_aubo_req1 = ExecuteGroupNamedStatesRequest()
        move_aubo_req1.group_name = 'aubo'
        move_aubo_req1.state_name = state_name
        move_aubo_resp1 = self.move_aubo_client(move_aubo_req1)       
        rospy.logwarn('Aubo has been successfully moved to Pose %s !!!!!. Recapture pointcloud once again!!!' % move_aubo_req1.state_name)

    def _store_detected_info_handle(self, req):
        # store the newest dection info into ros server
        position = [req.pose.position.x,
                    req.pose.position.y, req.pose.position.z]
        set_param('detected_obj_pose_position', position)
        orientation = [req.pose.orientation.x, req.pose.orientation.y,
                       req.pose.orientation.z, req.pose.orientation.w]
        set_param('detected_obj_pose_orientation', orientation)
        rospy.sleep(0.1)
        self.store_DI_state = True

        # wait for robot to fetch this newest detection info
        while not rospy.is_shutdown() and not self.fetch_DI_state:
            rospy.sleep(0.1)
        self.fetch_DI_state = False

        # wait for the box, which is detected just ago, to be picked away and then start a new detection
        resp = StoreDetectedInfoResponse()
        if self.get_tcp_pose_client in self.enabled_clients:
            can_trigger = False
            while not rospy.is_shutdown() and not can_trigger:
                can_trigger_resp = self.get_tcp_pose_client()
                rospy.sleep(0.1)
                if self.planner.pick_from_left:
                    if all_close(self.planner.right_middle_pose.position, can_trigger_resp.pose.position, 0.05):
                        can_trigger = True
                else:
                    if all_close(self.planner.left_middle_pose.position, can_trigger_resp.pose.position, 0.05):
                        can_trigger = True
            resp.result_status = resp.SUCCEEDED
            return resp

    def _fetch_detected_info_handle(self, req):
        resp = FetchDetectedInfoResponse()
        # wait for the newest detected info to be sent in ros server.
        while not rospy.is_shutdown() and not self.store_DI_state:
            rospy.sleep(0.1)

        # robot gets the newest detection info for motion planning
        position = get_param('detected_obj_pose_position')
        resp.pose.position.x = position[0]
        resp.pose.position.y = position[1]
        resp.pose.position.z = position[2]
        orientation = get_param('detected_obj_pose_orientation')
        resp.pose.orientation.x = orientation[0]
        resp.pose.orientation.y = orientation[1]
        resp.pose.orientation.z = orientation[2]
        resp.pose.orientation.w = orientation[3]
        print('pose fetched is {}', format(resp.pose))
        self.store_DI_state = False
        self.fetch_DI_state = True
        resp.result_status = resp.SUCCEEDED
        return resp


if __name__ == "__main__":
    try:
        rospy.init_node('roport_depalletizing_helper')
        helper = DepalletizingHelper()
        rospy.loginfo("Roport: Depalletizing helper ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
