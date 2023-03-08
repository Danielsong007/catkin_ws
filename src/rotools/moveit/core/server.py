#!/usr/bin/env python
from __future__ import print_function

import rospy

from roport.srv import *

# make sure setting > export PYTHONPATH="${PYTHONPATH}:~/rotools" in bashrc
import rotools.moveit.core.interface as interface


class MoveItServer(object):
    """An example RoPort server that uses the RoTools MoveIt interface to provide some
    handy services for controlling robot.
    """

    def __init__(self, kwargs):

        super(MoveItServer, self).__init__()

        self.interface = interface.MoveGroupInterface(**kwargs)

        self._srv_get_pose = rospy.Service('get_group_pose', GetGroupPose, self.get_pose_handle)
        self._srv_get_js = rospy.Service('get_group_joint_states', GetGroupJointStates, self.get_js_handle)

        self._srv_group_speed = rospy.Service('execute_group_speed', ExecuteGroupSpeed, self.group_speed_handle)
        self._srv_group_shift = rospy.Service('execute_group_shift', ExecuteGroupShift, self.group_shift_handle)
        self._srv_group_pose = rospy.Service('execute_group_pose', ExecuteGroupPose, self.group_pose_handle)
        self._srv_group_many_poses = rospy.Service('execute_group_many_poses', ExecuteGroupManyPoses, self.group_many_poses_handle)
        self._srv_group_js = rospy.Service('execute_group_joint_states', ExecuteGroupJointStates, self.group_js_handle)
        self._srv_group_home = rospy.Service('execute_group_named_states', ExecuteGroupNamedStates, self.group_named_states_handle)

        self._srv_add_box = rospy.Service('execute_add_box', ExecuteAddBox, self.add_box_handle)
        self._srv_add_plane = rospy.Service('execute_add_plane', ExecuteAddPlane, self.add_plane_handle)
        self._srv_attach_box = rospy.Service('execute_attach_box', ExecuteAttachBox, self.attach_box_handle)
        self._srv_detach_obj = rospy.Service('execute_detach_object', ExecuteDetachObject, self.detach_object_handle)
        self._srv_remove_obj = rospy.Service('execute_remove_object', ExecuteRemoveObject, self.remove_object_handle)

    def group_speed_handle(self, req):
        ok = self.interface.group_set_max_velocity_scaling_factor(req.group_name, req.value)
        resp = ExecuteGroupSpeedResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def get_pose_handle(self, req):
        resp = GetGroupPoseResponse()
        try:
            resp.pose = self.interface.get_current_pose_of_group(req.group_name, req.eef_name)
            resp.ee_link, resp.ref_link = self.interface.get_frame_of_group(req.group_name)
            if req.eef_name != '':
                resp.ee_link = req.eef_name
            resp.result_status = resp.SUCCEEDED
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def get_js_handle(self, req):
        resp = GetGroupJointStatesResponse()
        try:
            joint_names = self.interface.get_active_joint_names_of_group(req.group_name)
            joint_states = self.interface.get_joint_states_of_group(req.group_name)
            resp.joint_names = joint_names
            resp.joint_states = joint_states
            resp.result_status = resp.SUCCEEDED
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def group_named_states_handle(self, req):
        ok = self.interface.group_go_to_named_states(req.group_name, req.state_name)
        resp = ExecuteGroupNamedStatesResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_shift_handle(self, req):
        ok = self.interface.group_shift(req.group_name, req.axis, req.goal)
        resp = ExecuteGroupShiftResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_pose_handle(self, req):
        if not req.tolerance:
            req.tolerance = 0.01

        if req.goal_type == req.GLOBAL_BASE:
            ok = self.interface.group_go_to_global_base_goal(req.group_name, req.goal, req.tolerance, req.constraint)
        elif req.goal_type == req.LOCAL_BASE:
            ok = self.interface.group_go_to_local_base_goal(req.group_name, req.goal, req.tolerance, req.constraint)
        elif req.goal_type == req.EEF:
            ok = self.interface.group_go_to_eef_goal(req.group_name, req.goal, req.tolerance, req.constraint)
        else:
            rospy.logerr('Unknown goal type for group pose: {}'.format(req.goal_type))
            ok = False
        resp = ExecuteGroupPoseResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_many_poses_handle(self, req):
        # we use tolerance to be the switch for controlling the constraint of joint 5 and 6. 
        if not req.tolerance:
            req.tolerance = 0

        if not req.eef_step:
            req.eef_step = 0.1

        if req.goal_type == req.GLOBAL_BASE:
            plan = self.interface.build_absolute_path_for_group(req.group_name, req.goals, None, True, req.eef_step, req.tolerance)
            ok = self.interface.execute_plan_for_group(req.group_name, plan)
            waypoints_number = len(req.goals.poses)
            # ok = self.interface._wait_pose_goal_execution(req.group_name, req.goals.poses[waypoints_number-1], req.tolerance)
        else:
            rospy.logerr('Unknown goal type for group pose: {}'.format(req.goal_type))
            ok = False
        resp = ExecuteGroupManyPosesResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_js_handle(self, req):
        # we use tolerance to be the switch for controlling the constraint of joint 5 and 6.
        if not req.tolerance:
            req.tolerance = 0
        ok = self.interface.group_go_to_joint_states(req.group_name, req.goal, req.tolerance)
        resp = ExecuteGroupJointStatesResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def add_box_handle(self, req):
        ok = self.interface.add_box(req.group_name, req.box_name, req.box_pose,
                                    req.box_size, req.is_absolute, req.auto_subfix)
        resp = ExecuteAddBoxResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def attach_box_handle(self, req):
        ok = self.interface.attach_box(req.group_name, req.eef_group_name, req.box_name, req.box_pose, req.box_size)
        resp = ExecuteAttachBoxResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def add_plane_handle(self, req):
        ok = self.interface.add_plane(req.group_name, req.plane_name, req.plane_pose, req.plane_normal)
        resp = ExecuteAddPlaneResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def detach_object_handle(self, req):
        ok = self.interface.detach_object(req.group_name, req.obj_name)
        resp = ExecuteDetachObjectResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def remove_object_handle(self, req):
        ok = self.interface.remove_object(req.obj_name, req.is_exact)
        resp = ExecuteRemoveObjectResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp
