from __future__ import print_function

import sys
import math
import numpy as np

try:
    import rospy
    import moveit_commander

    import geometry_msgs.msg as GeometryMsg
    import moveit_msgs.msg as MoveItMsg
    import control_msgs.msg as ControlMsg
    import trajectory_msgs.msg as TrajectoryMsg
    import std_msgs.msg as StdMsg
    import sensor_msgs.msg as SensorMsg
except ImportError:
    pass

from rotools.utility import common, transform


class MoveGroupInterface(object):

    def __init__(
            self,
            robot_description,
            ns,
            group_names,
            ref_frames=None,
            ee_links=None,
    ):
        super(MoveGroupInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self.commander = moveit_commander.RobotCommander(robot_description, '')

        if not isinstance(group_names, list) and not isinstance(group_names, tuple):
            raise TypeError('group_names should be list or tuple, but got {}'.format(type(group_names)))

        self.group_names = group_names
        self.group_num = len(self.group_names)
        assert self.group_num >= 1

        # We can get a list of all the groups in the robot:
        all_group_names = self.commander.get_group_names()
        for name in self.group_names:
            assert name in all_group_names, 'Group name {} does not exist'.format(name)

        self.move_groups = []
        for name in self.group_names:
            self.move_groups.append(moveit_commander.MoveGroupCommander(name))

        # for group in self.move_groups:
        #     group.allow_looking(True)
        #     group.allow_replanning(True)

        if not ref_frames:
            self.ref_frames = []
            for group in self.move_groups:
                self.ref_frames.append(group.get_planning_frame())
        else:
            assert len(ref_frames) == self.group_num
            self.ref_frames = ref_frames
            for i, group in enumerate(self.move_groups):
                group.set_pose_reference_frame(ref_frames[i])

        if not ee_links:
            self.ee_links = []
            for group in self.move_groups:
                self.ee_links.append(group.get_end_effector_link())
        else:
            assert len(ee_links) == self.group_num
            self.ee_links = ee_links
            for i, group in enumerate(self.move_groups):
                group.set_end_effector_link(self.ee_links[i])

        self.scene = moveit_commander.PlanningSceneInterface(ns=ns)
        self._obj_suffix = 0

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print(self.commander.get_current_state())

    def get_all_group_names(self):
        return self.commander.get_group_names()

    def get_active_group_names(self):
        return self.group_names

    def _get_group_id(self, group_name):
        for i, name in enumerate(self.group_names):
            if name == group_name:
                return i
        return None

    def _get_group_by_name(self, group_name):
        for i, name in enumerate(self.group_names):
            if name == group_name:
                return self.move_groups[i]
        raise IndexError
    
    def _wait_js_goal_execution(self, group_name, js_goal, tol):
        js_temp = self.get_joint_states_of_group(group_name)
        cnt = 30
        while not rospy.is_shutdown() and cnt:
            rospy.sleep(0.1)
            js_curr = self.get_joint_states_of_group(group_name)
            if common.all_close(js_goal, js_curr, tol):
                return True
            if common.all_close(js_curr, js_temp, 0.001):
                cnt -= 1
            else:
                js_temp = js_curr
        return False
    
    def _wait_pose_goal_execution(self, group_name, pose_goal, tol):
        group = self._get_group_by_name(group_name)
        pose_temp = common.regularize_pose(group.get_current_pose().pose)
        cnt = 30
        while not rospy.is_shutdown() and cnt:
            rospy.sleep(0.1)
            pose_curr = common.regularize_pose(group.get_current_pose().pose)
            if common.all_close(pose_goal, pose_curr, tol):
                 return True
            if common.all_close(pose_curr.position, pose_temp.position, 0.001):
                cnt -= 1
            else:
                pose_temp = pose_curr
        return False

    def get_active_joint_names_of_all_groups(self):
        ret = []
        for group in self.move_groups:
            ret.append(group.get_active_joints())
        return ret

    def get_active_joint_names_of_group(self, group_name):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.move_groups[group_id].get_active_joints()

    def get_joint_states_of_all_groups(self):
        """Get joint states of all move groups

        :return: List[List[float]]
        """
        ret = []
        for group in self.move_groups:
            ret.append(group.get_current_joint_values())
        return ret

    def get_joint_states_of_group(self, group_name):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.move_groups[group_id].get_current_joint_values()

    def get_current_poses_of_all_groups(self):
        """Get the eef pose in ROS format.

        :return: List[PoseStamped]
        """
        ret = []
        for group in self.move_groups:
            ret.append(group.get_current_pose())
        return ret

    def get_current_pose_of_group(self, group_name, eef_name = ''):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.move_groups[group_id].get_current_pose(eef_name).pose

    def get_current_position_of_group(self, group_name):
        current_pose = self.get_current_pose_of_group(group_name)
        return current_pose.position

    def get_frame_of_group(self, group_name):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.ee_links[group_id], self.ref_frames[group_id]

    def group_go_to_joint_states(self, group_name, goal, tolerance=0):
        """Set the joint states of a group as goal.

        :param group_name: str Controlled group name
        :param goal: list Joint states
        :param tolerance: float
        :return: bool
        """
        group = self._get_group_by_name(group_name)
        if tolerance:
           joint_current = group.get_current_joint_values()
           constraints = MoveItMsg.Constraints()
           constraints.name = "freeze the J5 and J6"
        #    J5 constraint
           j5_constraint = MoveItMsg.JointConstraint()
           j5_constraint.position = joint_current[4]
           j5_constraint.tolerance_above = 0.0
           j5_constraint.tolerance_below = 0.0
           j5_constraint.weight = 1  
           j5_constraint.joint_name = "robot_joint5"
           constraints.joint_constraints.append(j5_constraint)
           rospy.logwarn('Path planning with J5 joint constraint !!!!!!!!!!!') 
           goal_constrainted = list(goal)
           goal_constrainted[4] = joint_current[4]       
        #    J6 constraint
           j6_constraint = MoveItMsg.JointConstraint()
           j6_constraint.position = joint_current[5]
           j6_constraint.tolerance_above = 0.0
           j6_constraint.tolerance_below = 0.0
           j6_constraint.weight = 1
           j6_constraint.joint_name = "robot_joint6"
           constraints.joint_constraints.append(j6_constraint)
           rospy.logwarn('Path planning with J6 joint constraint !!!!!!!!!!!') 
           goal_constrainted[5] = joint_current[5]
           goal = tuple(goal_constrainted)
           group.set_path_constraints(constraints)
        else:
           group.set_path_constraints(None) 

        ok = group.go(goal, wait=True)
        group.stop()
        group.clear_path_constraints()
        # return self._wait_js_goal_execution(group_name, goal, tolerance)
        return ok

    @staticmethod
    def _group_go_to_predefined_target(group):
        try:
            group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            return True
        except moveit_commander.MoveItCommanderException:
            group.stop()
            group.clear_pose_targets()
            return False

    # Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1]
    def group_set_max_velocity_scaling_factor(self, group_name, value):
        group = self._get_group_by_name(group_name)
        try:
            group.set_max_velocity_scaling_factor(value)
            rospy.logwarn('the velocity scaling factor of group [%s] has been set to [%f] !!!!!' % (group_name, value))
            return True
        except moveit_commander.MoveItCommanderException:
            return False

    def group_go_to_named_states(self, group_name, state_name):
        group = self._get_group_by_name(group_name)
        try:
            group.set_named_target(state_name)
            return self._group_go_to_predefined_target(group)
        except moveit_commander.MoveItCommanderException:
            return False

    def all_go_to_joint_states(self, goals):
        assert len(goals) == self.group_num
        for i, goal in enumerate(goals):
            ok = self.move_groups[i].go(goal, wait=True)
            if not ok:
                return False
        for group in self.move_groups:
            group.stop()
        return True

    def _group_go_to_pose_goal(self, group_name, goal, tolerance=0.01, constraint=''):
        """Set the pose of the tcp of a group as goal.

        :param group_name:
        :param goal:
        :param tolerance:
        :param constraint:
        :return:
        """
        print('excutepose')
        print(goal)
        group = self._get_group_by_name(group_name)
        group.set_pose_target(goal)
        try:
            ok, constraints = self.get_group_orientation_constraints(group_name, constraint)
            if ok:
                group.set_path_constraints(constraints)
            group.go(wait=True)
        except self.commander.MoveItCommanderException:
            group.stop()
            group.clear_pose_targets()
            group.clear_path_constraints()
            return False

        group.stop()
        group.clear_pose_targets()
        group.clear_path_constraints()
        # return self._wait_pose_goal_execution(group_name, goal, tolerance)
        return True

    def _local_base_to_global_pose(self, group_name, relative_pose, init_pose=None):
        """Convert a relative pose in local base frame to global base frame.

        :param group_name: str Planning group name
        :param relative_pose: Pose or List[float]
        :param init_pose: Initial pose of the robot before moving relatively
        :return: Pose
        """
        if not init_pose:
            current_pose = self.get_current_pose_of_group(group_name)
        else:
            current_pose = init_pose
        current_pose_mat = common.sd_pose(current_pose)  # T_be
        local_base_mat = transform.identity_matrix()
        local_base_mat[0:3, 3] = current_pose_mat[0:3, 3]  # T_bb'
        relative_pose_mat = common.sd_pose(relative_pose)  # T_b'e'
        absolute_pose_mat = np.dot(local_base_mat, relative_pose_mat)  # T_bb' * T_b'e' = T_be'
        return common.to_ros_pose(absolute_pose_mat)

    def _eef_pose_to_global_pose(self, group_name, relative_pose, init_pose=None):
        """Convert a relative pose in eef frame to global base frame.

        :param group_name: str Planning group name
        :param relative_pose: Pose or List[float]
        :param init_pose: Initial pose of the robot before moving relatively
        :return: Pose
        """
        if not init_pose:
            current_pose = self.get_current_pose_of_group(group_name)
        else:
            current_pose = init_pose
        current_pose_mat = common.sd_pose(current_pose)
        relative_pose_mat = common.sd_pose(relative_pose)
        absolute_pose_mat = np.dot(current_pose_mat, relative_pose_mat)  # T_b1 * T_12 = T_b2
        return common.to_ros_pose(absolute_pose_mat)

    def group_go_to_global_base_goal(self, group_name, goal, tolerance=0.01, constraint=''):
        """Move group to the goal pose wrt the global base frame

        :param group_name: Controlled group name
        :param goal: geometry_msgs.msg.Pose or PoseStamped
        :param tolerance:
        :param constraint: str, path constraint.
        :return: whether goal reached
        """
        if isinstance(goal, GeometryMsg.PoseStamped):
            goal_pose = goal.pose
        elif isinstance(goal, GeometryMsg.Pose):
            goal_pose = goal
        else:
            raise NotImplementedError('Goal of type {} is not defined'.format(type(goal)))

        return self._group_go_to_pose_goal(group_name, goal_pose, tolerance, constraint)

    def group_go_to_local_base_goal(self, group_name, goal, tolerance=0.01, constraint=''):
        """Move group to the goal pose wrt the local base frame

        :param group_name: Controlled group name
        :param goal: geometry_msgs.msg.Pose or PoseStamped
        :param tolerance:
        :param constraint: str, path constraint.
        :return: whether goal reached
        """
        if isinstance(goal, GeometryMsg.PoseStamped):
            goal_pose = goal.pose
        elif isinstance(goal, GeometryMsg.Pose):
            goal_pose = goal
        else:
            raise NotImplementedError('Goal of type {} is not defined'.format(type(goal)))

        abs_goal = self._local_base_to_global_pose(group_name, goal_pose)
        return self._group_go_to_pose_goal(group_name, abs_goal, tolerance, constraint)

    def group_go_to_eef_goal(self, group_name, goal, tolerance=0.01, constraint=''):
        """Move group to the goal pose wrt the eef frame

        :param group_name:
        :param goal:
        :param tolerance:
        :param constraint: str, path constraint.
        :return:
        """
        if isinstance(goal, GeometryMsg.PoseStamped):
            goal_pose = goal.pose
        elif isinstance(goal, GeometryMsg.Pose):
            goal_pose = goal
        else:
            raise NotImplementedError

        abs_goal = self._eef_pose_to_global_pose(group_name, goal_pose)
        return self._group_go_to_pose_goal(group_name, abs_goal, tolerance, constraint)

    def _group_go_to_position_goal(self, group_name, goal, tolerance=0.01):
        """Set the position of the tcp of a group as goal.

        :param group_name:
        :param goal:
        :param tolerance:
        :return:
        """
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        group = self.move_groups[group_id]

        group.set_position_target(goal)
        try:
            group.go(wait=True)
        except moveit_commander.MoveItCommanderException:
            group.stop()
            group.clear_pose_targets()
            return False

        group.stop()
        group.clear_pose_targets()

        current_position = self.get_current_position_of_group(group_name)
        # Standardize input before using all_close
        goal = common.sd_position(goal)
        current_position = common.sd_position(current_position)
        return common.all_close(goal, current_position, tolerance)

    def _to_absolute_position(self, group_name, relative_position):
        """Convert a relative position in eef frame to base frame.

        :param group_name: str Planning group name
        :param relative_position: Point or List[float]
        :return: ndarray
        """
        current_pose = self.get_current_pose_of_group(group_name)
        current_pose_mat = common.sd_pose(current_pose)
        relative_pose_mat = transform.identity_matrix()
        relative_pose_mat[0:3, 3] = common.sd_position(relative_position)
        absolute_pose_mat = np.dot(current_pose_mat, relative_pose_mat)  # T_b1 * T_12 = T_b2
        absolute_position = absolute_pose_mat[0:3, 3]
        return absolute_position

    def group_go_to_absolute_position_goal(self, group_name, goal, tolerance=0.001):
        goal = common.sd_position(goal)
        return self._group_go_to_position_goal(group_name, goal, tolerance)

    def group_go_to_relative_position_goal(self, group_name, goal, tolerance=0.001):
        goal = common.sd_position(goal)
        abs_goal = self._to_absolute_position(group_name, goal)
        return self._group_go_to_position_goal(group_name, abs_goal, tolerance)

    def group_shift(self, group_name, axis, goal):
        """Move the group along given axis and shift goal

        :param group_name:
        :param axis: str Axis id, could be x y z r p y
        :param goal: float Goal value of given axis
        :return:
        """
        group = self._get_group_by_name(group_name)
        # 0,1,2,3,4,5 for x y z roll pitch yaw
        axis_list = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        axis_id = -1
        for i, a in enumerate(axis_list):
            if a == axis:
                axis_id = i
                break

        if axis_id < 0:
            raise NotImplementedError

        group.shift_pose_target(axis_id, goal)

        try:
            group.go(wait=True)
        except moveit_commander.MoveItCommanderException:
            group.stop()
            group.clear_pose_targets()
            return False

        group.stop()
        group.clear_pose_targets()
        return True

    def _all_go_to_pose_goal(self, goals):
        plans = []
        for i, goal in enumerate(goals):
            self.move_groups[i].set_pose_target(goal)
            plan = self.move_groups[i].plan()
            plans.append(plan)

        for plan, group in zip(plans, self.move_groups):
            group.execute(plan, wait=True)

        for group in self.move_groups:
            group.stop()
            group.clear_pose_targets()
        return True

    def all_go_to_absolute_pose_goal(self, goals):
        if isinstance(goals, GeometryMsg.PoseArray):
            goals = goals.poses
        assert len(goals) == self.group_num
        return self._all_go_to_pose_goal(goals)

    def all_go_to_relative_pose_goal(self, goals):
        if isinstance(goals, GeometryMsg.PoseArray):
            goals = goals.poses
        assert len(goals) == self.group_num
        abs_goals = []
        for name, goal in zip(self.group_names, goals):
            abs_goals.append(self._to_absolute_pose(name, goal))
        return self._all_go_to_pose_goal(abs_goals)

    def _update_plan_time_stamps(self, group, plan, stamp):
        original_stamp = plan.joint_trajectory.points[-1].time_from_start.to_sec()
        # points_num = len(plan.joint_trajectory.points)
        velocity_scale = original_stamp / stamp
        acceleration_scale = velocity_scale
        curr_state = self.commander.get_current_state()
        updated_plan = group.retime_trajectory(curr_state, plan, velocity_scale, acceleration_scale)
        # for i in range(points_num):
        #     plan.joint_trajectory.points[i].time_from_start = rospy.Duration.from_sec(t * (i + 1))
        return updated_plan

    def build_absolute_path_for_group(self, group_name, poses, stamp=None, avoid_collisions=True, eef_step=0.1, joint_constraint=0):
        """Given way points in a list of geometry_msgs.Pose, plan a path
        go through all way points.

        :param group_name: Group name for building plan
        :param poses: geometry_msgs.PoseArray or List[Pose]
        :param stamp: Last time stamp from start
        :param avoid_collisions:
        """
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        group = self.move_groups[group_id]

        if joint_constraint:
           joint_current = group.get_current_joint_values()
           constraints = MoveItMsg.Constraints()
           constraints.name = "freeze the J5 and J6"
        #    J5 constraint
           j5_constraint = MoveItMsg.JointConstraint()
           j5_constraint.position = joint_current[4]
           j5_constraint.tolerance_above = 0.0
           j5_constraint.tolerance_below = 0.0
           j5_constraint.weight = 1  
           j5_constraint.joint_name = "robot_joint5"
           constraints.joint_constraints.append(j5_constraint)
           rospy.logwarn('Path planning with J5 joint constraint !!!!!!!!!!!')           
        #    J6 constraint
           j6_constraint = MoveItMsg.JointConstraint()
           j6_constraint.position = joint_current[5]
           j6_constraint.tolerance_above = 0.0
           j6_constraint.tolerance_below = 0.0
           j6_constraint.weight = 1
           j6_constraint.joint_name = "robot_joint6"
           constraints.joint_constraints.append(j6_constraint)
           rospy.logwarn('Path planning with J6 joint constraint !!!!!!!!!!!')
           group.set_path_constraints(constraints)
        else:
           group.set_path_constraints(None) 

        if isinstance(poses, GeometryMsg.PoseArray):
            poses = poses.poses
        print('excutemanyposes')
        print(poses)
        plan, fraction = group.compute_cartesian_path(poses, eef_step, jump_threshold=0,
                                                      avoid_collisions=avoid_collisions)
        if fraction == 1.0:
            rospy.logwarn('Path planning succussed, fraction is %f !!!!' % fraction)            
        elif fraction < 1.0 and fraction >= 0:   
            rospy.logerr('Path planning partially succussed, fraction is only %f !!!!' % fraction)
        # move_group_interface.h  L754
        elif fraction < 0:
            rospy.logerr('Path planning failed, fraction is smaller than 0.')

        if stamp:
            plan = self._update_plan_time_stamps(group, plan, stamp)
        return plan

    def build_relative_path_for_group(self, group_name, poses, stamp=None, avoid_collisions=True):
        """Build a path composed by relative poses for the planning group.

        :param group_name: String Planning group name
        :param poses: PoseArray or List[Pose] Each pose is relevant to the last one
        :param stamp: Double Time stamp for the last pose
        :param avoid_collisions:
        :return:
        """
        if isinstance(poses, GeometryMsg.PoseArray):
            poses = poses.poses
        init_pose = None
        abs_poses = []
        for rel_pose in poses:
            abs_pose = self._to_absolute_pose(group_name, rel_pose, init_pose)
            abs_poses.append(abs_pose)
            init_pose = abs_pose
        return self.build_absolute_path_for_group(group_name, abs_poses, stamp, avoid_collisions)

    def execute_plan_for_group(self, group_name, plan, wait=True):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        ok = self.move_groups[group_id].execute(plan, wait)  
        self.move_groups[group_id].clear_path_constraints()
        return ok

    def build_absolute_paths_for_all(self, all_poses, all_stamps=None, avoid_collisions=True):
        """

        :param all_poses: List[List[Pose]] or List[PoseArray]
        :param all_stamps:
        :param avoid_collisions:
        :return:
        """
        all_plans = []
        if all_stamps:
            assert len(all_poses) == len(all_stamps), \
                "all_poses len {} is not equal to all_stamps len {}".format(len(all_poses), len(all_stamps))
        else:
            all_stamps = [None] * len(all_poses)
        for i, poses in enumerate(all_poses):
            if isinstance(poses, GeometryMsg.PoseArray):
                poses = poses.poses
            group_name = self.group_names[i]
            plan = self.build_absolute_path_for_group(group_name, poses, all_stamps[i], avoid_collisions)
            all_plans.append(plan)
        return all_plans

    def build_relative_paths_for_all(self, all_poses, all_stamps=None, avoid_collisions=True):
        all_abs_poses = []
        for i, rel_poses in enumerate(all_poses):
            if isinstance(rel_poses, GeometryMsg.PoseArray):
                rel_poses = rel_poses.poses
            group_name = self.group_names[i]
            init_pose = None
            abs_poses_of_group = []
            for rel_pose in rel_poses:
                abs_pose = self._to_absolute_pose(group_name, rel_pose, init_pose)
                abs_poses_of_group.append(abs_pose)
                init_pose = abs_pose
            all_abs_poses.append(abs_poses_of_group)
        return self.build_absolute_paths_for_all(all_abs_poses, all_stamps, avoid_collisions)

    def execute_plans_for_all(self, plans):
        assert len(plans) == self.group_num
        for i, plan in enumerate(plans):
            group = self.move_groups[i]
            ok = group.execute(plan, wait=True)
            if not ok:
                return False
        return True

    def all_go_to_pose_goals(self, goals, is_absolute, stamps=None, avoid_collision=True):
        if isinstance(goals, GeometryMsg.PoseArray):
            goals = goals.poses
        assert len(goals) == self.group_num
        for i, goal in enumerate(goals):
            group_name = self.group_names[i]
            try:
                stamp = stamps[i] if stamps else None
            except IndexError:
                stamp = None
            if is_absolute:
                plan = self.build_absolute_path_for_group(group_name, [goal], stamp, avoid_collision)
            else:
                plan = self.build_relative_path_for_group(group_name, [goal], stamp, avoid_collision)

            ok = self.execute_plan_for_group(group_name, plan, wait=True)
            if not ok:
                return False
        return True

    def add_box(self, group_name, box_name, box_pose, box_size, is_absolute, auto_subfix=False):
        group = self._get_group_by_name(group_name)
        box_pose_stamped = GeometryMsg.PoseStamped()
        box_pose_stamped.pose = box_pose
        if is_absolute:
            box_pose_stamped.header.frame_id = group.get_planning_frame()
        else:
            box_pose_stamped.header.frame_id = group.get_end_effector_link()
        if auto_subfix:
            box_name += str(self._obj_suffix)
            self._obj_suffix += 1

        assert isinstance(box_size, GeometryMsg.Point)
        # size must be iterable
        self.scene.add_box(box_name, box_pose_stamped, size=(box_size.x, box_size.y, box_size.z))

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < 3) and not rospy.is_shutdown():
            if box_name in self.scene.get_known_object_names():
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return True

    def attach_box(self, group_name, eef_group_name, box_name, box_pose, box_size):
        ok = self.add_box(group_name, box_name, box_pose, box_size, is_absolute=False, auto_subfix=False)
        if not ok:
            return False
        group = self._get_group_by_name(group_name)
        grasping_group = eef_group_name
        touch_links = self.commander.get_link_names(group=grasping_group)
        self.scene.attach_box(group.get_end_effector_link(), box_name, touch_links=touch_links)
        return True

    def detach_object(self, group_name, obj_name):
        group = self._get_group_by_name(group_name)
        self.scene.remove_attached_object(group.get_end_effector_link(), name=obj_name)
        return True

    def remove_object(self, obj_name, is_exact=False):
        if is_exact:
            self.scene.remove_world_object(obj_name)
        else:
            for name in self.scene.get_known_object_names():
                if obj_name in name:
                    self.scene.remove_world_object(name)
        return True

    def add_plane(self, group_name, plane_name, plane_pose, plane_normal):
        group = self._get_group_by_name(group_name)
        plane_pose_stamped = GeometryMsg.PoseStamped()
        plane_pose_stamped.pose = plane_pose
        plane_pose_stamped.header.frame_id = group.get_planning_frame()
        self.scene.add_plane(plane_name, plane_pose_stamped, normal=(plane_normal.x, plane_normal.y, plane_normal.z))
        return True

    def get_group_orientation_constraints(self, group_name, constraint=''):
        """

        :return:
        """
        if not constraint:
            return False, None

        g_id = self._get_group_id(group_name)
        group = self._get_group_by_name(group_name)
        ref_pose = group.get_current_pose().pose
        constraints = MoveItMsg.Constraints()

        oc = MoveItMsg.OrientationConstraint()
        oc.orientation.x = ref_pose.orientation.x
        oc.orientation.y = ref_pose.orientation.y
        oc.orientation.z = ref_pose.orientation.z
        oc.orientation.w = ref_pose.orientation.w

        if 'r' in constraint:
            oc.absolute_x_axis_tolerance = math.pi  # roll
        else:
            oc.absolute_x_axis_tolerance = math.pi * 4

        if 'p' in constraint:
            oc.absolute_y_axis_tolerance = math.pi  # pitch
        else:
            oc.absolute_y_axis_tolerance = math.pi * 4

        if 'y' in constraint:
            oc.absolute_z_axis_tolerance = math.pi  # yaw
        else:
            oc.absolute_z_axis_tolerance = math.pi * 4

        oc.weight = 1.0
        oc.link_name = self.ee_links[g_id]
        oc.header.frame_id = self.ref_frames[g_id]
        oc.header.stamp = rospy.Time.now()
        constraints.orientation_constraints.append(oc)

        return True, constraints
