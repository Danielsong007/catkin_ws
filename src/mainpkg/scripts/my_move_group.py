#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class my_move_group(object):
  def __init__(self):
    super(my_move_group, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.move_group = moveit_commander.MoveGroupCommander("arm")
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    self.planning_frame = self.move_group.get_planning_frame()
    self.eef_link = self.move_group.get_end_effector_link()
    self.group_names = self.robot.get_group_names() # robot.get_current_state()
    self.box_name = ''
  
  def all_close(self, goal, actual, tolerance):
    if type(goal) is list:
      for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
          return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
      return self.all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
      return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True

  def go_to_joint_state(self,J1,J2,J3,J4,J5):
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = J1
    joint_goal[1] = J2
    joint_goal[2] = J3
    joint_goal[3] = J4
    joint_goal[4] = J5
    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()
    current_joints = self.move_group.get_current_joint_values()
    return self.all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self,ox,oy,oz,ow,px,py,pz):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = ox
    pose_goal.orientation.y = oy
    pose_goal.orientation.z = oz
    pose_goal.orientation.w = ow
    pose_goal.position.x = px
    pose_goal.position.y = py
    pose_goal.position.z = pz
    self.move_group.set_pose_target(pose_goal)
    plan = self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    print('Current pose is:', current_pose)
    return self.all_close(pose_goal, current_pose, 0.01)

  def plan_cartesian_path(self, scale=1):
    waypoints = []
    wpose =self.move_group.get_current_pose().pose
    wpose.position.z += scale * 0.1  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z -= scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z += scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) =self.move_group.compute_cartesian_path(waypoints, 0.001, 0.0) # waypoints to follow, eef_step, jump_threshold
    return plan, fraction

  def display_trajectory(self, plan):
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)

  def execute_plan(self, plan):
   self.move_group.execute(plan, wait=True)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = self.scene.get_attached_objects([self.box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = self.box_name in self.scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

  def add_box(self, timeout=4):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "Link_5"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.07
    self.box_name = "mbox"
    self.scene.add_box(self.box_name, box_pose, size=(0.2, 0.3, 1.6))
    self.box_name=self.box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_box(self, timeout=4):
    grasping_group = 'hand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    self.scene.remove_attached_object(self.eef_link, name=self.box_name)
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    self.scene.remove_world_object(self.box_name)
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

if __name__ == '__main__':
  try:
    my_move_group = my_move_group()
    # my_move_group.go_to_joint_state(0,0,0,0,0)
    my_move_group.go_to_pose_goal(0,0,-0.000279156683585,0.999999961036,0.5,0.56,0.4)
    # cartesian_plan, fraction = my_move_group.plan_cartesian_path(scale=3)
    # my_move_group.display_trajectory(cartesian_plan)
    # my_move_group.execute_plan(cartesian_plan)
    # my_move_group.add_box()
    # my_move_group.attach_box()
    # my_move_group.detach_box()
    # my_move_group.remove_box()
  except rospy.ROSInterruptException:
    pass
