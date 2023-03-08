#!/usr/bin/env python2
import sys
import math
import argparse
import os
import rospy

from controller import Supervisor
from joint_state_publisher import JointStatePublisher
from trajectory_follower import TrajectoryFollower
from rosgraph_msgs.msg import Clock

from std_srvs.srv import *
from geometry_msgs.msg import Pose
from robot_webots.srv import *



parser = argparse.ArgumentParser()
parser.add_argument('--node-name', dest='nodeName',
                    default='aubo_driver', help='Specifies the name of the node.')
parser.add_argument('--webots-robot-name', dest='webotsRobotName', default='aubo',
                    help='Specifies the "name" field of the robot in Webots.')
arguments, unknown = parser.parse_known_args()
if arguments.webotsRobotName:
    os.environ['WEBOTS_ROBOT_NAME'] = arguments.webotsRobotName

rospy.init_node(arguments.nodeName, disable_signals=True)


jointPrefix = rospy.get_param('prefix', '')
if jointPrefix:
    print('Setting prefix to %s' % jointPrefix)

robot = Supervisor()
root_field_ = robot.getRoot().getField('children')
rangefinder = robot.getRangeFinder('RangeFinder')
camera = robot.getCamera('camera')

jointNames = ['shoulder_joint', 
              'upperArm_joint', 
              'foreArm_joint', 
              'wrist1_joint', 
              'wrist2_joint', 
              'wrist3_joint'
]
jointstates_namespace = "aubo"
jointStatePublisher = JointStatePublisher(robot, jointPrefix, jointNames, jointstates_namespace)
rospy.logwarn('Aubo JointStatePublisher is initialized')

controller_namespace = "aubo/aubo_controller"
trajectoryFollower = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, jointNames, controller_namespace)
trajectoryFollower.start()
rospy.logwarn('Aubo TrajectoryFollower is initialized')

# we want to use simulation time for ROS
# clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
# if not rospy.get_param('use_sim_time', False):
#     rospy.logwarn('use_sim_time is not set!')
# deleteBox Callback
def deleteBoxSrvCb(req):
    # rospy.sleep(0.5)
    resp = TriggerResponse()
    last_obj_id = root_field_.getCount()-1
    obj_node = root_field_.getMFNode(last_obj_id)
    name_field = obj_node.getField('name')
    if name_field.getSFString().find('box') != -1:
        obj_node.remove()
        rospy.logwarn('Box has been deleted!')
        resp.success = True
    else:
        rospy.logerr('can not delete box !!!!!!!!')
        resp.success = False
    return resp

srv_delete_box = rospy.Service(
    '/simulation/supervisor/delete_box', Trigger, deleteBoxSrvCb)    
timestep = int(robot.getBasicTimeStep())
rangefinder.enable(timestep)
camera.enable(timestep)
rospy.logwarn('Aubo HV1000 is initialized')
while robot.step(timestep) != -1 and not rospy.is_shutdown():
    jointStatePublisher.publish()
    trajectoryFollower.update()
    # pulish simulation clock
    # msg = Clock()
    # time = robot.getTime()
    # msg.clock.secs = int(time)
    # # round prevents precision issues that can cause problems with ROS timers
    # msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
    # clockPublisher.publish(msg)
