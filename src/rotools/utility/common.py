import numpy as np

try:
    import rospy
    import geometry_msgs.msg as GeometryMsg
    import moveit_msgs.msg as MoveItMsg
    import trajectory_msgs.msg as TrajectoryMsg

    from moveit_commander.conversions import pose_to_list
except ImportError:
    pass

from rotools.utility import transform


def all_close(goal, actual, tolerance):
    """Test if a list of values are within a tolerance of their counterparts in another list.

    :param goal: A list of floats, a Pose or a PoseStamped
    :param actual: A list of floats, a Pose or a PoseStamped
    :param tolerance: float
    :returns: bool
    """
    if type(goal) is list or type(goal) is tuple:
        if not np.allclose(goal, actual, atol=tolerance):
            return False
    elif isinstance(goal, np.ndarray) and isinstance(actual, np.ndarray):
        return all_close(list(goal), list(actual), tolerance)
    elif type(goal) is GeometryMsg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is GeometryMsg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    elif type(goal) is GeometryMsg.Point:
        pose_to_list_g = [goal.x, goal.y, goal.z]
        pose_to_list_a = [actual.x, actual.y, actual.z]
        return all_close(pose_to_list_g, pose_to_list_a, tolerance)
    else:
        raise NotImplementedError('Goal type is {} while actual pose type is {}'.format(type(goal), type(actual)))
    return True


def sd_joint_state():
    pass


def offset_ros_pose(pose, offset):
    output = GeometryMsg.Pose()
    output.position.x  = pose.position.x + offset[0]
    output.position.y  = pose.position.y + offset[1]
    output.position.z  = pose.position.z + offset[2]
    output.orientation = pose.orientation
    return output


def regularize_pose(pose):
    """It is odd if we do not regularize the pose

    :param pose: geometry_msgs/Pose
    :return:
    """
    pose_mat = sd_pose(pose)
    return to_ros_pose(pose_mat)


def sd_pose(pose):
    """Standardize the input pose to the 4x4 homogeneous transformation
    matrix in special Euclidean group SE(3).

    :param pose:
    :return: transformation matrix
    """
    if isinstance(pose, np.ndarray):
        if pose.ndim == 1 and pose.size == 7:
            t = pose[:3]
            q = pose[3:]
            tm = transform.translation_matrix(t)
            rm = transform.quaternion_matrix(q)
            # make sure to let tm left product rm
            return np.dot(tm, rm)
        elif pose.ndim == 1 and pose.size == 6:
            t = pose[:3]
            rpy = pose[3:]
            tm = transform.translation_matrix(t)
            rm = transform.euler_matrix(rpy[0], rpy[1], rpy[2])
            return np.dot(tm, rm)
        elif pose.shape == (4, 4):
            return pose
        else:
            raise NotImplementedError
    elif isinstance(pose, list):
        return sd_pose(np.array(pose))
    elif isinstance(pose, GeometryMsg.Pose):
        p = pose.position
        o = pose.orientation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    elif isinstance(pose, GeometryMsg.Transform):
        p = pose.transform.translation
        o = pose.transform.rotation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    elif isinstance(pose, GeometryMsg.TransformStamped):
        p = pose.transform.translation
        o = pose.transform.rotation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    elif isinstance(pose, GeometryMsg.PoseStamped):
        p = pose.pose.position
        o = pose.pose.orientation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    else:
        raise NotImplementedError


def to_ros_pose(pose):
    """Convert standard pose in 4x4 matrix to ROS geometry msg pose

    :param pose: ndarray, standard pose matrix representing a single pose
    :return: geometry_msgs.Pose
    """
    if isinstance(pose, np.ndarray):
        if pose.shape == (4, 4):
            t = transform.translation_from_matrix(pose)
            q = transform.quaternion_from_matrix(pose)
            msg = GeometryMsg.Pose()
            msg.position.x = t[0]
            msg.position.y = t[1]
            msg.position.z = t[2]
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            return msg
        elif pose.size == 7:
            msg = GeometryMsg.Pose()
            msg.position.x = pose[0]
            msg.position.y = pose[1]
            msg.position.z = pose[2]
            msg.orientation.x = pose[3]
            msg.orientation.y = pose[4]
            msg.orientation.z = pose[5]
            msg.orientation.w = pose[6]
            return msg
        else:
            raise NotImplementedError
    else:
        raise NotImplementedError


def sd_position(position):
    if isinstance(position, np.ndarray):
        if position.shape == (3,):
            return position
        else:
            raise NotImplementedError
    elif isinstance(position, list):
        return sd_position(np.array(position))
    elif isinstance(position, GeometryMsg.Point):
        return sd_position(np.array([position.x, position.y, position.z]))
    else:
        raise NotImplementedError


def to_ros_plan(t, p, v=None, a=None):
    """Convert a series of time stamps and positions to ros MoveItMsg.RobotTrajectory msg.
    Note that the msg contains no joint name, which need to be added explicitly.

    :param t: timestamp of shape N
    :param p: way point positions of shape [dim, N]
    :param v: way point velocities of shape [dim, N], could be all 0
    :param a: way point accelerations of shape [dim, N], could be all 0
    :return: MoveItMsg.RobotTrajectory with joint names be empty
    """
    msg = MoveItMsg.RobotTrajectory()
    way_point_num = t.size
    dim = p.shape[0]
    zero_list = np.zeros(dim).tolist()

    for w in range(way_point_num):
        if w == 0:
            continue  # omit the starting point identical to the current pose
        wpt = TrajectoryMsg.JointTrajectoryPoint()

        wpt.positions = list(p[:, w])
        wpt.velocities = zero_list if v is None else list(v[:, w])
        wpt.accelerations = zero_list if a is None else list(a[:, w])
        wpt.time_from_start = rospy.Duration.from_sec(t[w])
        msg.joint_trajectory.points.append(wpt)

    return msg


def get_param(name, value=None):
    """Get ros param from param server

    :param name: String Param name
    :param value: Return value if param is not set
    :return:
    """
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


def set_param(name, value):
    """Set ros param to param server

    :param name: String Param name
    :param value: Input value to be sent to ros param
    :return:
    """
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.set_param(private, value)
    elif rospy.has_param(name):
        return rospy.set_param(name, value)
    else:
        raise NotImplementedError('There is no ros_param called {}'.format(name))