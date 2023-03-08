from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
import rospy
import actionlib
import time, numpy, math, copy, sys
sys.path.append("..")
from base.curi_robot_control import robot

class curi_ros_trajectory_action(robot):
    def __init__(self, joint_size, joint_name, dt, goal_time_tolerance=None):
        robot.__init__(self, joint_size, numpy.array([0.0] * joint_size))

        self.goal_time_tolerance = goal_time_tolerance or rospy.Duration(0.0)
        self.joint_goal_tolerances = [0.05] * joint_size
        self.following_lock = True
        self.T0 = time.time()
        self.server = actionlib.ActionServer("robot/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction,
                                             self.on_goal, self.on_cancel, auto_start=False)

        self.goal_handle = None
        self.traj = None
        self.traj_t0 = 0.0
        self.first_waypoint_id = 10
        self.last_point_sent = True
        self.joint_name = joint_name
        self.dt = dt

        self.update_timer = rospy.Timer(rospy.Duration(self.dt), self._update)
 
    def set_robot(self, robot):
        # Cancels any goals in progress
        if self.goal_handle:
            self.goal_handle.set_canceled()
            self.goal_handle = None
        self.traj = None
        self.init_traj_from_robot()
    
    # Reorders the JointTrajectory traj according to the order in
    # joint_names.  Destructive.
    def reorder_traj_joints(self, traj, joint_names):
        order = [traj.joint_names.index(j) for j in joint_names]

        new_points = []
        for p in traj.points:
            new_points.append(JointTrajectoryPoint(
                positions = [p.positions[i] for i in order],
                velocities = [p.velocities[i] for i in order] if p.velocities else [],
                accelerations = [p.accelerations[i] for i in order] if p.accelerations else [],
                time_from_start = p.time_from_start))
        traj.joint_names = joint_names
        traj.points = new_points

    def interp_cubic(self, p0, p1, t_abs):
        T = (p1.time_from_start - p0.time_from_start).to_sec()
        t = t_abs - p0.time_from_start.to_sec()
        q = [0] * self.JointSize
        qdot = [0] * self.JointSize
        qddot = [0] * self.JointSize
        for i in range(len(p0.positions)):
            a = p0.positions[i]
            b = p0.velocities[i]
            c = (-3*p0.positions[i] + 3*p1.positions[i] - 2*T*p0.velocities[i] - T*p1.velocities[i]) / T**2
            d = (2*p0.positions[i] - 2*p1.positions[i] + T*p0.velocities[i] + T*p1.velocities[i]) / T**3

            q[i] = a + b*t + c*t**2 + d*t**3
            qdot[i] = b + 2*c*t + 3*d*t**2
            qddot[i] = 2*c + self.JointSize*d*t
        return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot, time_from_start=rospy.Duration(t_abs))

    def interp_c(self, p0, p1, t_abs):
        T = (p1.time_from_start - p0.time_from_start).to_sec()
        t = t_abs - p0.time_from_start.to_sec()
        
        return t

    # Returns (q, qdot, qddot) for sampling the JointTrajectory at time t.
    # The time t is the time since the trajectory was started.
    def sample_traj(self, traj, t):
        # First point
        if t <= 0.0:
            return copy.deepcopy(traj.points[0])
        # Last point
        if t >= traj.points[-1].time_from_start.to_sec():
            return copy.deepcopy(traj.points[-1])
        
        # Finds the (middle) segment containing t
        i = 0
        while traj.points[i+1].time_from_start.to_sec() < t:
            i += 1
        return self.interp_cubic(traj.points[i], traj.points[i+1], t)


    def sample_t(self, traj, t):
        # First point
        if t <= 0.0:
            return copy.deepcopy(traj.points[0])
        # Last point
        if t >= traj.points[-1].time_from_start.to_sec():
            return copy.deepcopy(traj.points[-1])
        
        # Finds the (middle) segment containing t
        i = 0
        while traj.points[i+1].time_from_start.to_sec() < t:
            i += 1
        return self.interp_c(traj.points[i], traj.points[i+1], t)

    def traj_is_finite(self, traj):
        for pt in traj.points:
            for p in pt.positions:
                if math.isinf(p) or math.isnan(p):
                    return False
            for v in pt.velocities:
                if math.isinf(v) or math.isnan(v):
                    return False
        return True
            
    def has_limited_velocities(self, traj):
        max_velocity = 5.0
        for p in traj.points:
            for v in p.velocities:
                if math.fabs(v) > max_velocity:
                    return False
        return True

    def has_velocities(self, traj):
        for p in traj.points:
            if len(p.velocities) != len(p.positions):
                return False
        return True

    def within_tolerance(self, a_vec, b_vec, tol_vec):
        for a, b, tol in zip(a_vec, b_vec, tol_vec):
            if abs(a - b) > tol:
                return False
        return True

    # Sets the trajectory to remain stationary at the current position
    # of the robot.
    def init_traj_from_robot(self):
        self.traj_t0 = time.time()
        self.traj = JointTrajectory()
        self.traj.joint_names = self.joint_name
        self.traj.points = [JointTrajectoryPoint(
            positions = self.JointCurPos,
            velocities = [0] * self.JointSize,
            accelerations = [0] * self.JointSize,
            time_from_start = rospy.Duration(0.0))]

    def start(self):
        self.init_traj_from_robot()
        self.server.start()
        print("The action server for this driver has been started")

    def on_goal(self, goal_handle):
        # Checks that the robot is connected
        if not self.ConnectRobot:
            rospy.logerr("Received a goal, but the robot is not connected")
            goal_handle.set_rejected()
            return

        # Checks if the joints are just incorrect
        if set(goal_handle.get_goal().trajectory.joint_names) != set(self.joint_name):
            rospy.logerr("Received a goal with incorrect joint names: (%s)" % \
                         ', '.join(goal_handle.get_goal().trajectory.joint_names))
            goal_handle.set_rejected()
            return

        if not self.traj_is_finite(goal_handle.get_goal().trajectory):
            rospy.logerr("Received a goal with infinites or NaNs")
            goal_handle.set_rejected(text="Received a goal with infinites or NaNs")
            return
        
        # Checks that the trajectory has velocities
        if not self.has_velocities(goal_handle.get_goal().trajectory):
            rospy.logerr("Received a goal without velocities")
            goal_handle.set_rejected(text="Received a goal without velocities")
            return

        # Checks that the velocities are withing the specified limits
        if not self.has_limited_velocities(goal_handle.get_goal().trajectory):
            message = "Received a goal with velocities that are higher than %f" % max_velocity
            rospy.logerr(message)
            goal_handle.set_rejected(text=message)
            return

        # Orders the joints of the trajectory according to joint_names
        self.reorder_traj_joints(goal_handle.get_goal().trajectory, self.joint_name)
        
        if self.following_lock:
            if self.goal_handle:
                # Cancels the existing goal
                self.goal_handle.set_canceled()
                self.first_waypoint_id += len(self.goal_handle.get_goal().trajectory.points)
                self.goal_handle = None

            # Inserts the current setpoint at the head of the trajectory
            now = time.time()
            point0 = self.sample_traj(self.traj, now - self.traj_t0)
            point0.time_from_start = rospy.Duration(0.0)
            goal_handle.get_goal().trajectory.points.insert(0, point0)
            self.traj_t0 = now

            # Replaces the goal
            self.goal_handle = goal_handle
            self.traj = goal_handle.get_goal().trajectory
            self.goal_handle.set_accepted()

    def on_cancel(self, goal_handle):
        if goal_handle == self.goal_handle:
            if self.following_lock:
                # Uses the next little bit of trajectory to slow to a stop
                STOP_DURATION = 0.5
                now = time.time()
                point0 = self.sample_traj(self.traj, now - self.traj_t0)
                point0.time_from_start = rospy.Duration(0.0)
                point1 = self.sample_traj(self.traj, now - self.traj_t0 + STOP_DURATION)
                point1.velocities = [0] * self.JointSize
                point1.accelerations = [0] * self.JointSize
                point1.time_from_start = rospy.Duration(STOP_DURATION)
                self.traj_t0 = now
                self.traj = JointTrajectory()
                self.traj.joint_names = self.joint_name
                self.traj.points = [point0, point1]
                self.goal_handle.set_canceled()
                self.goal_handle = None
        else:
            goal_handle.set_canceled()

    def _update(self, event):
        # print('update', self.JointCmdPos)
        if self.ConnectRobot and self.traj:
            now = time.time()
            if (now - self.traj_t0) <= self.traj.points[-1].time_from_start.to_sec():
                self.last_point_sent = False #sending intermediate points
                setpoint = self.sample_traj(self.traj, now - self.traj_t0)
                self.JointCmdPos = numpy.array(setpoint.positions)
                self.tx = setpoint.time_from_start.to_sec()
                    
            elif not self.last_point_sent:
                # All intermediate points sent, sending last point to make sure we
                # reach the goal.
                # This should solve an issue where the robot does not reach the final
                # position and errors out due to not reaching the goal point.
                last_point = self.traj.points[-1]
                position_in_tol = self.within_tolerance(self.JointCurPos, last_point.positions, self.joint_goal_tolerances)
                # Performing this check to try and catch our error condition.  We will always
                # send the last point just in case.
                if not position_in_tol:
                    rospy.logwarn("Trajectory time exceeded and current robot state not at goal, last point required")
                    rospy.logwarn("Current trajectory time: %s, last point time: %s" % \
                                (now - self.traj_t0, self.traj.points[-1].time_from_start.to_sec()))
                    rospy.logwarn("Desired: %s\nactual: %s\nvelocity: %s" % \
                                          (last_point.positions, self.JointCurPos, self.JointCurVel))
                setpoint = self.sample_traj(self.traj, self.traj.points[-1].time_from_start.to_sec())
                self.JointCmdPos = numpy.array(setpoint.positions)
                self.last_point_sent = True
                    
            else:  # Off the end
                if self.goal_handle:
                    last_point = self.traj.points[-1]
                    position_in_tol = self.within_tolerance(self.JointCurPos, last_point.positions, [0.1]*self.JointSize)
                    velocity_in_tol = self.within_tolerance(self.JointCurVel, last_point.velocities, [0.05]*self.JointSize)
                    if position_in_tol and velocity_in_tol:
                        # The arm reached the goal (and isn't moving).  Succeeding
                        self.goal_handle.set_succeeded()
                        self.goal_handle = None
