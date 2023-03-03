#pragma once

#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Scalar.h>

#include <cstdio>
#include <mutex>

#include "robot_controller/RobotDriver.hpp"
#include <std_msgs/Bool.h>


class RobotFakeDriver : public RobotDriver {
public:
  RobotFakeDriver();
  ~RobotFakeDriver();

  virtual bool read(sensor_msgs::JointState &msg);
  virtual bool write(sensor_msgs::JointState msg);

  virtual bool controlGripper(bool enable);
  virtual bool executeTrajectory(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg);

protected:

  virtual bool connect(std::string ip_address);
  virtual bool getJointStates(std::vector<double> &positions);
  virtual bool setJointStates(std::vector<double> js_cmd, bool accurate);

private:
  double idx;
  std::vector<double> currentJointPosition;

};


