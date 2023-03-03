#pragma once

#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/JointState.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Scalar.h>

#include <cstdio>
#include <mutex>

#include <std_msgs/Bool.h>
//include robot control lib here


class RobotDriver {
public:
  RobotDriver();
  RobotDriver(std::string ip_address, int gripper_port);
  ~RobotDriver();

  virtual bool read(sensor_msgs::JointState &msg);
  virtual bool write(sensor_msgs::JointState msg);

  virtual bool controlGripper(bool enable);

protected:
  // Robot handle
  int gripperPort;

  std::vector<std::string> jointNames;
  std::vector<double> currentJointPosition;

  virtual bool connect(std::string ip_address);
  virtual bool getJointStates();
  virtual bool setJointStates(std::vector<double> js_cmd, bool accurate);
  bool controlJointPosition(const std::vector<double>& js_cmd, float tol, bool accurate);
  bool checkAllClose(std::vector<double> a, std::vector<double> b, float tol);
};


