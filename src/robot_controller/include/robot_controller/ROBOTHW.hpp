#pragma once 

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <mutex>
#include "robot_controller/RobotDriver.hpp"

using namespace hardware_interface;

class ROBOTHW: public hardware_interface::RobotHW
{
public:
  ROBOTHW(RobotDriver *robot_driver, bool use_arm_controller);

  virtual void read();
  virtual void write();

private:
  RobotDriver *robot_driver_;

  // hardware interfaces
  JointStateInterface jointStateInterface_;
  PositionJointInterface positionJointInterface_;

  std::timed_mutex jointStatesMutex;

  // Shared memory
  std::vector<std::string> jointNames;
  std::vector<double> currentJointPosition;
  std::vector<double> currentJointVelocity;
  std::vector<double> currentJointEffort;
  std::vector<double> jointPositionCommand;

  bool use_arm_controller_;
};

