//
// Created by andylee on 2021/1/26.
//
#include <cmath>
#include "robot_controller/RobotDriver.hpp"

const float TO_METER = 0.001;
const float TO_MM = 1000.;

const float TO_DEG = 180. / M_PI;
const float TO_RAD = M_PI / 180.;

RobotDriver::RobotDriver() {
  jointNames.push_back("robot_joint1");
  jointNames.push_back("robot_joint2");
  jointNames.push_back("robot_joint3");
  jointNames.push_back("robot_joint4");
  jointNames.push_back("robot_joint5");
  jointNames.push_back("robot_joint6");

  // for(int i = 0; i < jointNames.size(); i++){
  //   currentJointPosition.push_back(0.0);
  // }
}

RobotDriver::RobotDriver(std::string ip_address, int gripper_port) : gripperPort(gripper_port){
  jointNames.push_back("robot_joint1");
  jointNames.push_back("robot_joint2");
  jointNames.push_back("robot_joint3");
  jointNames.push_back("robot_joint4");
  jointNames.push_back("robot_joint5");
  jointNames.push_back("robot_joint6");

  for(int i = 0; i < jointNames.size(); i++){
    currentJointPosition.push_back(0.0);
  }

  connect(ip_address);
}

RobotDriver::~RobotDriver() {
 //   TO DO: Close connection;
}

bool RobotDriver::connect(std::string ip_address) {
  ROS_INFO("Connection to %s", ip_address.c_str());

 //   TO DO: Start TCP connection;

  // if (  ) {
  //   ROS_INFO("Connection to %s established", ip_address.c_str());
  //   return true;
  // }
  // else {
  //   ROS_ERROR("Connection error");
  //   return false;
  // }
}

bool RobotDriver::read(sensor_msgs::JointState &msg) {
  // Read joint states from robot using Robot Control Library
  ROS_INFO("read");
  if (getJointStates()) {
    // Return current joint position
    msg.header.stamp = ros::Time::now();
    msg.name = jointNames;
    msg.position = currentJointPosition;
    return true;
  }

  return false;
}

bool RobotDriver::write(sensor_msgs::JointState msg) {
  ROS_INFO("write");
  if (msg.position.size() != jointNames.size()) {
    ROS_ERROR("Joint position msg size %zu error", msg.position.size());
    return false;
  }
  
  return controlJointPosition(msg.position, 0.01, false);
}

bool RobotDriver::controlJointPosition(const std::vector<double>& js_cmd, float tol, bool accurate)
{
  if (js_cmd.size() == jointNames.size()) {
    std::vector<double> js_temp(currentJointPosition.begin(), currentJointPosition.end());
    if (!setJointStates(js_cmd, accurate)) {
      return false;
    }
    int duration = 10;
    while (ros::ok() && duration) {
      if (checkAllClose(js_cmd, currentJointPosition, tol)) {
        return true;
      }
      ros::Duration(0.1).sleep();
      if (checkAllClose(js_temp, currentJointPosition, 0.001)) {
        if (!setJointStates(js_cmd, accurate)) {
          return false;
        }
        duration--;
      } else {
        for (int i = 0; i < jointNames.size(); ++i) {
          js_temp[i] = currentJointPosition[i];
        }
      }
    }
    ROS_WARN("Execute joint states service call failed");
    //
    return false;
  } else {
    ROS_ERROR("Joint position goal size %zu error", js_cmd.size());
    return false;
  }
}

bool RobotDriver::checkAllClose(std::vector<double> a, std::vector<double> b, float tol) {
  if (tol == 0) tol = 0.01;
  if (a.size() != b.size()) {
    ROS_ERROR("Size mismatch error, %zu, %zu", a.size(), b.size());
    return true;
  }
  for (int i = 0; i < a.size(); ++i) {
    float v_a = a[i];
    float v_b = b[i];
    if (fabs(v_a - v_b) > tol) {
      //ROS_WARN_STREAM("Mismatch " << i << " " << v_a << " " << v_b);
      return false;
    }
  }
  return true;
}

bool RobotDriver::getJointStates() {
  //   TO DO: check connection state at first;

  // if ( ) {
  //   ROS_ERROR_THROTTLE(11, "No connection to robot (print every 11 seconds)");
  //   return false;
  // }
  // std::vector<float> curr_js_temp;
  // int axes_num = jointNames.size();
  // float jointvalue[axes_num];
  // memset(&jointvalue, 0, sizeof(jointvalue));



  //   TO DO: get robot's joint states;
  //   Get jointvalue[] by robot control lib;

  // if ( ) {
  //   for (int nAxis = 0; nAxis < jointNames.size(); nAxis++) {
  //     curr_js_temp.push_back(jointvalue[nAxis] * TO_RAD);
  //   }
  //   for (int i = 0; i < jointNames.size(); i++) {
  //     currentJointPosition[i] = curr_js_temp[i];
  //   }
  //   return true;
  // }
  // else {
  //   ROS_ERROR("Read joint states error");
  //   return false;
  // }
  return true;
}

bool RobotDriver::setJointStates(std::vector<double> js_cmd, bool accurate) {
  ROS_INFO("setJointStates");
  //   TO DO: check connection state at first;

  // if () return false;
  // int axes_num = jointNames.size();
  // float jointangle[axes_num];
  // memset(&jointangle, 0, sizeof(jointangle));
  // if (js_cmd.size() != axes_num) {
  //   ROS_ERROR("Command size error: %zu", js_cmd.size());
  //   return false;
  // }



  //   TO DO: set robot's joint angles;
  //   Set jointangle[] by robot control lib;

  // for (int i = 0; i < jointNames.size(); i++) {
  //   ROS_INFO("js_cmd[%d] %f", i, js_cmd[i]);
  //   jointangle[i] = static_cast<float>(js_cmd[i] * TO_DEG);
  // }
  // if () {
  //   return true;
  // } else {
  //   ROS_ERROR("Write joint states error: %d", nErr);
  //   return false;
  // }
  return true;

}


bool RobotDriver::controlGripper(bool enable)
{
  //   TO DO: gripper switch;
  // bool bValue[1];
  // memset(bValue, 0, sizeof(bValue));
  // bValue[0] = enable;

  //   Change gripper state by robot control lib;
  // if () {
  //   ROS_DEBUG("Gripper command write successfully");
  //   return true;
  // }
  // else {
  //   ROS_ERROR("Gripper command write error");
  //   return false;
  // }
  return true;
}



