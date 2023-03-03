#pragma once

#include <ros/ros.h>
#include "robot_controller/RobotDriver.hpp"
#include <thread>

class RobotController {
public:
    RobotController(ros::NodeHandle &nh, ros::NodeHandle &pnh, RobotDriver *robot_driver);

    bool gripperSrvCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
    RobotDriver *robot_driver_;

    ros::NodeHandle &nh_;
    ros::NodeHandle &pnh_;

    std::mutex mutex_;

    ros::ServiceServer gripperService;

    std::unique_ptr<std::thread> thread_ptr;

    bool use_arm_controller = true;
};