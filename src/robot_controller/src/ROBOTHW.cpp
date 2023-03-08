#include "robot_controller/ROBOTHW.hpp"

#include <utility>


ROBOTHW::ROBOTHW(RobotDriver *robot_driver, bool use_arm_controller) : robot_driver_(robot_driver), use_arm_controller_(use_arm_controller)
{
    jointNames.push_back("robot_joint1");
    jointNames.push_back("robot_joint2");
    jointNames.push_back("robot_joint3");
    jointNames.push_back("robot_joint4");
    jointNames.push_back("robot_joint5");
    jointNames.push_back("robot_joint6");

    // Resize vectors
    currentJointPosition.resize(jointNames.size());
    currentJointVelocity.resize(jointNames.size());
    currentJointEffort.resize(jointNames.size());
    jointPositionCommand.resize(jointNames.size());

    std::fill(currentJointPosition.begin(), currentJointPosition.end(), 0);
    std::fill(currentJointVelocity.begin(), currentJointVelocity.end(), 0);
    std::fill(currentJointEffort.begin(),   currentJointEffort.end(), 0);

    sensor_msgs::JointState joints_msg;
    bool result = robot_driver_->read(joints_msg);
    if(result) {
      ROS_INFO("Got initial states");
      jointPositionCommand = joints_msg.position;
    } else {
      float initials[] = {0.0,0.0,0.0,0.0,0.0,0.0};
      jointPositionCommand.assign(initials,initials+6);
    }

    //std::fill(jointPositionCommand.begin(), jointPositionCommand.end(), 0);
    ROS_INFO_STREAM("Initialized buffers");

    // Initialize Controller
    for (int i = 0; i < jointNames.size(); ++i) {
      // Create joint state interface
      JointStateHandle jointStateHandle(jointNames[i], &currentJointPosition[i],
                                        &currentJointVelocity[i], &currentJointEffort[i]);
      jointStateInterface_.registerHandle(jointStateHandle);

      // Create position joint interface
      JointHandle jointPositionHandle(jointStateHandle, &jointPositionCommand[i]);
      positionJointInterface_.registerHandle(jointPositionHandle);
    }

    registerInterface(&jointStateInterface_);
    registerInterface(&positionJointInterface_);
    ROS_INFO_STREAM("Initialized Robot joint states interfaces");
}

void ROBOTHW::read()
{
    sensor_msgs::JointState msg;
    robot_driver_->read(msg);
    std::unique_lock<std::timed_mutex> lock(jointStatesMutex, std::chrono::seconds(2));
    if (!lock) {
      ROS_ERROR("Couldn't acquire mutex, couldn't set joint state");
      return;
    }

    for (int i = 0; i < jointNames.size(); i++) {
        //ROS_INFO_STREAM("Getting state of joint "<< i);
        currentJointPosition.at(i) = msg.position.at(i);
        // currentJointVelocity.at(i) = msg.velocity.at(i);
        // currentJointEffort.at(i) = msg.effort.at(i);
        //ROS_INFO_STREAM("Got state of joint "<< i);
    }
}

void ROBOTHW::write()
{
    if(!use_arm_controller_){
      return;
    }

    sensor_msgs::JointState msg;
    msg.position = std::vector<double>();

    for (int i = 0; i < jointNames.size(); i++) {
        msg.position.push_back(jointPositionCommand[i]);
    }
    
    robot_driver_->write(msg);
}




