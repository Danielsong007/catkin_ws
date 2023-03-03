//
// Created by andylee on 2021/1/26.
//
#include <cmath>
#include "robot_controller/RobotFakeDriver.hpp"

int i = 0;

RobotFakeDriver::RobotFakeDriver() : RobotDriver() {
  idx = 0.0;
  ROS_INFO("jointNames size = %d\n", jointNames.size());
//   for(int i = 0; i < jointNames.size(); i++) {
//       currentJointPosition.push_back(0);
//   }
    currentJointPosition = {0.8004, -1.8953, 2.1806, 1.2872, -1.5733, -0.0037};
}

RobotFakeDriver::~RobotFakeDriver() {

}

bool RobotFakeDriver::connect(std::string ip_address) {
    ROS_INFO("connect\n");
    return true;
}

bool RobotFakeDriver::read(sensor_msgs::JointState &msg) {
    //ROS_INFO("read\n");
    idx += 0.1;
    if (getJointStates(currentJointPosition)) {
        // Return current joint position
        msg.header.stamp = ros::Time::now();
        msg.name = jointNames;
        std::vector<double> q;
        q.reserve(jointNames.size());
        for (int i = 0; i < jointNames.size(); i++) {
            q.push_back(currentJointPosition[i]);
        }
        msg.position = q;
        return true;
    }

    return false;
}

bool RobotFakeDriver::write(sensor_msgs::JointState msg) {
    if (msg.position.size() != jointNames.size()) {
        ROS_ERROR("Joint position msg size %zu error", msg.position.size());
        return false;
    }
    // ROS_INFO("write points %f %f %f %f %f %f", msg.position[0],msg.position[1],
    //     msg.position[2],msg.position[3],msg.position[4],msg.position[5]);
    return setJointStates(msg.position, true);
}

bool RobotFakeDriver::getJointStates(std::vector<double> &currentJointPosition) {
    //ROS_INFO("getJointStates\n");
    return true;
}

bool RobotFakeDriver::setJointStates(std::vector<double> js_cmd, bool accurate) {
    bool update = false;
    for (int i = 0; i < jointNames.size(); i++) {
        if(js_cmd[i] != currentJointPosition[i]){
            update = true;
            break;
        }
    }
    if(update){
        ROS_INFO("%d setJointStates %f %f %f %f %f %f\n", i, js_cmd[0], js_cmd[1],js_cmd[2],js_cmd[3],js_cmd[4],js_cmd[5]);
        currentJointPosition = js_cmd;
        i++;
    }
    return true;
}

bool RobotFakeDriver::controlGripper(bool enable)
{
    ROS_INFO("controlGripper\n");
    return true;
}


bool RobotFakeDriver::executeTrajectory(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg){
    for (int i = 0; i < msg->goal.trajectory.points.size(); i++) {
        //ROS_INFO("executeTrajectory %f %f %f %f %f %f", msg->goal.trajectory.points[i].positions[0], msg->goal.trajectory.points[i].positions[1],
        // msg->goal.trajectory.points[i].positions[2],msg->goal.trajectory.points[i].positions[3],
        // msg->goal.trajectory.points[i].positions[4],msg->goal.trajectory.points[i].positions[5]);
    }

    ros::Rate rate(10);
    for (int i = 0; i < msg->goal.trajectory.points.size(); i++) {
        setJointStates(msg->goal.trajectory.points[i].positions, true);
        rate.sleep();
    }

    return true;
}

