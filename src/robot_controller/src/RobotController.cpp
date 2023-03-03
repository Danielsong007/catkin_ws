#include "robot_controller/RobotController.hpp"

const std::string run_gripper_srv_id = "/gripper/run";


RobotController::RobotController(ros::NodeHandle &nh, ros::NodeHandle &pnh, RobotDriver *robot_driver) :
  nh_(nh), pnh_(pnh), robot_driver_(robot_driver) {

    thread_ptr = NULL;

    // Initialize gripper service
    gripperService = nh_.advertiseService(run_gripper_srv_id, &RobotController::gripperSrvCb, this);
    ROS_INFO_STREAM("Advertising service " << run_gripper_srv_id);

}

bool RobotController::gripperSrvCb(std_srvs::SetBool::Request &req,
                          std_srvs::SetBool::Response &res)
{
    bool result;
    if (req.data) {
      result = robot_driver_->controlGripper(true);
    } else {
      result = robot_driver_->controlGripper(false);
    }
    res.success = result;
    return true;
}


