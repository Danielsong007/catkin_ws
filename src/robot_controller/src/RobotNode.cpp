#include <ros/ros.h>

#include "robot_controller/ROBOTHW.hpp"
#include "robot_controller/RobotController.hpp"
#include "robot_controller/RobotFakeDriver.hpp"



int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    //set ip address of the robot
    std::string ip_address = "192.168.1.10";

    //set use_arm_controller or not
    bool use_arm_controller = true;

    //set use_fake_driver or not
    bool use_fake_driver = true;
    pnh.getParam("use_fake_driver", use_fake_driver);

    //Create driver
    RobotDriver *driver = NULL;
    int gripper_port = 1;
    if(use_fake_driver) {
        driver = new RobotFakeDriver();
    } else {
        driver = new RobotDriver(ip_address, gripper_port);
    }

    //Create Controller for handle gripper command
    RobotController controller(nh, pnh, driver);

    //Create ROBOTHW for handle joint states reading and joint command dispatching
    ROBOTHW robot(driver, use_arm_controller);

    //Create controller manager
    controller_manager::ControllerManager cm(&robot);
  
    // Set spin rate
    float dur = 0.01;
    ros::Rate rate(1.0 / ros::Duration(dur).toSec());
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok())
    {
        robot.read();
        cm.update(ros::Time::now(), ros::Duration(dur));
        robot.write();
        rate.sleep();
    }

    spinner.stop();

    delete driver;
    driver = NULL;

    return 0;
}


