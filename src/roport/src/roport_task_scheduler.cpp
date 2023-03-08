#include <roport/bt_service_node.h>
#include <roport/bt_action_node.h>
#include <roport/bt_generic_types.h>
#include <roport/rosout_logger.h>

// ROS
#include <ros/ros.h>

// Services (customized)
#include <roport/ExecuteGroupJointStates.h>
#include <roport/ExecuteGroupPose.h>
#include <roport/ExecuteGroupSpeed.h>
#include <roport/ExecuteGroupManyPoses.h>
#include <roport/ExecuteGroupNamedStates.h>
#include <roport/ExecuteGroupShift.h>
#include <roport/TypeInPose.h>
#include <roport/ConnectWaypoints.h>

#include <roport/ExecuteAddBox.h>
#include <roport/ExecuteAddPlane.h>
#include <roport/ExecuteAttachBox.h>
#include <roport/ExecuteDetachObject.h>
#include <roport/ExecuteRemoveObject.h>

#include <roport/ExecutePlanning.h>
#include <roport/ExecuteSuction.h>

#include <roport/SenseObjectPose.h>
#include <roport/StoreDetectedInfo.h>
#include <roport/FetchDetectedInfo.h>
#include <roport/SetInitFlag.h>
#include <roport/SetTriggerPhoto.h>

// Actions (customized)


using namespace BT;

class ExecuteGroupAngularJointStates : public RosServiceNode<roport::ExecuteGroupJointStates>
{
public:
  ExecuteGroupAngularJointStates(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteGroupJointStates>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<DoubleArray>("goal"),
        InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    DoubleArray goal{};
    getInput<DoubleArray>("goal", goal);
    request.goal = goal.degToROS();
    getInput<double>("tolerance", request.tolerance);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupLinearJointStates : public RosServiceNode<roport::ExecuteGroupJointStates>
{
public:
  ExecuteGroupLinearJointStates(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteGroupJointStates>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<DoubleArray>("goal"),
        InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    DoubleArray goal{};
    getInput<DoubleArray>("goal", goal);
    request.goal = goal.plainToROS();
    getInput<double>("tolerance", request.tolerance);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupNamedStates : public RosServiceNode<roport::ExecuteGroupNamedStates>
{
public:
  ExecuteGroupNamedStates(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteGroupNamedStates>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<std::string>("state_name"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("state_name", request.state_name);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupSpeed : public RosServiceNode<roport::ExecuteGroupSpeed>
{
public:
  ExecuteGroupSpeed(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteGroupSpeed>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<double>("value"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<double>("value", request.value);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupPose : public RosServiceNode<roport::ExecuteGroupPose>
{
public:
  ExecuteGroupPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteGroupPose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<int>("goal_type"),
        InputPort<Pose>("goal"),
        InputPort<double>("tolerance"),
        InputPort<std::string>("constraint"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);

    int goal_type;
    getInput<int>("goal_type", goal_type);
    request.goal_type = goal_type;

    Pose goal{};
    getInput<Pose>("goal", goal);
    request.goal = goal.toROS();

    getInput<double>("tolerance", request.tolerance);
    getInput<std::string>("constraint", request.constraint);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupManyPoses : public RosServiceNode<roport::ExecuteGroupManyPoses>
{
public:
  ExecuteGroupManyPoses(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteGroupManyPoses>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<int>("goal_type"),
        InputPort<PoseArray>("goals"),
        InputPort<double>("tolerance"),
        InputPort<double>("eef_step"),
        InputPort<std::string>("constraint"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);

    int goal_type;
    getInput<int>("goal_type", goal_type);
    request.goal_type = goal_type;

    PoseArray goals{};
    getInput<PoseArray>("goals", goals);
    request.goals = goals.toROS();

    getInput<double>("tolerance", request.tolerance);
    getInput<double>("eef_step", request.eef_step);
    getInput<std::string>("constraint", request.constraint);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupShift : public RosServiceNode<roport::ExecuteGroupShift>
{
public:
  ExecuteGroupShift(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteGroupShift>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<int>("is_absolute"),
        InputPort<std::string>("axis"),
        InputPort<double>("goal"),
        InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);

    int is_absolute;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = bool(is_absolute);

    getInput<std::string>("axis", request.axis);
    getInput<double>("goal", request.goal);

    getInput<double>("tolerance", request.tolerance);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecutePlanning : public RosServiceNode<roport::ExecutePlanning>
{
public:
  ExecutePlanning(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecutePlanning>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<Pose>("pose"),
        InputPort<std::string>("category"),
        OutputPort<DoubleArray>("pre_middle_pose"),
        OutputPort<Pose>("pre_pick_pose"),
        OutputPort<Pose>("pick_pose"),
        OutputPort<Pose>("post_pick_pose"),
        OutputPort<Pose>("post_pick_pose_edge"),
        OutputPort<DoubleArray>("post_middle_pose"),
        OutputPort<DoubleArray>("place_pose"),
    };
  }

  void onSendRequest(RequestType &request) override {
    Pose pose{};
    getInput<Pose>("pose", pose);
    request.pose = pose.toROS();
    getInput<std::string>("category", request.category);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      DoubleArray pre_middle_pose{};
      pre_middle_pose.fromROS(response.pre_middle_pose);
      setOutput("pre_middle_pose", pre_middle_pose);

      Pose pre_pick_pose{};
      pre_pick_pose.fromROS(response.pre_pick_pose);
      setOutput("pre_pick_pose", pre_pick_pose);

      Pose pick_pose{};
      pick_pose.fromROS(response.pick_pose);
      setOutput("pick_pose", pick_pose);

      Pose post_pick_pose{};
      post_pick_pose.fromROS(response.post_pick_pose);
      setOutput("post_pick_pose", post_pick_pose);

      Pose post_pick_pose_edge{};
      post_pick_pose_edge.fromROS(response.post_pick_pose_edge);
      setOutput("post_pick_pose_edge", post_pick_pose_edge);

      DoubleArray post_middle_pose{};
      post_middle_pose.fromROS(response.post_middle_pose);
      setOutput("post_middle_pose", post_middle_pose);

      DoubleArray place_pose{};
      place_pose.fromROS(response.place_pose);
      setOutput("place_pose", place_pose);
      
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class TypeInPose : public RosServiceNode<roport::TypeInPose>
{
public:
  TypeInPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::TypeInPose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Pose>("type_in_pose"),
        OutputPort<Pose>("pose_on_blackboard"),
    };
  }

  void onSendRequest(RequestType &request) override {
    Pose type_in_pose{};
    getInput<Pose>("type_in_pose", type_in_pose);
    request.type_in_pose = type_in_pose.toROS();
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      Pose pose_on_blackboard{};
      pose_on_blackboard.fromROS(response.pose_on_blackboard);
      setOutput("pose_on_blackboard", pose_on_blackboard);
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ConnectWaypoints : public RosServiceNode<roport::ConnectWaypoints>
{
public:
  ConnectWaypoints(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ConnectWaypoints>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Pose>("waypoint1"),
        InputPort<Pose>("waypoint2"),
        InputPort<Pose>("waypoint3"),
        InputPort<Pose>("waypoint4"),
        InputPort<Pose>("waypoint5"),
        OutputPort<PoseArray>("connected_waypoints"),
    };
  }

  void onSendRequest(RequestType &request) override {
    Pose waypoint1{};
    getInput<Pose>("waypoint1", waypoint1);
    request.waypoint1 = waypoint1.toROS();
    Pose waypoint2{};
    getInput<Pose>("waypoint2", waypoint2);
    request.waypoint2 = waypoint2.toROS();
    Pose waypoint3{};
    getInput<Pose>("waypoint3", waypoint3);
    request.waypoint3 = waypoint3.toROS();
    Pose waypoint4{};
    getInput<Pose>("waypoint4", waypoint4);
    request.waypoint4 = waypoint4.toROS();
    Pose waypoint5{};
    getInput<Pose>("waypoint5", waypoint5);
    request.waypoint5 = waypoint5.toROS();
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      PoseArray connected_waypoints{};
      connected_waypoints.fromROS(response.connected_waypoints);
      setOutput("connected_waypoints", connected_waypoints);
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteSuction : public RosServiceNode<roport::ExecuteSuction>
{
public:
  ExecuteSuction(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteSuction>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<int>("enable"),
    };
  }

  void onSendRequest(RequestType &request) override {
    int enable;
    getInput<int>("enable", enable);
    request.enable = bool(enable);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteAddPlane : public RosServiceNode<roport::ExecuteAddPlane>
{
public:
  ExecuteAddPlane(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteAddPlane>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<std::string>("plane_name"),
        InputPort<Pose>("plane_pose"),
        InputPort<Point>("plane_normal"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("plane_name", request.plane_name);
    Pose plane_pose{};
    getInput<Pose>("plane_pose", plane_pose);
    request.plane_pose = plane_pose.toROS();
    Point plane_normal{};
    getInput<Point>("plane_normal", plane_normal);
    request.plane_normal = plane_normal.toROS();
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteAddBox : public RosServiceNode<roport::ExecuteAddBox>
{
public:
  ExecuteAddBox(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteAddBox>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<std::string>("box_name"),
        InputPort<Pose>("box_pose"),
        InputPort<Point>("box_size"),
        InputPort<int>("is_absolute"),
        InputPort<int>("auto_subfix"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("box_name", request.box_name);
    Pose box_pose{};
    getInput<Pose>("box_pose", box_pose);
    request.box_pose = box_pose.toROS();
    Point box_size{};
    getInput<Point>("box_size", box_size);
    request.box_size = box_size.toROS();
    int is_absolute;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = bool(is_absolute);
    int auto_subfix;
    getInput<int>("auto_subfix", auto_subfix);
    request.auto_subfix = bool(auto_subfix);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteAttachBox : public RosServiceNode<roport::ExecuteAttachBox>
{
public:
  ExecuteAttachBox(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteAttachBox>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<std::string>("eef_group_name"),
        InputPort<std::string>("box_name"),
        InputPort<Pose>("box_pose"),
        InputPort<Point>("box_size"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("eef_group_name", request.eef_group_name);
    getInput<std::string>("box_name", request.box_name);
    Pose box_pose{};
    getInput<Pose>("box_pose", box_pose);
    request.box_pose = box_pose.toROS();
    Point box_size{};
    getInput<Point>("box_size", box_size);
    request.box_size = box_size.toROS();
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteDetachObject : public RosServiceNode<roport::ExecuteDetachObject>
{
public:
  ExecuteDetachObject(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteDetachObject>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<std::string>("obj_name"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("obj_name", request.obj_name);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteRemoveObject : public RosServiceNode<roport::ExecuteRemoveObject>
{
public:
  ExecuteRemoveObject(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::ExecuteRemoveObject>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("obj_name"),
        InputPort<int>("is_exact"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("obj_name", request.obj_name);
    int is_exact;
    getInput<int>("is_exact", is_exact);
    request.is_exact = bool(is_exact);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class SenseObjectPose : public RosServiceNode<roport::SenseObjectPose>
{
public:
  SenseObjectPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::SenseObjectPose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        OutputPort<Pose>("pose"),
        OutputPort<std::string>("category"),
    };
  }

  void onSendRequest(RequestType &request) override {
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      Pose pose{};
      pose.fromROS(response.pose);
      setOutput("pose", pose);
      std::string category = response.category;
      setOutput("category", category);
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class StoreDetectedInfo : public RosServiceNode<roport::StoreDetectedInfo>
{
public:
  StoreDetectedInfo(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::StoreDetectedInfo>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        InputPort<Pose>("pose"),
        InputPort<std::string>("category"),
    };
  }

  void onSendRequest(RequestType &request) override {
    Pose pose{};
    getInput<Pose>("pose", pose);
    request.pose = pose.toROS();
    getInput<std::string>("category", request.category);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class FetchDetectedInfo : public RosServiceNode<roport::FetchDetectedInfo>
{
public:
  FetchDetectedInfo(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::FetchDetectedInfo>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
        OutputPort<Pose>("pose"),
        OutputPort<std::string>("category"),
    };
  }

  void onSendRequest(RequestType &request) override {
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      Pose pose{};
      pose.fromROS(response.pose);
      setOutput("pose", pose);
      std::string category = response.category;
      setOutput("category", category);
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class SetInitFlag : public RosServiceNode<roport::SetInitFlag>
{
public:
  SetInitFlag(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::SetInitFlag>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
    };
  }

  void onSendRequest(RequestType &request) override {
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class SetTriggerPhoto : public RosServiceNode<roport::SetTriggerPhoto>
{
public:
  SetTriggerPhoto(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
      RosServiceNode<roport::SetTriggerPhoto>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
        InputPort<Header>("header"),
    };
  }

  void onSendRequest(RequestType &request) override {
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roport_task_scheduler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string tree_file;
  pnh.getParam("tree_file", tree_file);
  if (tree_file.empty()) {
    ROS_ERROR("RoPort: No valid tree file.");
    return -1;
  }

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  RegisterRosService<ExecuteGroupAngularJointStates>(factory, "ExecuteGroupAngularJointStates", nh);
  RegisterRosService<ExecuteGroupLinearJointStates>(factory, "ExecuteGroupLinearJointStates", nh);

  RegisterRosService<ExecuteGroupNamedStates>(factory, "ExecuteGroupNamedStates", nh);
  RegisterRosService<ExecuteGroupShift>(factory, "ExecuteGroupShift", nh);
  RegisterRosService<ExecuteGroupPose>(factory, "ExecuteGroupPose", nh);
  RegisterRosService<ExecuteGroupSpeed>(factory, "ExecuteGroupSpeed", nh);
  RegisterRosService<ExecuteGroupManyPoses>(factory, "ExecuteGroupManyPoses", nh);
  RegisterRosService<TypeInPose>(factory, "TypeInPose", nh);
  RegisterRosService<ConnectWaypoints>(factory, "ConnectWaypoints", nh);

  RegisterRosService<ExecuteAddBox>(factory, "ExecuteAddBox", nh);
  RegisterRosService<ExecuteAddPlane>(factory, "ExecuteAddPlane", nh);
  RegisterRosService<ExecuteAttachBox>(factory, "ExecuteAttachBox", nh);
  RegisterRosService<ExecuteDetachObject>(factory, "ExecuteDetachObject", nh);
  RegisterRosService<ExecuteRemoveObject>(factory, "ExecuteRemoveObject", nh);

  RegisterRosService<ExecutePlanning>(factory, "ExecutePlanning", nh);
  RegisterRosService<ExecuteSuction>(factory, "ExecuteSuction", nh);
  RegisterRosService<SenseObjectPose>(factory, "SenseObjectPose", nh);
  RegisterRosService<StoreDetectedInfo>(factory, "StoreDetectedInfo", nh);
  RegisterRosService<FetchDetectedInfo>(factory, "FetchDetectedInfo", nh);
  RegisterRosService<SetInitFlag>(factory, "SetInitFlag", nh);
  RegisterRosService<SetTriggerPhoto>(factory, "SetTriggerPhoto", nh);

  auto tree = factory.createTreeFromFile(tree_file);

  RosoutLogger logger(tree.rootNode());
  printTreeRecursively(tree.rootNode());

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  while(ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
    ros::spinOnce();
    status = tree.tickRoot();
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}