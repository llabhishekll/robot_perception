#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#define FindGraspableObjects grasping_msgs::action::FindGraspableObjects
#define ClientGoalHandle rclcpp_action::ClientGoalHandle<FindGraspableObjects>
#define SUCCESS moveit::core::MoveItErrorCode::SUCCESS

static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_TOOL = "gripper";

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_node");

class GetPoseClient : public rclcpp::Node {
private:
  // member variables
  double px;
  double py;
  double dx;
  double dy;
  bool goal_done;
  bool is_approachable;

  // ros objects
  rclcpp_action::Client<FindGraspableObjects>::SharedPtr action_client;
  ClientGoalHandle::SharedPtr goal_handle;
  rclcpp::TimerBase::SharedPtr action_timer;

  // member function
  void send_goal() {
    int counter = 0;
    while (
        !this->action_client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for server : %d", counter);
      counter++;
    }

    auto goal = FindGraspableObjects::Goal();
    goal.plan_grasps = false;

    auto options =
        rclcpp_action::Client<FindGraspableObjects>::SendGoalOptions();
    options.goal_response_callback = std::bind(
        &GetPoseClient::goal_response_callback, this, std::placeholders::_1);
    options.result_callback = std::bind(&GetPoseClient::goal_result_callback,
                                        this, std::placeholders::_1);
    // send goal to server
    this->action_client->async_send_goal(goal, options);
    this->action_timer->cancel();
  }

  void goal_response_callback(const ClientGoalHandle::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "/find_objects rejected the goal");
    } else {
      this->goal_handle = goal_handle;
      RCLCPP_INFO(this->get_logger(), "/find_objects accepted the goal");
    }
  }

  void goal_result_callback(const ClientGoalHandle::WrappedResult &result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      goal_done = true;
      if (!result.result->objects.empty()) {
        px = result.result->objects[0].object.primitive_poses[0].position.x;
        py = result.result->objects[0].object.primitive_poses[0].position.y;
        dx = result.result->objects[0].object.primitives[0].dimensions[0];
        dy = result.result->objects[0].object.primitives[0].dimensions[1];
        RCLCPP_INFO(this->get_logger(), "X : %f, Y : %f", px, py);
        if (px >= 0.0) {
          is_approachable = true;
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Goal passed, but no object detected!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Goal failed!");
    }
  }

public:
  // constructor
  GetPoseClient()
      : Node("get_pose_client"), goal_done(false), is_approachable{false} {
    // ros objects
    this->action_client = rclcpp_action::create_client<FindGraspableObjects>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "/find_objects");
    this->action_timer =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&GetPoseClient::send_goal, this));
  }

  bool is_get_goal_pose_done() {
    // check if action serve has completed the calculation
    return this->goal_done;
  }

  double get_position_x() {
    // return the detected object position x
    double px_prime = (px / std::fabs(px)) * (std::fabs(px) + (dx / 2));
    return std::ceil(px_prime * 100.0) / 100.0;
  }

  double get_position_y() {
    // return the detected object position y
    double py_prime = (py / std::fabs(py)) * (std::fabs(py) + (dy / 2));
    return std::ceil(py_prime * 100.0) / 100.0;
  }

  bool is_object_approachable() {
    // check if object is detected and is approachable
    return this->is_approachable;
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  // define node options for moveit2 parameters overrides
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_and_place_node", options);

  // add node to executor and spin
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // move group interface
  auto move_group_arm =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node, PLANNING_GROUP_ARM);
  auto move_group_tool =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node, PLANNING_GROUP_TOOL);

  auto joints_arm =
      move_group_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  auto joints_tool = move_group_tool->getCurrentState()->getJointModelGroup(
      PLANNING_GROUP_TOOL);

  // get current state of system
  std::vector<double> positions_arm;
  std::vector<double> positions_tool;

  moveit::core::RobotStatePtr state_arm = move_group_arm->getCurrentState(10);
  moveit::core::RobotStatePtr state_tool = move_group_tool->getCurrentState(10);

  state_arm->copyJointGroupPositions(joints_arm, positions_arm);
  state_tool->copyJointGroupPositions(joints_tool, positions_tool);

  // define planning interface
  moveit::planning_interface::MoveGroupInterface::Plan plan_arm;
  moveit::planning_interface::MoveGroupInterface::Plan plan_tool;

  const double eef_step = 0.01;
  const double jump_threshold = 0.0;

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  // check if object is found and moveit can start robot movement
  auto client = std::make_shared<GetPoseClient>();
  int counter = 0;
  while (!client->is_get_goal_pose_done()) {
    RCLCPP_WARN(LOGGER, "Waiting for object detection : %d", counter);
    counter++;
    // halt this tread for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    rclcpp::spin_some(client);
  }

  double px = 0.0;
  double py = 0.0;

  // check if found object is approachable
  if (client->is_object_approachable()) {
    px = client->get_position_x();
    py = client->get_position_y();
    RCLCPP_INFO(LOGGER, "Target : X : %f, Y : %f", px, py);
  } else {
    RCLCPP_ERROR(LOGGER, "Shutting down as object is not approachable!");
    rclcpp::shutdown();
    return 0;
  }

  // free up some resources
  client.reset();

  // start robot movement
  RCLCPP_INFO(LOGGER, "Initilization done starting robot arm movement!");
  move_group_arm->setStartStateToCurrentState();
  move_group_tool->setStartStateToCurrentState();

  // position : home
  move_group_arm->setNamedTarget("home");
  if (move_group_arm->plan(plan_arm) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `home`");
    move_group_arm->execute(plan_arm);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `home`");
  }

  // position : grasp
  move_group_arm->setNamedTarget("grasp_left");
  if (move_group_arm->plan(plan_arm) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `grasp_right`");
    move_group_arm->execute(plan_arm);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `grasp_right`");
  }

  // tool : open
  move_group_tool->setNamedTarget("gripper_open");
  if (move_group_tool->plan(plan_tool) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `gripper_open`");
    move_group_tool->execute(plan_tool);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `gripper_open`");
  }

  // define approach target_pose message
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = px;
  target_pose.position.y = py;
  target_pose.position.z = 0.20;
  target_pose.orientation.x = -1.0;
  target_pose.orientation.y = 0.00;
  target_pose.orientation.z = 0.00;
  target_pose.orientation.w = 0.00;

  // define waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints_approach;
  waypoints_approach.push_back(target_pose);
  target_pose.position.z -= 0.02;
  waypoints_approach.push_back(target_pose);

  // compute trajectory
  auto f = move_group_arm->computeCartesianPath(
      waypoints_approach, eef_step, jump_threshold, trajectory_approach);

  // position : approach (using IK)
  move_group_tool->execute(trajectory_approach);
  RCLCPP_INFO(LOGGER, "Plan passed : `approach`");

  // tool : close
  move_group_tool->setNamedTarget("gripper_close");
  if (move_group_tool->plan(plan_tool) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `gripper_close`");
    move_group_tool->execute(plan_tool);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `gripper_close`");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  move_group_tool->setNamedTarget("gripper_hold");
  if (move_group_tool->plan(plan_tool) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `gripper_hold`");
    move_group_tool->execute(plan_tool);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `gripper_hold`");
  }

  // define waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints_retreat;
  target_pose.position.z += 0.02;
  waypoints_retreat.push_back(target_pose);

  // compute trajectory
  auto b = move_group_arm->computeCartesianPath(
      waypoints_retreat, eef_step, jump_threshold, trajectory_retreat);

  // position : retreat (using IK)
  move_group_tool->execute(trajectory_retreat);
  RCLCPP_INFO(LOGGER, "Plan passed : `retreat`");

  // position : rotate
  move_group_arm->setNamedTarget("rotate");
  if (move_group_arm->plan(plan_arm) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `rotate`");
    move_group_arm->execute(plan_arm);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `rotate`");
  }

  // tool : open
  move_group_tool->setNamedTarget("gripper_open");
  if (move_group_tool->plan(plan_tool) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `gripper_open`");
    move_group_tool->execute(plan_tool);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `gripper_open`");
  }

  // position : stand
  move_group_arm->setNamedTarget("stand");
  if (move_group_arm->plan(plan_arm) == SUCCESS) {
    RCLCPP_INFO(LOGGER, "Plan passed : `stand`");
    move_group_arm->execute(plan_arm);
  } else {
    RCLCPP_ERROR(LOGGER, "Plan failed : `stand`");
  }

  // suppress unused variables warnings
  (void)f;
  (void)b;

  // shutdown
  rclcpp::shutdown();
  return 0;
}