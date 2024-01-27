#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define FindGraspableObjects grasping_msgs::action::FindGraspableObjects
#define ClientGoalHandle rclcpp_action::ClientGoalHandle<FindGraspableObjects>

class GetPoseClient : public rclcpp::Node {
private:
  // member variables
  double px;
  double py;
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
        RCLCPP_INFO(this->get_logger(), "X : %f, Y : %f", px, py);
        if (px >= 0.0) {
          is_approachable = true;
          RCLCPP_INFO(this->get_logger(), "is_approachable : `true`");
        } else {
          RCLCPP_WARN(this->get_logger(), "is_approachable : `false`");
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
    return this->px;
  }

  double get_position_y() {
    // return the detected object position y
    return this->py;
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<GetPoseClient> node = std::make_shared<GetPoseClient>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}
