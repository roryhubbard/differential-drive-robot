#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "ddbot_msgs/action/track_trajectory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ddbot_controller
{
class ControllerActionClient : public rclcpp::Node
{
public:
  using TrackTrajectory = ddbot_msgs::action::TrackTrajectory;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<TrackTrajectory>;

  explicit ControllerActionClient(const rclcpp::NodeOptions & options)
  : Node("controller_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<TrackTrajectory>(
      this,
      "track_trajectory");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ControllerActionClient::send_goal, this));
  }

  //void send_goal(TrackTrajectory::Goal goal_msg)
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto pose1 = geometry_msgs::msg::Pose();
    pose1.position.x = 1.0;
    pose1.position.y = 1.0;
    pose1.position.z = 1.0;
    auto tps1 = ddbot_msgs::msg::TrajectoryPointStamped();
    tps1.trajectory_point.pose = pose1;

    auto pose2 = geometry_msgs::msg::Pose();
    pose2.position.x = 2.0;
    pose2.position.y = 2.0;
    pose2.position.z = 2.0;
    auto tps2 = ddbot_msgs::msg::TrajectoryPointStamped();
    tps2.trajectory_point.pose = pose2;

    std::vector<ddbot_msgs::msg::TrajectoryPointStamped> trajectory;
    trajectory.push_back(tps1);
    trajectory.push_back(tps2);

    auto goal_msg = TrackTrajectory::Goal();
    goal_msg.trajectory = trajectory;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<TrackTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ControllerActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ControllerActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ControllerActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<TrackTrajectory>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleFollowPath::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFollowPath::SharedPtr,
    const std::shared_ptr<const TrackTrajectory::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Trajectory points remaining: ";
    ss << feedback->trajectory_points_remaining << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFollowPath::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    ss << result.result->success<< " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class ControllerActionClient

}  // namespace ddbot_controller

RCLCPP_COMPONENTS_REGISTER_NODE(ddbot_controller::ControllerActionClient)
