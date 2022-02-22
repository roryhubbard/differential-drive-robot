#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robotino_msgs/action/track_trajectory.hpp"
#include "robotino_msgs/srv/get_trajectory.hpp"


using namespace std::chrono_literals;
using TrackTrajectory = robotino_msgs::action::TrackTrajectory;
using GoalHandleTrackTrajectory = rclcpp_action::ClientGoalHandle<TrackTrajectory>;
rclcpp::Node::SharedPtr node = nullptr;


void goal_response_callback(
    std::shared_future<GoalHandleTrackTrajectory::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
  }
}


void feedback_callback(
    GoalHandleTrackTrajectory::SharedPtr,
    const std::shared_ptr<const TrackTrajectory::Feedback> feedback)
{
  RCLCPP_INFO(
      node->get_logger(),
      "Trajectory points remaining: %d",
      feedback->trajectory_points_remaining);
}


void result_callback(
    const GoalHandleTrackTrajectory::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      return;
  }
  RCLCPP_INFO(
      node->get_logger(),
      "Result received: success = %d",
      result.result->success);
  rclcpp::shutdown();
}


std::shared_future<GoalHandleTrackTrajectory::SharedPtr> send_goal(
    rclcpp_action::Client<TrackTrajectory>::SharedPtr control_client,
    TrackTrajectory::Goal &goal_msg)
{
  if (!control_client->wait_for_action_server()) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(node->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<TrackTrajectory>::SendGoalOptions();
  send_goal_options.goal_response_callback = goal_response_callback;
  send_goal_options.feedback_callback = feedback_callback;
  send_goal_options.result_callback = result_callback;
  return control_client->async_send_goal(goal_msg, send_goal_options);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("autonomy");

  rclcpp::Client<robotino_msgs::srv::GetTrajectory>::SharedPtr planning_client =
    node->create_client<robotino_msgs::srv::GetTrajectory>("get_trajectory");

  rclcpp_action::Client<TrackTrajectory>::SharedPtr control_client =
    rclcpp_action::create_client<TrackTrajectory>(node, "track_trajectory");

  auto start = nav_msgs::msg::Odometry();
  start.pose.pose.position.x = -2.;
  start.pose.pose.position.y = -2.;
  auto goal = nav_msgs::msg::Odometry();
  goal.pose.pose.position.x = 2.;
  goal.pose.pose.position.y = 2.;

  auto request = std::make_shared<robotino_msgs::srv::GetTrajectory::Request>();
  request->start = start;
  request->goal = goal;

  while (!planning_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
          "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Wait for the result
  auto planning_result_future = planning_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, planning_result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call to get_trajectory failed");
    return 1;
  }

  auto goal_msg = TrackTrajectory::Goal();
  goal_msg.trajectory = planning_result_future.get()->trajectory;

  auto goal_handle_future = send_goal(control_client, goal_msg);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed on track_trajectory");
    return 1;
  }

  GoalHandleTrackTrajectory::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "goal was rejected by control action server");
    return 1;
  }

  // Wait for the server to be done with the goal
  auto result_future = control_client->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed on track_trajectory");
    return 1;
  }

  control_client.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}

