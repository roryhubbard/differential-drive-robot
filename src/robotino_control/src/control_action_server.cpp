#include <functional>
#include <memory>
#include <thread>
#include <tuple>
#include <cmath>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "robotino_msgs/action/track_trajectory.hpp"
#include "robotino_control/linear_quadratic_regulator.h"

namespace robotino_control
{

class ControllerActionServer : public rclcpp::Node
{
public:
  using TrackTrajectory = robotino_msgs::action::TrackTrajectory;
  using GoalHandleTrackTrajectory = rclcpp_action::ServerGoalHandle<TrackTrajectory>;

  explicit ControllerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("control_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<TrackTrajectory>(
      this,
      "track_trajectory",
      std::bind(&ControllerActionServer::handle_goal, this, _1, _2),
      std::bind(&ControllerActionServer::handle_cancel, this, _1),
      std::bind(&ControllerActionServer::handle_accepted, this, _1));

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/model/robotino/cmd_vel", default_qos);
  }

private:
  rclcpp_action::Server<TrackTrajectory>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TrackTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(),
                "Received track trajectory request with path size %lu", goal->trajectory.size());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTrackTrajectory> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTrackTrajectory> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ControllerActionServer::execute, this, _1), goal_handle}.detach();
  }

  Eigen::Vector3d express_in_rotated_frame(const Eigen::Ref<const Eigen::Vector3d>& x, double theta)
  {
    using namespace std;
    Eigen::Matrix3d R;
    R << cos(theta), sin(theta), 0.,
        -sin(theta), cos(theta), 0.,
                 0.,         0., 1.;
    return R * x;
  }

  Eigen::Vector3d msg_to_state_vector(const nav_msgs::msg::Odometry &odometry)
  {
    const auto x = odometry.pose.pose.position.x;
    const auto y = odometry.pose.pose.position.y;
    const tf2::Quaternion quatr(
      odometry.pose.pose.orientation.x,
      odometry.pose.pose.orientation.y,
      odometry.pose.pose.orientation.z,
      odometry.pose.pose.orientation.w);
    tf2::Matrix3x3 m(quatr);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return Eigen::Vector3d{x, y, yaw};
  }

  std::pair<double, double> lqr_control_input(
    const nav_msgs::msg::Odometry &reference_odom,
    const nav_msgs::msg::Odometry &current_odom,
    const double &t)
  {
    const auto qr = msg_to_state_vector(reference_odom);
    const auto q = msg_to_state_vector(current_odom);
    const auto e = express_in_rotated_frame(qr - q, q(3));

    const auto xr = reference_odom.twist.twist.linear.x;
    const auto yr = reference_odom.twist.twist.linear.y;
    const auto vr = std::hypot(xr, yr);
    const auto wr = reference_odom.twist.twist.angular.z;

    Eigen::Matrix3d A; 
    A << 1., wr*t,   0.,
      -wr*t,   1., vr*t,
         0.,   0.,   1.;
    Eigen::Matrix<double, 3, 2> B;
    B << -t, 0.,
         0., 0.,
         0., -t;
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();

    const auto res = drake::systems::controllers::DiscreteTimeLinearQuadraticRegulator(A, B, Q, R);
    const auto K = res.K;
    const auto u = -K * e;
    const auto v = vr + std::cos(e(3)) + u(1);
    const auto w = wr + u(2);

    return std::make_pair(v, w);
  }

  void execute(const std::shared_ptr<GoalHandleTrackTrajectory> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TrackTrajectory::Feedback>();
    auto & trajectory_points_remaining = feedback->trajectory_points_remaining;
    trajectory_points_remaining = goal->trajectory.size();
    auto result = std::make_shared<TrackTrajectory::Result>();

    for (unsigned long int i = 0; (i < goal->trajectory.size()) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // TODO: make this work
     // auto &q = goal->trajectory[i];
     // auto u = lqr_control_input(goal->trajectory[i], q, 0.1);
     // auto v = u.first;
     // auto w = u.second;

      auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_msg->linear.x = goal->trajectory[i].twist.twist.linear.x;
      cmd_msg->angular.z = goal->trajectory[i].twist.twist.angular.z;

      cmd_pub_->publish(std::move(cmd_msg));

      // Update feedback
      trajectory_points_remaining = goal->trajectory.size() - i;
      // Publish feedback
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  // brief Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

};  // class ControllerActionServer


}  // namespace robotino_control

RCLCPP_COMPONENTS_REGISTER_NODE(robotino_control::ControllerActionServer)
