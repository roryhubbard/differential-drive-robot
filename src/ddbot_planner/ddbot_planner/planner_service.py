import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.srv import GetPlan
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from .diff_drive_planner import DiffDrivePlanner


class PlannerService(Node):

  def __init__(self):
    super().__init__('planner_service')
    self.service = self.create_service(GetPlan, 'get_plan', self.get_plan)

  def get_plan(self, request, response):
    self.get_logger().info(
      f'Planner request received with start postion {request.start}, goal position {request.goal}')
    return self.hack_plan(request, response)

  def hack_plan(self, request, response):
    start_velocity = 0.
    goal_velocity = 0.
    start_acceleration = 0.
    goal_acceleration = 0.
    start_position = request.start.pose.position
    goal_position = request.goal.pose.position
    _, _, start_yaw = euler_from_quaternion(request.start.pose.orientation)
    _, _, goal_yaw = euler_from_quaternion(request.goal.pose.orientation)

    travel_time = 10
    time_samples = np.linspace(0, travel_time, 5)
    n_flat_outputs = 2
    poly_degree = 3
    smoothness_degree = 2
    ddp = DiffDrivePlanner(time_samples, n_flat_outputs,
                           poly_degree, smoothness_degree)

    ddp.add_cost()
    ddp.add_constraint(t=0, derivative_order=0,
      bounds=[start_position.x, start_position.y], equality=True)
    ddp.add_constraint(t=0, derivative_order=1,
      bounds=[start_velocity, start_velocity], equality=True)
    ddp.add_constraint(t=0, derivative_order=2,
      bounds=[start_acceleration, start_acceleration], equality=True)
    ddp.add_constraint(t=0, derivative_order=1,
      bounds=[start_velocity*np.cos(start_yaw), start_velocity*np.sin(start_yaw)], equality=True)

    ddp.add_constraint(t=travel_time, derivative_order=0,
      bounds=[goal_position.x, goal_position.y], equality=True)
    ddp.add_constraint(t=travel_time, derivative_order=1,
      bounds=[goal_velocity, goal_velocity], equality=True)
    ddp.add_constraint(t=travel_time, derivative_order=2,
      bounds=[goal_acceleration, goal_acceleration], equality=True)
    ddp.add_constraint(t=travel_time, derivative_order=1,
      bounds=[goal_velocity*np.cos(goal_yaw), goal_velocity*np.sin(goal_yaw)], equality=True)

    square = np.array([
      [1, 1],
      [-1, 1],
      [-1, -1],
      [1, -1],
      [1, 1],
    ])
    checkpoints = np.linspace(0, travel_time, 22)
    ddp.add_obstacle(square, checkpoints)

    ddp.solve()

    t_result = np.linspace(0, 10, 100)
    for t in t_result:
      x, y = ddp.eval(t, 0)
      yaw = ddp.recover_yaw(t, 0)
      pose_stamped = PoseStamped()
      pose_stamped.pose.position = Point(x=x, y=y, z=0.)
      pose_stamped.pose.orientation = quaternion_from_euler(0, 0, yaw)
      response.plan.poses.append(pose_stamped)

    return response


def main():
  rclpy.init()
  planner_service = PlannerService()
  rclpy.spin(planner_service)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
