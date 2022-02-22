import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovariance, TwistWithCovariance
from robotino_msgs.srv import GetTrajectory
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from .diff_drive_planner import DiffDrivePlanner


class PlanningService(Node):

  def __init__(self):
    super().__init__('planning_service')
    self.service = self.create_service(GetTrajectory, 'get_trajectory', self.get_trajectory)

  def get_trajectory(self, request, response):
    self.get_logger().info(
      f'Planning request received with start point {request.start}, goal point {request.goal}')
    return self.hack_plan(request, response)

  def hack_plan(self, request, response):
    pi = request.start.pose.pose.position
    qi = request.start.pose.pose.orientation
    vi = request.start.twist.twist.linear
    wi = request.start.twist.twist.angular

    pf = request.goal.pose.pose.position
    qf = request.goal.pose.pose.orientation
    vf = request.goal.twist.twist.linear
    wf = request.goal.twist.twist.angular

    _rolli, _pitchi, yawi = euler_from_quaternion([qi.x, qi.y, qi.z, qi.w])
    _rollf, _pitchf, yawf = euler_from_quaternion([qf.x, qf.y, qf.z, qf.w])

    travel_time = 10
    time_samples = np.linspace(0, travel_time, 5)
    n_flat_outputs = 2
    poly_degree = 3
    smoothness_degree = 2
    ddp = DiffDrivePlanner(time_samples, n_flat_outputs, poly_degree, smoothness_degree)

    ddp.add_cost()
    ddp.add_constraint(t=0, derivative_order=0, bounds=[pi.x, pi.y], equality=True)
    ddp.add_constraint(t=0, derivative_order=1, bounds=[vi.x, vi.y], equality=True)
    ddp.add_constraint(t=0, derivative_order=2, bounds=[0., 0.], equality=True)

    ddp.add_constraint(t=travel_time, derivative_order=0, bounds=[pf.x, pf.y], equality=True)
    ddp.add_constraint(t=travel_time, derivative_order=1, bounds=[vf.x, vf.x], equality=True)
    ddp.add_constraint(t=travel_time, derivative_order=2, bounds=[0., 0.], equality=True)

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
      yaw = ddp.recover_yaw(t)
      v = ddp.recover_longitudinal_velocity(t)
      w = ddp.recover_angular_velocity(t)

      pose = PoseWithCovariance()
      pose.pose.position = Point(x=x, y=y, z=0.)
      q = quaternion_from_euler(0, 0, yaw)
      pose.pose.orientation.x = q[0]
      pose.pose.orientation.y = q[1]
      pose.pose.orientation.z = q[2]
      pose.pose.orientation.w = q[3]

      twist = TwistWithCovariance()
      twist.twist.linear.x = v
      twist.twist.angular.z = w

      odometry = Odometry()
      odometry.pose = pose
      odometry.twist = twist

      response.trajectory.append(odometry)

    return response


def main():
  rclpy.init()
  planning_service = PlanningService()
  rclpy.spin(planning_service)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
