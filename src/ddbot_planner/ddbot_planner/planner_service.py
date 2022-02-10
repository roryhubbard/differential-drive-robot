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
    vi = 0.
    vf = 0.
    ai = 0.
    af = 0.
    pi = request.start.pose.position
    pf = request.goal.pose.position
    qi = request.start.pose.orientation
    qf = request.goal.pose.orientation
    _rolli, _pitchi, yawi = euler_from_quaternion([qi.x, qi.y, qi.z, qi.w])
    _rollf, _pitchf, yawf = euler_from_quaternion([qf.x, qf.y, qf.z, qf.w])

    travel_time = 10
    time_samples = np.linspace(0, travel_time, 5)
    n_flat_outputs = 2
    poly_degree = 3
    smoothness_degree = 2
    ddp = DiffDrivePlanner(time_samples, n_flat_outputs,
                           poly_degree, smoothness_degree)

    ddp.add_cost()
    ddp.add_constraint(t=0, derivative_order=0, bounds=[pi.x, pi.y], equality=True)
    ddp.add_constraint(t=0, derivative_order=1, bounds=[vi, vi], equality=True)
    ddp.add_constraint(t=0, derivative_order=2, bounds=[ai, ai], equality=True)

    ddp.add_constraint(t=travel_time, derivative_order=0, bounds=[pf.x, pf.y], equality=True)
    ddp.add_constraint(t=travel_time, derivative_order=1, bounds=[vf, vf], equality=True)
    ddp.add_constraint(t=travel_time, derivative_order=2, bounds=[af, af], equality=True)

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
      q = quaternion_from_euler(0, 0, yaw)
      pose_stamped.pose.orientation.x = q[0]
      pose_stamped.pose.orientation.y = q[1]
      pose_stamped.pose.orientation.z = q[2]
      pose_stamped.pose.orientation.w = q[3]
      response.plan.poses.append(pose_stamped)

    return response


def main():
  rclpy.init()
  planner_service = PlannerService()
  rclpy.spin(planner_service)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
