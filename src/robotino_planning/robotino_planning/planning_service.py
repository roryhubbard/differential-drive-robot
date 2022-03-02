import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovariance, TwistWithCovariance
from robotino_msgs.srv import GetTrajectory
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from .diff_drive_planner import DiffDrivePlanner


def polygon_to_array(polygon):
  return np.array([
    [p.x, p.y]
    for p in polygon.points
  ])


class PlanningService(Node):

  def __init__(self):
    super().__init__('planning_service')
    self.service = self.create_service(GetTrajectory, 'get_trajectory', self.get_trajectory)

    self.n_flat_outputs = 2
    self.poly_degree = 5
    self.smoothness_degree = 2
    # TODO: calculate bigM on a per map basis
    self.bigM = 32

  def get_trajectory(self, request, response):
    if len(request.goal_odometries) != len(request.goal_times):
      self.get_logger().error(
          'Planning request received with unmatched goal odometries and goal time, ignoring request')
    self.get_logger().info(
        f'Planning request received with start point {request.start_odometry.pose.pose.position}, final goal point {request.goal_odometries[-1].pose.pose.position}')

    pi = request.start_odometry.pose.pose.position
    qi = request.start_odometry.pose.pose.orientation
    vi = request.start_odometry.twist.twist.linear

    num_splines = len(request.goal_times)
    travel_time = request.goal_times[-1]
    time_samples = np.linspace(0, travel_time, num_splines+1)
    ddp = DiffDrivePlanner(time_samples, self.n_flat_outputs,
                           self.poly_degree, self.smoothness_degree)

    # TODO: incorporate starting yaw and yaw derivative constraints
    _rolli, _pitchi, _yawi = euler_from_quaternion([qi.x, qi.y, qi.z, qi.w])
    _wi = request.start_odometry.twist.twist.angular

    ddp.add_cost()
    ddp.add_constraint(t=0, derivative_order=0, bounds=[pi.x, pi.y], equality=True)
    ddp.add_constraint(t=0, derivative_order=1, bounds=[vi.x, vi.y], equality=True)
    ddp.add_constraint(t=0, derivative_order=2, bounds=[0., 0.], equality=True)

    for i, goal_odometry in enumerate(request.goal_odometries):
      pf = goal_odometry.pose.pose.position
      qf = goal_odometry.pose.pose.orientation
      vf = goal_odometry.twist.twist.linear

      goal_time = request.goal_times[i]
      ddp.add_constraint(t=goal_time, derivative_order=0, bounds=[pf.x, pf.y], equality=True)

      # only enforce velocity and acceleration constraints at the end of the trajectory
      if i == len(request.goal_odometries) - 1:
        ddp.add_constraint(t=goal_time, derivative_order=1, bounds=[vf.x, vf.y], equality=True)
        ddp.add_constraint(t=goal_time, derivative_order=2, bounds=[0., 0.], equality=True)

      # TODO: incorporate goal yaw and yaw derivative constraints
      _rollf, _pitchf, _yawf = euler_from_quaternion([qf.x, qf.y, qf.z, qf.w])
      _wf = goal_odometry.twist.twist.angular

    for obstacle in request.obstacles:
      obstacle_array = polygon_to_array(obstacle)
      checkpoints = np.linspace(0, travel_time, travel_time*request.control_frequency)
      ddp.add_obstacle(obstacle_array, checkpoints, self.bigM)

    ddp.solve()

    for t in np.linspace(0, travel_time, travel_time*request.control_frequency):
      x, y = ddp.eval(t, 0)
      yaw = ddp.recover_yaw(t)
      v = ddp.recover_longitudinal_velocity(t)
      w = ddp.recover_angular_velocity(t)

      pose = PoseWithCovariance()
      pose.pose.position = Point(x=x, y=y)
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

