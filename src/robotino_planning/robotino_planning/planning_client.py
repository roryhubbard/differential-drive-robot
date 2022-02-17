import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from robotino_msgs.srv import GetTrajectory


class PlanningClient(Node):

  def __init__(self):
    super().__init__('planning_client')
    self.client = self.create_client(GetTrajectory, 'get_trajectory')
    while not self.client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')
    self.req = GetTrajectory.Request()

  def send_request(self):
    start = Odometry()
    start_point = Point(x=-2., y=-2., z=0.)
    start.pose.pose.position = start_point
    goal = Odometry()
    goal_point = Point(x=2., y=2., z=0.)
    goal.pose.pose.position = goal_point

    self.req.start = start
    self.req.goal = goal
    self.future = self.client.call_async(self.req)


def main():
  rclpy.init()

  planning_client = PlanningClient()
  planning_client.send_request()

  while rclpy.ok():
    rclpy.spin_once(planning_client)
    if planning_client.future.done():
      try:
        response = planning_client.future.result()
      except Exception as e:
        planning_client.get_logger().info(
          'Service call failed %r' % (e,))
      else:
        planning_client.get_logger().info(response)
    break

  planning_client.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

