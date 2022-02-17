import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from robotino_msgs.srv import GetTrajectory


class PlannerClient(Node):

  def __init__(self):
    super().__init__('planner_client')
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

  planner_client = PlannerClient()
  planner_client.send_request()

  while rclpy.ok():
    rclpy.spin_once(planner_client)
    if planner_client.future.done():
      try:
        response = planner_client.future.result()
      except Exception as e:
        planner_client.get_logger().info(
          'Service call failed %r' % (e,))
      else:
        planner_client.get_logger().info(response)
    break

  planner_client.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

