#! urs/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute

class TurtleMover(Node):

    def __init__(self):
        super().__init__(node_name="Turtle_move")
        self.client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting...")
        self.req = TeleportAbsolute.Request()

    def move_turtle(self, x, y, theta):
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.move_callback)

    def move_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Success')
        except Exception as e:
            self.get_logger().info(f'{str(e)}')

def main(args = None):
    rclpy.init(args=args)
    node = TurtleMover()
    rclpy.spin(node)
    rclpy.shutdown()