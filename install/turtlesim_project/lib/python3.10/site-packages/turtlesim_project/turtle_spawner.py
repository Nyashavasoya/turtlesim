#! usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class TurtleSpawn(Node):

    def __init__(self):
        super().__init__(node_name="turtle_spawner")
        self.client = self.create_client(Spawn, "spawn")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting...")
        self.req = Spawn.Request()

    def spawn_turtle(self, x, y, theta, name):
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.req.name = name
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Successfully spawned turtle: {response.name}')
        except Exception as e:
            self.get_logger().error(f'Failed to spawn turtle: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawn()
    rclpy.spin(node=node)
    rclpy.shutdown()