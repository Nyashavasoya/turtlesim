#! usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from turtlesim.action import MoveTurtleSquare
from turtlesim.srv import TeleportAbsolute, SetParameters
from geometry_msgs.msg import Twist
import math

class TurtleActionServer(Node):

    def __init__(self):
        super().__init__('turtle_action_server')
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('speed', 1.0)

        self.side_length = self.get_parameter('side_length').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value

        self.action_server = ActionServer(
            self,
            MoveTurtleSquare,
            'move_turtle_square',
            self.execute_callback)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')

        self.set_parameters_srv = self.create_service(SetParameters, 'set_turtle_action_parameters', self.set_parameters_callback)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        side_length = self.side_length
        speed = self.speed

        for _ in range(4):
            self.move_straight(side_length, speed)
            self.rotate(math.pi / 2, speed)

        goal_handle.succeed()
        result = MoveTurtleSquare.Result()
        return result

    def move_straight(self, distance, speed):
        twist = Twist()
        twist.linear.x = speed

        duration = distance / speed
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + duration
        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            self.cmd_vel_publisher.publish(twist)

        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)

    def rotate(self, angle, speed):
        twist = Twist()
        twist.angular.z = speed

        duration = angle / speed
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + duration
        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            self.cmd_vel_publisher.publish(twist)

        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def set_parameters_callback(self, request, response):
        self.side_length = request.parameters[0].double_value
        self.speed = request.parameters[1].double_value
        self.get_logger().info(f'Set parameters to side_length: {self.side_length}, speed: {self.speed}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtleActionServer()
    rclpy.spin(node)
    rclpy.shutdown()