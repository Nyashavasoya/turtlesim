#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import SetParameters

class TurtleParameters(Node):

    def __init__(self):
        super().__init__('turtle_parameters')
        self.declare_parameter('background_r', 255)
        self.declare_parameter('background_g', 255)
        self.declare_parameter('background_b', 255)

        self.background_r = self.get_parameter('background_r').get_parameter_value().integer_value
        self.background_g = self.get_parameter('background_g').get_parameter_value().integer_value
        self.background_b = self.get_parameter('background_b').get_parameter_value().integer_value

        self.set_parameters_srv = self.create_service(SetParameters, 'set_background_color', self.set_parameters_callback)

    def set_parameters_callback(self, request, response):
        self.background_r = request.parameters[0].integer_value
        self.background_g = request.parameters[1].integer_value
        self.background_b = request.parameters[2].integer_value
        self.get_logger().info(f'Set background color to ({self.background_r}, {self.background_g}, {self.background_b})')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtleParameters()
    rclpy.spin(node)
    rclpy.shutdown()