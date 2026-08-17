#!/usr/bin/python3

from arcs2_service.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts
import time

class Arcs2ServerNode(Node):
    def __init__(self):
        super().__init__('arcs2_server_node')

        self.create_service(AddTwoInts, 'my_add2ints', self.add_two_ints_callback)
        self.get_logger().info('Arcs2ServerNode has been started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Sending response: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Arcs2ServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
