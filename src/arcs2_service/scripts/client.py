#!/usr/bin/python3

from arcs2_service.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts
import random
from functools import partial

class Arcs2ClientNode(Node):
    def __init__(self):
        super().__init__('arcs2_client_node')

        self.add_client = self.create_client(AddTwoInts, 'my_add2ints')
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Arcs2ClientNode has been started.')

    def timer_callback(self):
        a = random.randint(0, 10)
        b = random.randint(0, 10)
        self.send_request(a, b)
        self.get_logger().info(f'Sent request: {a} + {b}')
        return

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.add_client.call_async(request)
        future.add_done_callback(partial(self.handle_response, a=a, b=b))

    def handle_response(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f'Response received: {a} + {b} = {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Arcs2ClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
