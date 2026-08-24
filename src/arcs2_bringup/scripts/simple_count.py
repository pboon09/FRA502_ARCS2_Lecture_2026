#!/usr/bin/python3

from arcs2_bringup.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

class SimpleCountNode(Node):
    def __init__(self):
        super().__init__('simple_count_node')

        namespace = self.get_namespace()
        node_name = self.get_name()

        self.declare_parameter('count', 0)
        self.declare_parameter('multiplier', 1)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('log', True)

        self.count = self.get_parameter('count').value
        self.multiplier = self.get_parameter('multiplier').value
        publish_rate = self.get_parameter('publish_rate').value
        self.log = self.get_parameter('log').value

        self.count_publisher = self.create_publisher(Int32, 'count', 10)
        self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(f'{node_name} started with count={self.count}')
        self.get_logger().info(f'Multiplier: {self.multiplier}, Publish rate: {publish_rate} Hz, Logging: {self.log}')
        self.get_logger().info(f'Namespace: {namespace}')

    def publish_count(self, data):
        msg = Int32()
        msg.data = data
        self.count_publisher.publish(msg)

    def timer_callback(self):
        self.count += 1
        self.publish_count(self.count * self.multiplier)

        if self.log:
            self.get_logger().info(f'Published count: {self.count * self.multiplier}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCountNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
