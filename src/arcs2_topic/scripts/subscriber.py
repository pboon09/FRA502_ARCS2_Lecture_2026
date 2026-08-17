#!/usr/bin/python3

from FRA502_ARCS2_Lecture_2026.src.arcs2_topic.arcs2_topic.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  


class Arcs2SubscriberNode(Node):
    def __init__(self):
        super().__init__('Arcs2SubscriberNode')

        self.count = 0

        self.count_subscriber = self.create_subscription(Int32, 'count', self.count_callback, 10)
        self.get_logger().info('Arcs2SubscriberNode has been started.')

    def count_callback(self, msg):
        self.count = msg.data
        self.get_logger().info(f'Received count: {self.count}')

def main(args=None):
    rclpy.init(args=args)
    node = Arcs2SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
