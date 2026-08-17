#!/usr/bin/python3

from FRA502_ARCS2_Lecture_2026.src.arcs2_topic.arcs2_topic.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  


class Arcs2PublisherNode(Node):
    def __init__(self):
        super().__init__('Arcs2PublisherNode')

        self.count = 0

        rate = 20.0

        self.count_publisher = self.create_publisher(Int32, 'count', 10)
        self.create_timer(1.0/rate, self.publish_count)

        self.get_logger().info('Arcs2PublisherNode has been started.')

    def publish_count(self):
        msg = Int32()
        msg.data = self.count
        self.count_publisher.publish(msg)
        self.get_logger().info(f'Published count: {self.count}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = Arcs2PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
