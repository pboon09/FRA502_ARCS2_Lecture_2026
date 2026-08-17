#!/usr/bin/python3

from arcs2_custom.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from arcs2_interfaces.msg import BatteryStatus
import random

class BatteryStatusNode(Node):
    def __init__(self):
        super().__init__('battery_status_node')

        self.battery_status_publisher = self.create_publisher(BatteryStatus, 'battery_status', 10)
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('BatteryStatusNode has been started.')

    def battery_publisher(self, voltage, current, percentage, charging):
        msg = BatteryStatus()
        msg.battery_voltage = voltage
        msg.battery_current = current
        msg.battery_percentage = percentage
        msg.charging = charging

        self.battery_status_publisher.publish(msg)

    def read_battery_status(self):
        v = random.uniform(11.0, 13.0)
        c = random.uniform(0.0, 2.0)
        p = random.uniform(0.0, 100.0)
        ch = random.choice([True, False])
        return v, c, p, ch

    def timer_callback(self):
        v, c, p, ch = self.read_battery_status()
        self.battery_publisher(v, c, p, ch)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
