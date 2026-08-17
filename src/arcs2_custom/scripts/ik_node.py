#!/usr/bin/python3

from arcs2_custom.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from arcs2_interfaces.srv import CalculateIK
import random

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        self.create_service(CalculateIK, 'calculate_ik', self.ik_callback)
        self.get_logger().info('IKNode has been started.')

    def ik_callback(self, request, response):
        if request.target_pose.position.x == 0.0 and request.target_pose.position.y == 0.0 and request.target_pose.position.z == 0.0:
            response.success = False
            response.joint_positions = []
            response.message = "Target pose is at the origin, which is unreachable."
        elif request.target_pose.position.x < 0.0 or request.target_pose.position.y < 0.0 or request.target_pose.position.z < 0.0:
            response.success = False
            response.joint_positions = []
            response.message = "Target pose is in the negative space, which is unreachable."
        else:
            response.success = True
            response.joint_positions = [random.uniform(-3.14, 3.14) for _ in range(6)]
            response.message = "IK solution found."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
