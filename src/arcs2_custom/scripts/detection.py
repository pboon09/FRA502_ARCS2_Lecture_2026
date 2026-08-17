#!/usr/bin/python3

from arcs2_custom.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from arcs2_interfaces.msg import ObjectDetection, ObjectDetectionArray
import random


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.object_detection_publisher = self.create_publisher(ObjectDetectionArray, 'object_detection', 10)
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('DetectionNode has been started.')

    def object_detection_array_publisher(self, detections):
        msg = ObjectDetectionArray()
        msg.detections = detections
        self.object_detection_publisher.publish(msg)

    def object_detection_generator(self, id_no, name, confidence, x, y):
        detection = ObjectDetection()
        detection.object_id = str(id_no)
        detection.class_name = str(name)
        detection.confidence = confidence
        detection.position.x = x
        detection.position.y = y
        return detection

    def timer_callback(self):

        number_of_detections = random.randint(0, 5)
        class_names = ['person', 'car', 'bicycle', 'dog', 'cat']
        detections = []

        for i in range(number_of_detections):
            id_no = i + 1
            name = random.choice(class_names)
            confidence = random.uniform(0.5, 1.0)
            x = random.uniform(0.0, 640.0)
            y = random.uniform(0.0, 480.0)
            detection = self.object_detection_generator(id_no, name, confidence, x, y)
            detections.append(detection)

        self.object_detection_array_publisher(detections)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
