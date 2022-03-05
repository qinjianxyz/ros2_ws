import os
import cv2
import time
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from elements.yolo import OBJ_DETECTION
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32

# Give names for nodes and topics for ROS
NODE_NAME = 'mask_node'
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
MASK_DETECTION_TOPIC_NAME = '/mask_detection'
SERVO_TOPIC_NAME = '/servo'

# types of objects that can be detected
Object_classes = ['temp', 'with_mask', 'without_mask']
Object_detector = OBJ_DETECTION('weights/best.pt', Object_classes)


class MaskDetection(Node):

    def __init__(self):
        # initialize the node
        super().__init__(NODE_NAME)
        self.camera_subscriber = self.create_subscription(Image,
        CAMERA_TOPIC_NAME, self.detect_mask, 10)
        self.mask_publisher = self.create_publisher(Int32, MASK_DETECTION_TOPIC_NAME, 1)
        self.servo_publisher = self.create_publisher(Float32,
        SERVO_TOPIC_NAME, 10)
        self.bridge = CvBridge()
        self.mask_detected = Int32()
        self.mask_degree = Float32()

    def detect_mask(self, data):
        frame = self.bridge.imgmsg_to_cv2(data)  # camera feed

        # detection process
        objs = Object_detector.detect(frame)  # detect the object

        if objs is None:
            print(f"Nothing detected")

        # plotting
        for obj in objs:
            label = obj['label']
            score = obj['score']
            self.get_logger().info(f"Label: {label}, Score: {score}")
            print(f"Label: {label}, Score: {score}")
            # if a stop sign is detected send out a 1 else send out 0
            if label == 'without_mask' and score > 0.9:
                self.mask_detected.data = 1
                self.mask_degree.data = 0.0
                self.servo_publisher.publish(self.mask_degree)
                self.mask_publisher.publish(self.mask_detected)
            else:
                self.mask_degree.data = 90.0
                self.servo_publisher.publish(self.mask_degree)
                self.mask_detected.data = 0
                self.mask_publisher.publish(self.mask_detected)


def main(args=None):
    rclpy.init(args=args)
    mask_publisher = MaskDetection()
    try:
        rclpy.spin(mask_publisher)
        mask_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        mask_publisher.get_logger().info(
            f'Shutting down {NODE_NAME}...')
        time.sleep(1)
        mask_publisher.destroy_node()
        rclpy.shutdown()
        mask_publisher.get_logger().info(
        f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()
