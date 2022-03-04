import os
import cv2
import time
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from elements.yolo import OBJ_DETECTION
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

# Give names for nodes and topics for ROS
MASK_NODE_NAME = 'mask_node'
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
MASK_DETECTION_TOPIC_NAME = '/mask_detection'

# types of objects that can be detected
Object_classes = ['with_mask', 'without_mask']
Object_detector = OBJ_DETECTION('weights/best.pt', Object_classes)


class MaskDetection(Node):

    def __init__(self):
        # initialize the node
        self.init_node = rclpy.init_node(MASK_NODE_NAME, anonymous=False)
        self.camera_subscriber = rclpy.Subscriber(
            CAMERA_TOPIC_NAME, Image, self.detect_stop)
        self.mask_publisher = rclpy.Publisher(
            MASK_DETECTION_TOPIC_NAME, Int32, queue_size=1)
        self.bridge = CvBridge()
        self.mask_detected = Int32()

    def detect_mask(self, data):
        frame = self.bridge.imgmsg_to_cv2(data)  # camera feed

        # detection process
        objs = Object_detector.detect(frame)  # detect the object

        # plotting
        for obj in objs:
            label = obj['label']
            score = obj['score']

            # if a stop sign is detected send out a 1 else send out 0
            if label == 'mask_off' and score > 0.1:
                self.mask_detected.data = 1
                self.mask_publisher.publish(self.mask_detected)
            else:
                self.mask_detected.data = 0
                self.mask_publisher.publish(self.mask_detected)


if __name__ == '__main__':
    Mask_detector = MaskDetection()
    rate = rclpy.Rate(15)
    while not rclpy.is_shutdown():
        rclpy.spin()
        rate.sleep()
