#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import LaserScan
import math
import time
import os


NODE_NAME = 'simple_obstacle_detection_node'
SUBSCRIBER_TOPIC_NAME = '/scan'
OBSTACLE_DETECTED_TOPIC_NAME = '/obstacle_detection'

class ObstacleDetection:
    def __init__(self):
        super().__init__(NODE_NAME)
        self.sub = self.create_subscription(LaserScan, SUBSCRIBER_TOPIC_NAME, self.detect_obstacle)
        self.obstacle_pub = self.create_publisher(Float32MultiArray, OBSTACLE_DETECTED_TOPIC_NAME, queue_size=3)
        self.obstacle_info = Float32MultiArray()
        
        # Lidar properties (needs to be updated to be ros parameters loaded from config depending on lidar brand)
        self.viewing_angle = 360

        # Obstacle distance limits (meters) (update/calibrate as needed)
        self.max_distance_tolerance = 0.6
        self.min_distance_tolerance = 0.2

        '''
        For LD06
        values at 0 degrees   ---> (straight)
        values at 90 degrees  ---> (full right)
        values at -90 degrees ---> (full left)
        '''

    def detect_obstacle(data):
        total_number_of_scans = len(data.ranges)
        scans_per_degree = int(total_number_of_scans/self.viewing_angle)

        angle_values = [0, 45, 67.5, 90, 315, 292.5, 270]
        range_values = []
        for angle in angle_values:
            range_values.append(data.ranges[angle*scans_per_degree])
        
        min_distance = min(range_values)
        min_angle_index = range_values.index(min(range_values))
        min_angle = angle_values[min_angle_index]

        if max_distance_tolerance >= abs(min_distance) >= min_distance_tolerance:
            angle_rad = (min_angle * math.pi) / 180
            normalized_angle = round(math.cos(angle_rad))
            obstacle_detected = 1.0

            # Publish ROS message
            self.obstacle_info.append(min_distance)
            self.obstacle_info.append(normalized_angle)
            self.obstacle_info.append(obstacle_detected)
            self.obstacle_pub.publish(self.obstacle_detected)

        else:
            # nonsense values
            min_distance = -1.0
            normalized_angle = -1.0
            obstacle_detected = 0.0

            # Publish ROS message
            self.obstacle_info.append(min_distance_data)
            self.obstacle_info.append(normalized_angle)
            self.obstacle_info.append(obstacle_detected)
            self.obstacle_pub.publish(self.obstacle_detected)


def main(args=None):
    rclpy.init(args=args)
    obstacle_detection = ObstacleDetection()
    try:
        rclpy.spin(obstacle_detection)
        obstacle_detection.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        obstacle_detection.get_logger().info(f'Shutting down {NODE_NAME}...')
        time.sleep(1)
        obstacle_detection.destroy_node()
        rclpy.shutdown()
        obstacle_detection.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()

