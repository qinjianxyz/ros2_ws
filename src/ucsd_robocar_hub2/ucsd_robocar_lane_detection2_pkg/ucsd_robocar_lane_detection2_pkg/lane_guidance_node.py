import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Int32MultiArray
from geometry_msgs.msg import Twist
import time
import os

NODE_NAME = 'lane_guidance_node'
CENTROID_TOPIC_NAME = '/centroid'
ACTUATOR_TOPIC_NAME = '/cmd_vel'
SERVO_TOPIC_NAME = '/servo'
MASK_DETECTION_TOPIC_NAME = '/mask_detection'

class PathPlanner(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        self.mask_detected = 0
        self.mask_detection_subscriber = self.create_subscription(Float32,
        MASK_DETECTION_TOPIC_NAME, self.mask_detection_callback, 10)
        self.centroid_subscriber = self.create_subscription(Float32,
            CENTROID_TOPIC_NAME, self.controller, 10)

        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('steering_sensitivity', 1),
                ('no_error_throttle', 1),
                ('error_throttle', 1),
                ('error_threshold', 1),
                ('zero_throttle', 1)
            ])
        self.steering_sensitivity = self.get_parameter('steering_sensitivity').value
        self.no_error_throttle = self.get_parameter('no_error_throttle').value
        self.error_throttle = self.get_parameter('error_throttle').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.zero_throttle = self.get_parameter('zero_throttle').value
        self.get_logger().info(
            f'\nsteering_sensitivity: {self.steering_sensitivity}'
            f'\nno_error_throttle: {self.no_error_throttle}'
            f'\nerror_throttle: {self.error_throttle}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_throttle: {self.zero_throttle}'
        )

    def mask_detection_callback(self, data):
        self.mask_detected = int(data.data)

    def controller(self, data):
        try:
            kp = self.steering_sensitivity
            error_x = data.data
            self.get_logger().info(f"{error_x}")

            # mask_detection
            
            if self.mask_detected == 1:
                throttle_float = float(self.zero_throttle)
            else:
                throttle_float = float(self.no_error_throttle)
            
            '''
            if error_x <= self.error_threshold:
                throttle_float = float(self.no_error_throttle)
            else:
                throttle_float = float(self.error_throttle)
            '''
            steering_float = float(kp * error_x)
            if steering_float < -1.0:
                steering_float = -1.0
            elif steering_float > 1.0:
                steering_float = 1.0
            else:
                pass
            self.twist_cmd.angular.z = steering_float
            self.twist_cmd.linear.x = throttle_float
            self.twist_publisher.publish(self.twist_cmd)

        except KeyboardInterrupt:
            self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)


def main(args=None):
    rclpy.init(args=args)
    path_planner_publisher = PathPlanner()
    try:
        rclpy.spin(path_planner_publisher)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        path_planner_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        path_planner_publisher.twist_cmd.linear.x = path_planner_publisher.zero_throttle
        path_planner_publisher.twist_publisher.publish(path_planner_publisher.twist_cmd)
        time.sleep(1)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
        path_planner_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
