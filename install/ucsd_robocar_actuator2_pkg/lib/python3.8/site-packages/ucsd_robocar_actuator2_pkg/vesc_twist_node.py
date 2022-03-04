import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .vesc_submodule.vesc_client import VESC_
import time

NODE_NAME = 'vesc_twist_node'
TOPIC_NAME = '/cmd_vel'


class VescTwist(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.vesc = VESC_()
        self.rpm_subscriber = self.create_subscription(Twist, TOPIC_NAME, self.callback, 10)

        # Default actuator values
        self.default_rpm_value = int(5000) 
        self.default_steering_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
        self.default_throttle_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_rpm', self.default_rpm_value),
                ('steering_polarity', self.default_steering_polarity),
                ('throttle_polarity', self.default_throttle_polarity)
            ])
        self.max_rpm = int(self.get_parameter('max_rpm').value)
        self.steering_polarity = int(self.get_parameter('steering_polarity').value)
        self.throttle_polarity = int(self.get_parameter('throttle_polarity').value)

        self.get_logger().info(
            f'\nmax_rpm: {self.max_rpm}'
            f'\nsteering_polarity: {self.steering_polarity}'
            f'\nthrottle_polarity: {self.throttle_polarity}'
            )


    def callback(self, msg):
        self.max_rpm = self.get_parameter('max_rpm').value # ability to update max RPM in real-time
        self.steering_polarity = self.get_parameter('steering_polarity').value # ability to update steering polarity in real-time
        self.throttle_polarity = self.get_parameter('throttle_polarity').value # ability to update throttle polarity in real-time
        # self.get_logger().info(
        #     f'\nmax_rpm: {self.max_rpm}'
        #     f'\nsteering_polarity: {self.steering_polarity}'
        #     f'\nthrottle_polarity: {self.throttle_polarity}'
        #     )
        # Steering map from [-1,1] --> [0,1]  
        data_min_limit = -1
        data_max_limit = 1 
        vesc_min_limit = 0 # These will be rosparams eventually... : max_left
        vesc_max_limit = 1 # These will be rosparams eventually... : max_right
        steering_angle = float(-0.1 + ((msg.angular.z-data_min_limit)*(vesc_max_limit - vesc_min_limit))/(data_max_limit-data_min_limit))
        
        # RPM map from [-1,1] --> [-max_rpm,max_rpm]
        rpm = self.max_rpm * msg.linear.x

        self.vesc.send_rpm(int(self.throttle_polarity * rpm))
        self.vesc.send_servo_angle(float(self.steering_polarity * steering_angle))


def main(args=None):
    rclpy.init(args=args)
    try:
        vesc_twist = VescTwist()
        rclpy.spin(vesc_twist)
        vesc_twist.destroy_node()
        rclpy.shutdown()
    except:
        vesc_twist.get_logger().info(f'Could not connect to VESC, Shutting down {NODE_NAME}...')
        time.sleep(1)
        vesc_twist.destroy_node()
        rclpy.shutdown()
        vesc_twist.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
