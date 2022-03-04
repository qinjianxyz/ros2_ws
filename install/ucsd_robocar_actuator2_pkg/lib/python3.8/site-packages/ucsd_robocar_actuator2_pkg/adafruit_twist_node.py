import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit

NODE_NAME = 'adafruit_twist_node'
TOPIC_NAME = '/cmd_vel'

class AdafruitTwist(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.rpm_subscriber = self.create_subscription(Twist, TOPIC_NAME, self.callback, 10)
        self.kit = ServoKit(channels=16)

        # Default actuator values
        self.default_steering_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
        self.default_throttle_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
        self.declare_parameters(
            namespace='',
            parameters=[
                ('steering_polarity', self.default_steering_polarity),
                ('throttle_polarity', self.default_throttle_polarity)
            ])
        self.steering_polarity = int(self.get_parameter('steering_polarity').value)
        self.throttle_polarity = int(self.get_parameter('throttle_polarity').value)

    def callback(self, msg):
        self.steering_polarity = self.get_parameter('steering_polarity').value # ability to update steering polarity in real-time
        self.throttle_polarity = self.get_parameter('throttle_polarity').value # ability to update throttle polarity in real-time
        # Steering map from [-1,1] --> [max_left,max_right] # to do: implement into calibration
        data_min_limit = -1
        data_max_limit = 1
        adafruit_min_limit = -1 # These will be rosparams eventually... : max_left
        adafruit_max_limit = 1  # These will be rosparams eventually... : max_right
        steering_angle = float(-0.1 + ((msg.angular.z-data_min_limit)*(adafruit_max_limit - adafruit_min_limit))/(data_max_limit-data_min_limit))

        # Send values to adafruit board 
        self.kit.servo[1].angle = float(self.steering_polarity * 90 * (1 + msg.angular.z))
        self.kit.continuous_servo[2].throttle = self.throttle_polarity * msg.linear.x


def main(args=None):
    rclpy.init(args=args)
    try:
        adafruit_twist = AdafruitTwist()
        rclpy.spin(adafruit_twist)
        adafruit_twist.destroy_node()
        rclpy.shutdown()
    except:
        adafruit_twist.get_logger().info(f'Could not connect to Adafruit, Shutting down {NODE_NAME}...')
        time.sleep(1)
        adafruit_twist.destroy_node()
        rclpy.shutdown()
        adafruit_twist.get_logger().info(f'{NODE_NAME} shut down successfully.')



if __name__ == '__main__':
    main()
