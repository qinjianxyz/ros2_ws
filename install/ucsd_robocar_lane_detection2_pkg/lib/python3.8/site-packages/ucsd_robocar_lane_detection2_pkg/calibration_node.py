import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Int32MultiArray
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import os.path
import time

# Nodes in this program
CALIBRATION_NODE_NAME = 'calibration_node'

# Nodes listening to rosparameters
LG_NODE_NAME = 'lane_guidance_node'
LD_NODE_NAME = 'lane_detection_node'
VESC_NODE_NAME = 'vesc_twist_node'
ADA_NODE_NAME = 'adafruit_twist_node'

# Topics subscribed/published to in this program
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
ACTUATOR_TOPIC_NAME = '/cmd_vel'

IMG_WINDOW_NAME = 'img'
BW_WINDOW_NAME = 'blackAndWhiteImage'
MASK_WINDOW_NAME = 'mask'
THR_STR_WINDOW_NAME = 'throttle_and_steering'

cv2.namedWindow(IMG_WINDOW_NAME)
cv2.namedWindow(BW_WINDOW_NAME)
cv2.namedWindow(MASK_WINDOW_NAME)
cv2.namedWindow(THR_STR_WINDOW_NAME)

#
def callback(x):
    pass


def slider_to_normalized(slider_input):
    input_start = 0
    input_end = 2000
    output_start = -1
    output_end = 1
    normalized_output = float(output_start + (slider_input - input_start) * (
            (output_end - output_start) / (input_end - input_start)))
    return normalized_output


lowH = 0
highH = 179
lowS = 0
highS = 255
lowV = 0
highV = 255

glow = 0
ghigh = 255

not_inverted = 0
inverted = 1

min_width = 10
max_width = 500

max_number_of_lines = 100
max_error_threshold = 100
default_error_threshold = 20

min_frame_height = 1
max_frame_height = 100
default_frame_width = 100
max_frame_width = 100
default_min_rows = 50
max_rows = 100
default_min_offset = 50
max_offset = 100

steer_left = 0
steer_straight = 1000
steer_right = 2000

steer_sensitivity_max = 100
steer_sensitivity_default = 100

throttle_reverse = 0
throttle_neutral = 1000
throttle_forward = 2000

zero_throttle_mode = 0
zero_error_throttle_mode = 1
error_throttle_mode = 2
max_rpm = 10000
zero_rpm = 0
steering_polarity_normal = 1
steering_polarity_reversed = 0
throttle_polarity_normal = 1
throttle_polarity_reversed = 0


cv2.createTrackbar('lowH', MASK_WINDOW_NAME , lowH, highH, callback)
cv2.createTrackbar('highH', MASK_WINDOW_NAME , highH, highH, callback)
cv2.createTrackbar('lowS', MASK_WINDOW_NAME , lowS, highS, callback)
cv2.createTrackbar('highS', MASK_WINDOW_NAME , highS, highS, callback)
cv2.createTrackbar('lowV', MASK_WINDOW_NAME , lowV, highV, callback)
cv2.createTrackbar('highV', MASK_WINDOW_NAME , highV, highV, callback)


cv2.createTrackbar('gray_lower', BW_WINDOW_NAME , glow, ghigh, callback)
cv2.createTrackbar('Inverted_filter', BW_WINDOW_NAME , not_inverted, inverted, callback)

cv2.createTrackbar('min_width', IMG_WINDOW_NAME, min_width, max_width, callback)
cv2.createTrackbar('max_width', IMG_WINDOW_NAME, max_width, max_width, callback)
cv2.createTrackbar('number_of_lines', IMG_WINDOW_NAME, max_number_of_lines, max_number_of_lines, callback)
cv2.createTrackbar('error_threshold', IMG_WINDOW_NAME, default_error_threshold, max_error_threshold, callback)

cv2.createTrackbar('frame_width', IMG_WINDOW_NAME, default_frame_width, max_frame_width, callback)
cv2.createTrackbar('rows_to_watch', IMG_WINDOW_NAME, default_min_rows, max_rows, callback)
cv2.createTrackbar('rows_offset', IMG_WINDOW_NAME, default_min_offset, max_offset, callback)

cv2.createTrackbar('Steering_sensitivity', THR_STR_WINDOW_NAME, steer_sensitivity_default, steer_sensitivity_max, callback)
cv2.createTrackbar('Steering_value', THR_STR_WINDOW_NAME, steer_straight, steer_right, callback)
cv2.createTrackbar('Throttle_mode', THR_STR_WINDOW_NAME, zero_throttle_mode, error_throttle_mode, callback)
cv2.createTrackbar('Throttle_value', THR_STR_WINDOW_NAME, throttle_neutral, throttle_forward, callback)
cv2.createTrackbar('max_rpm', THR_STR_WINDOW_NAME, max_rpm, max_rpm, callback)
cv2.createTrackbar('steering_polarity', THR_STR_WINDOW_NAME, steering_polarity_normal, steering_polarity_normal, callback)
cv2.createTrackbar('throttle_polarity', THR_STR_WINDOW_NAME, throttle_polarity_normal, throttle_polarity_normal, callback)


class Calibration(Node):
    def __init__(self):
        super().__init__(CALIBRATION_NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.live_calibration_values, 10)
        self.camera_subscriber
        self.bridge = CvBridge()

        # declare parameters
        self.declare_parameter('Hue_low')
        self.declare_parameter('Hue_high')
        self.declare_parameter('Saturation_low')
        self.declare_parameter('Saturation_high')
        self.declare_parameter('Value_low')
        self.declare_parameter('Value_high')
        self.declare_parameter('gray_lower')
        self.declare_parameter('inverted_filter')
        self.declare_parameter('Width_min')
        self.declare_parameter('Width_max')
        self.declare_parameter('number_of_lines')
        self.declare_parameter('error_threshold')
        self.declare_parameter('camera_start_height')
        self.declare_parameter('camera_bottom_height')
        self.declare_parameter('camera_left_width')
        self.declare_parameter('camera_right_width')
        self.declare_parameter('Steering_sensitivity')
        self.declare_parameter('zero_throttle')
        self.declare_parameter('zero_error_throttle')
        self.declare_parameter('error_throttle')
        self.declare_parameter('max_rpm')
        self.declare_parameter('steering_polarity')
        self.declare_parameter('throttle_polarity')

        
        # Get previously set params
        self.Hue_low = self.get_parameter('Hue_low').value
        self.Hue_high = self.get_parameter('Hue_high').value
        self.Saturation_low = self.get_parameter('Saturation_low').value
        self.Saturation_high = self.get_parameter('Saturation_high').value
        self.Value_low = self.get_parameter('Value_low').value
        self.Value_high = self.get_parameter('Value_high').value
        self.gray_lower = self.get_parameter('gray_lower').value
        self.inverted_filter = self.get_parameter('inverted_filter').value
        self.number_of_lines = self.get_parameter('number_of_lines').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.min_width = self.get_parameter('Width_min').value
        self.max_width = self.get_parameter('Width_max').value
        self.number_of_lines = self.get_parameter('number_of_lines').value
        self.error_threshold = self.get_parameter('error_threshold').value 
        self.camera_start_height = self.get_parameter('camera_start_height').value
        self.camera_bottom_height = self.get_parameter('camera_bottom_height').value
        self.camera_left_width = self.get_parameter('camera_left_width').value
        self.camera_right_width = self.get_parameter('camera_right_width').value
        self.Steering_sensitivity = self.get_parameter('Steering_sensitivity').value
        self.zero_throttle = self.get_parameter('zero_throttle').value
        self.zero_error_throttle = self.get_parameter('zero_error_throttle').value
        self.error_throttle = self.get_parameter('error_throttle').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.steering_polarity = self.get_parameter('steering_polarity').value
        self.throttle_polarity = self.get_parameter('throttle_polarity').value



        self.get_logger().info(
            f'\nHue_low: {self.Hue_low}'
            f'\nHue_high: {self.Hue_high}'
            f'\nSaturation_low: {self.Saturation_low}'
            f'\nSaturation_high: {self.Saturation_high}'
            f'\nValue_low: {self.Value_low}'
            f'\nValue_high: {self.Value_high}'
            f'\ngray_lower: {self.gray_lower}'
            f'\ninverted_filter: {self.inverted_filter}'
            f'\nnumber_of_lines: {self.number_of_lines}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nmin_width: {self.min_width}'
            f'\nmax_width: {self.max_width}')
        
        # setting default values for actuators
        self.zero_throttle = 0.0
        self.zero_error_throttle = 0.0
        self.error_throttle = 0.0
        self.max_rpm = 1000
        self.steering_polarity = int(1)
        self.throttle_polarity = int(1)

        try:
            # Set trackbars to previously saved config
            cv2.setTrackbarPos('lowH', MASK_WINDOW_NAME, self.Hue_low)
            cv2.setTrackbarPos('highH', MASK_WINDOW_NAME, self.Hue_high)
            cv2.setTrackbarPos('lowS', MASK_WINDOW_NAME, self.Saturation_low)
            cv2.setTrackbarPos('highS', MASK_WINDOW_NAME, self.Saturation_high)
            cv2.setTrackbarPos('lowV', MASK_WINDOW_NAME, self.Value_low)
            cv2.setTrackbarPos('highV', MASK_WINDOW_NAME, self.Value_high)
            cv2.setTrackbarPos('gray_lower', BW_WINDOW_NAME, self.gray_lower)
            cv2.setTrackbarPos('Inverted_filter', BW_WINDOW_NAME, self.inverted_filter)
            cv2.setTrackbarPos('min_width', IMG_WINDOW_NAME, self.min_width)
            cv2.setTrackbarPos('max_width', IMG_WINDOW_NAME, self.max_width)
            cv2.setTrackbarPos('number_of_lines', IMG_WINDOW_NAME, self.number_of_lines)
            cv2.setTrackbarPos('error_threshold', IMG_WINDOW_NAME, int(self.error_threshold*100))

            ## To do:
            # self.camera_start_height = self.get_parameter('camera_start_height').value
            # self.camera_bottom_height = self.get_parameter('camera_bottom_height').value
            # self.camera_left_width = self.get_parameter('camera_left_width').value
            # self.camera_right_width = self.get_parameter('camera_right_width').value

            cv2.setTrackbarPos('Steering_sensitivity', THR_STR_WINDOW_NAME, self.Steering_sensitivity)
            cv2.setTrackbarPos('max_rpm', THR_STR_WINDOW_NAME, self.max_rpm)
            cv2.setTrackbarPos('steering_polarity', THR_STR_WINDOW_NAME, self.steering_polarity)
            cv2.setTrackbarPos('throttle_polarity', THR_STR_WINDOW_NAME, self.throttle_polarity)
        except TypeError:
            pass


    def live_calibration_values(self, data):
        
        # get trackbar positions
        self.Hue_low = cv2.getTrackbarPos('lowH', MASK_WINDOW_NAME)
        self.Hue_high = cv2.getTrackbarPos('highH', MASK_WINDOW_NAME)
        self.Saturation_low = cv2.getTrackbarPos('lowS', MASK_WINDOW_NAME)
        self.Saturation_high = cv2.getTrackbarPos('highS', MASK_WINDOW_NAME)
        self.Value_low = cv2.getTrackbarPos('lowV', MASK_WINDOW_NAME)
        self.Value_high = cv2.getTrackbarPos('highV', MASK_WINDOW_NAME)
        self.gray_lower = cv2.getTrackbarPos('gray_lower', BW_WINDOW_NAME)
        self.inverted_filter = cv2.getTrackbarPos('Inverted_filter', BW_WINDOW_NAME)
        self.min_width = cv2.getTrackbarPos('min_width', IMG_WINDOW_NAME)
        self.max_width = cv2.getTrackbarPos('max_width', IMG_WINDOW_NAME)
        self.number_of_lines = cv2.getTrackbarPos('number_of_lines', IMG_WINDOW_NAME)
        self.error_threshold = float(cv2.getTrackbarPos('error_threshold', IMG_WINDOW_NAME)/100)
        crop_width_percent = cv2.getTrackbarPos('frame_width', IMG_WINDOW_NAME)
        rows_to_watch_percent = cv2.getTrackbarPos('rows_to_watch', IMG_WINDOW_NAME)
        rows_offset_percent = cv2.getTrackbarPos('rows_offset', IMG_WINDOW_NAME)
        steer_input = cv2.getTrackbarPos('Steering_value', THR_STR_WINDOW_NAME)
        Steering_sensitivity = float(cv2.getTrackbarPos('Steering_sensitivity', THR_STR_WINDOW_NAME)/100)
        Throttle_mode = cv2.getTrackbarPos('Throttle_mode', THR_STR_WINDOW_NAME)
        throttle_input = cv2.getTrackbarPos('Throttle_value', THR_STR_WINDOW_NAME)

        # Motor parameters
        max_rpm_test = self.get_parameter('max_rpm').value
        # self.get_logger().info(f'\nmax_rpm_test: {max_rpm_test}')

        self.max_rpm = int(cv2.getTrackbarPos('max_rpm', THR_STR_WINDOW_NAME))
        steering_pol_slider = int(cv2.getTrackbarPos('steering_polarity', THR_STR_WINDOW_NAME))
        throttle_pol_slider = int(cv2.getTrackbarPos('throttle_polarity', THR_STR_WINDOW_NAME))
        if steering_pol_slider == 0:
            self.steering_polarity = int(-1)
        else:
            self.steering_polarity = int(1)
        if throttle_pol_slider == 0:
            self.throttle_polarity = int(-1)
        else:
            self.throttle_polarity = int(1)
        max_rpm_param = rclpy.parameter.Parameter('max_rpm', Parameter.Type.INTEGER, self.max_rpm)
        steering_pol_param = rclpy.parameter.Parameter('steering_polarity', Parameter.Type.INTEGER, self.steering_polarity)
        throttle_pol_param = rclpy.parameter.Parameter('throttle_polarity', Parameter.Type.INTEGER, self.throttle_polarity)
        self.set_parameters([max_rpm_param,steering_pol_param,throttle_pol_param])
        
        # Setting throttle and steering values
        if Throttle_mode == 0:
            self.zero_throttle = slider_to_normalized(throttle_input)
        elif Throttle_mode == 1:
            self.zero_error_throttle = slider_to_normalized(throttle_input)
        elif Throttle_mode == 2:
            self.error_throttle = slider_to_normalized(throttle_input)

        self.twist_cmd.angular.z = Steering_sensitivity*slider_to_normalized(steer_input)
        self.twist_cmd.linear.x = slider_to_normalized(throttle_input)
        self.twist_publisher.publish(self.twist_cmd)

        # Setting lower constraints on camera values
        if crop_width_percent < 1:
            crop_width_percent = 1
        if rows_to_watch_percent < 1:
            rows_to_watch_percent = 1
        if rows_offset_percent < 1:
            rows_offset_percent = 1

        # Image processing from slider values
        frame = self.bridge.imgmsg_to_cv2(data)
        height, width, channels = frame.shape

        rows_to_watch_decimal = rows_to_watch_percent / 100
        rows_offset_decimal = rows_offset_percent / 100
        crop_width_decimal = crop_width_percent / 100
        rows_to_watch = int(height * rows_to_watch_decimal)
        rows_offset = int(height * (1 - rows_offset_decimal))

        start_height = int(height - rows_offset)
        bottom_height = int(start_height + rows_to_watch)
        left_width = int((width / 2) * (1 - crop_width_decimal))
        right_width = int((width / 2) * (1 + crop_width_decimal))

        img = frame[start_height:bottom_height, left_width:right_width]
        image_width = right_width-left_width
        image_height = bottom_height-start_height

        # changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([self.Hue_low, self.Saturation_low, self.Value_low])
        higher = np.array([self.Hue_high, self.Saturation_high, self.Value_high])
        mask = cv2.inRange(hsv, lower, higher)

        if self.inverted_filter == 1:
            bitwise_mask = cv2.bitwise_and(img, img, mask=cv2.bitwise_not(mask))
        else:
            bitwise_mask = cv2.bitwise_and(img, img, mask=mask)

        # changing to gray color space
        gray = cv2.cvtColor(bitwise_mask, cv2.COLOR_BGR2GRAY)

        # changing to black and white color space
        gray_upper = 255
        (dummy, blackAndWhiteImage) = cv2.threshold(gray, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        centers = []
        cx_list = []
        cy_list = []

        # Creating points to be drawn on image 
        start_point = (int(image_width/2),0)
        end_point = (int(image_width/2),int(bottom_height))

        start_point_thresh_pos_x = int((image_width/2)*(1-self.error_threshold))
        start_point_thresh_neg_x = int((image_width/2)*(1+self.error_threshold))
        
        start_point_thresh_pos = (start_point_thresh_pos_x,0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x,0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(bottom_height))

        # plotting contours and their centroids
        for contour in contours[:self.number_of_lines]:
            x, y, w, h = cv2.boundingRect(contour)
            if self.min_width < w < self.max_width:
                try:
                    x, y, w, h = cv2.boundingRect(contour)
                    img = cv2.drawContours(img, contour, -1, (0, 255, 0), 3)
                    m = cv2.moments(contour)
                    cx = int(m['m10'] / m['m00'])
                    cy = int(m['m01'] / m['m00'])
                    centers.append([cx, cy])
                    cx_list.append(cx)
                    cy_list.append(cy)
                    cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
                    img = cv2.line(img, start_point, end_point, (0,255,0), 4)
                    img = cv2.line(img, start_point_thresh_pos, end_point_thresh_pos, (0,0,255), 2)
                    img = cv2.line(img, start_point_thresh_neg, end_point_thresh_neg, (0,0,255), 2)
                except ZeroDivisionError:
                    pass
        try:
            if len(cx_list) > 1:
                error_list = []
                count = 0
                for cx_pos in cx_list:
                    error = float(((image_width/2) - cx_pos) / (image_width/2))
                    error_list.append(error)
                avg_error = (sum(error_list) / float(len(error_list)))
                p_horizon_diff = error_list[0] - error_list[-1]
                if abs(p_horizon_diff) <=self.error_threshold:
                    error_x = avg_error
                    pixel_error = int((image_width/2)*(1-error_x))
                    mid_x, mid_y = pixel_error, int((image_height/2))
                    self.get_logger().info(f"straight curve: {error_x}, {error_list}")
                else: 
                    for error in error_list:
                        if abs(error) < self.error_threshold:
                            error = 1
                            error_list[count] = error
                        count+=1
                    error_x = min(error_list, key=abs)
                    error_x_index = error_list.index(min(error_list, key=abs))
                    mid_x, mid_y = cx_list[error_x_index], cy_list[error_x_index]
                    self.get_logger().info(f"curvy road: {error_x}, {error_list}")

                cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
                start_point_error = (int(image_width/2), mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                centers = []
                cx_list = []
                cy_list = []
            elif len(cx_list) == 1:
                mid_x, mid_y = cx_list[0], cy_list[0]
                self.get_logger().info(f"only detected one line")
                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)
            else:
                pass
        except ValueError:
            pass

        # plotting results
        cv2.imshow(IMG_WINDOW_NAME, img)
        cv2.imshow(MASK_WINDOW_NAME, mask)
        cv2.imshow(BW_WINDOW_NAME, blackAndWhiteImage)
        cv2.waitKey(1)


        # Write files to yaml file for storage
        # color_config_path = str(os.path.dirname(__file__) + '/../config/ros_racer_calibration.yaml')
        color_config_path = str('/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/config/ros_racer_calibration.yaml')
        f = open(color_config_path, "w")
        f.write(
            f"{LD_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    Hue_low : {self.Hue_low} \n"
            f"    Hue_high : {self.Hue_high} \n"
            f"    Saturation_low : {self.Saturation_low} \n"
            f"    Saturation_high : {self.Saturation_high} \n"
            f"    Value_low : {self.Value_low} \n"
            f"    Value_high : {self.Value_high} \n"
            f"    number_of_lines : {self.number_of_lines} \n"
            f"    error_threshold : {self.error_threshold} \n"
            f"    Width_min : {self.min_width} \n"
            f"    Width_max : {self.max_width} \n"
            f"    gray_lower : {self.gray_lower} \n"
            f"    inverted_filter : {self.inverted_filter} \n"
            f"    camera_start_height : {start_height} \n"
            f"    camera_bottom_height : {bottom_height} \n"
            f"    camera_left_width : {left_width} \n"
            f"    camera_right_width : {right_width} \n"
            f"{CALIBRATION_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    Hue_low : {self.Hue_low} \n"
            f"    Hue_high : {self.Hue_high} \n"
            f"    Saturation_low : {self.Saturation_low} \n"
            f"    Saturation_high : {self.Saturation_high} \n"
            f"    Value_low : {self.Value_low} \n"
            f"    Value_high : {self.Value_high} \n"
            f"    number_of_lines : {self.number_of_lines} \n"
            f"    error_threshold : {self.error_threshold} \n"
            f"    Width_min : {self.min_width} \n"
            f"    Width_max : {self.max_width} \n"
            f"    gray_lower : {self.gray_lower} \n"
            f"    inverted_filter : {self.inverted_filter} \n"
            f"{LG_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    steering_sensitivity : {Steering_sensitivity} \n"
            f"    zero_throttle  : {self.zero_throttle } \n"
            f"    zero_error_throttle : {self.zero_error_throttle} \n"
            f"    error_throttle : {self.error_throttle} \n"
            f"    error_threshold : {self.error_threshold} \n"
            f"{VESC_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    max_rpm : {self.max_rpm} \n"
            f"    steering_polarity : {self.steering_polarity} \n"
            f"    throttle_polarity : {self.throttle_polarity} \n"
            f"{ADA_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    steering_polarity : {self.steering_polarity} \n"
            f"    throttle_polarity : {self.throttle_polarity} \n"
        )
        f.close()



def main(args=None):
    rclpy.init(args=args)
    CAL_publisher = Calibration()
    try:
        rclpy.spin(CAL_publisher)
        CAL_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print(f"\nShutting down {CALIBRATION_NODE_NAME}...")

        # Stop the car
        CAL_publisher.twist_cmd.linear.x = CAL_publisher.zero_throttle
        CAL_publisher.twist_publisher.publish(CAL_publisher.twist_cmd)

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        CAL_publisher.destroy_node()
        rclpy.shutdown()
        print(f"{CALIBRATION_NODE_NAME} shut down successfully.")


if __name__ == '__main__':
    main()
