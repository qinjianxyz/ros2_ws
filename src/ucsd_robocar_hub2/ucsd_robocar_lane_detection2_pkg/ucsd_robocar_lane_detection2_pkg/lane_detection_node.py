import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Int32MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

# Nodes in this program
NODE_NAME = 'lane_detection_node'

# Topics subcribed/published to in this program
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
CENTROID_TOPIC_NAME = '/centroid'


class LaneDetection(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.centroid_error_publisher = self.create_publisher(Float32, CENTROID_TOPIC_NAME, 10)
        self.centroid_error_publisher
        self.centroid_error = Float32()
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.locate_centroid, 10)
        self.camera_subscriber
        self.bridge = CvBridge()
        self.max_num_lines_detected = 10
        self.image_width = 0
        self.error_threshold = 0.1
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Hue_low', 1),
                ('Hue_high', 1),
                ('Saturation_low', 1),
                ('Saturation_high', 1),
                ('Value_low', 1),
                ('Value_high', 1),
                ('gray_lower', 1),
                ('inverted_filter', 0),
                ('number_of_lines', 0),
                ('error_threshold', 0),
                ('Width_min', 1),
                ('Width_max', 1),
                ('camera_start_height', 1),
                ('camera_bottom_height', 1),
                ('camera_left_width', 1),
                ('camera_right_width', 1),
                ('debug_cv', 0)
            ])
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
        self.start_height = self.get_parameter('camera_start_height').value
        self.bottom_height = self.get_parameter('camera_bottom_height').value
        self.left_width = self.get_parameter('camera_left_width').value
        self.right_width = self.get_parameter('camera_right_width').value
        self.debug_cv = self.get_parameter('debug_cv').value
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
            f'\nmax_width: {self.max_width}'
            f'\nstart_height: {self.start_height}'
            f'\nbottom_height: {self.bottom_height}'
            f'\nleft_width: {self.left_width}'
            f'\nright_width: {self.right_width}'
            f'\ndebug_cv: {self.debug_cv}')


    def locate_centroid(self, data):
        # Image processing from rosparams
        frame = self.bridge.imgmsg_to_cv2(data)

        self.image_width = int(self.right_width - self.left_width)

        img = frame[self.start_height:self.bottom_height, self.left_width:self.right_width]

        image_height = self.bottom_height-self.start_height

        # changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # setting threshold limits for white color filter
        lower = np.array([self.Hue_low, self.Saturation_low, self.Value_low])
        upper = np.array([self.Hue_high, self.Saturation_high, self.Value_high])
        mask = cv2.inRange(hsv, lower, upper)

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

        # Setting up data arrays
        centers = []
        cx_list = []
        cy_list = []

        # Defining points of a line to be drawn for visualizing error
        start_point = (int(self.image_width/2),0)
        end_point = (int(self.image_width/2),int(self.bottom_height))

        start_point_thresh_pos_x = int((self.image_width/2)*(1-self.error_threshold))
        start_point_thresh_neg_x = int((self.image_width/2)*(1+self.error_threshold))
        
        start_point_thresh_pos = (start_point_thresh_pos_x,0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(self.bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x,0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(self.bottom_height))

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
        # Further image processing to determine optimal steering value
        # simulating MPC with camera with a simple min error as cost function
        try:
            if len(cx_list) > 1:
                error_list = []
                count = 0
                for cx_pos in cx_list:
                    error = float(((self.image_width/2) - cx_pos) / (self.image_width/2))
                    error_list.append(error)
                avg_error = (sum(error_list) / float(len(error_list)))
                p_horizon_diff = error_list[0] - error_list[-1]
                if abs(p_horizon_diff) <= self.error_threshold:
                    error_x = avg_error
                    pixel_error = int((self.image_width/2)*(1-error_x))
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
                start_point_error = (int(self.image_width/2), mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                self.centroid_error.data = float(error_x)
                self.centroid_error_publisher.publish(self.centroid_error)
                centers = []
                cx_list = []
                cy_list = []
            elif len(cx_list) == 1:
                mid_x, mid_y = cx_list[0], cy_list[0]
                error_x = float(((self.image_width/2) - mid_x) / (self.image_width/2))
                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)
                self.centroid_error.data = error_x
                self.centroid_error_publisher.publish(self.centroid_error)
                self.get_logger().info(f"only detected one line")

            centers = []
            cx_list = []
            cy_list = []
            error_list = [0] * self.number_of_lines
        except ValueError:
            pass

        self.debug_cv = self.get_parameter('debug_cv').value # ability to update debug in real-time
        # plotting results
        '''
        if self.debug_cv:
            cv2.imshow('img', img)
            cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)
            cv2.waitKey(1)
        else:
            cv2.destroyAllWindows()
        '''


def main(args=None):
    rclpy.init(args=args)
    centroid_publisher = LaneDetection()
    try:
        rclpy.spin(centroid_publisher)
        centroid_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        centroid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        centroid_publisher.destroy_node()
        rclpy.shutdown()
        centroid_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
