# ucsd_robocar_lane_detection2_pkg 

<img src="ucsd_ros2_logos.png">

<div>

## Table of Contents
  - [**Config**](#config)
    - [ros_racer_calibration](#ros_racer_calibration)
  - [**Nodes**](#nodes)
    - [lane_detection_node](#lane_detection_node)
    - [lane_guidance_node](#lane_guidance_node)
    - [calibration_node](#calibration_node)
  - [**Topics**](#topics)
  - [**Launch**](#launch)
    - [camera navigation calibration](#camera-navigation-calibration-launch)
    - [camera navigation](#camera_nav_launch_py)
  - [**Tools**](#tools)
    - [ROS1&2 Guide Books](#ros12-guide-books)
    - [Calibration Tutorial](#calibration-tutorial)
  - [**Troubleshooting**](#troubleshooting)
    - [Camera not working](#camera-not-working)
    - [Throttle and steering not working](#throttle-and-steering-not-working)
  - [**Demonstration videos**](#demonstration-videos)
    - [Lane detection example with yellow filter](#lane-detection-example-with-yellow-filter)
    - [Blue color detection](#blue-color-detection)
    - [Yellow color detection and line width specification](#yellow-color-detection-and-line-width-specification)
    - [Throttle and steering](#throttle-and-steering)
    - [Manipulating image dimensions](#manipulating-image-dimensions)

<div align="center">

## Config

</div>

### **ros_racer_calibration**

Associated file: **ros_racer_calibration.yaml**


<div align="center">

## Nodes

</div>


### **lane_detection_node**

Associated file: **lane_detection.py**

Associated Topics:
- Subscribes to the **camera** [**Topics**](#topics)
- Publishes to the **centroid** [**Topics**](#topics)

This node subscribes from the [**camera**](#camera) topic and uses opencv to identify line
information from the image, and publish the information of the lines centroid to the [**centroid**](#centroid). 

The color scheme is defined as follows:

- 2 contours : green bounding boxes and a blue average centroid
- 1 contour : green bounding box with a single red centroid

Below show the image post processing techniques, cv2 methods and the logic applied respectively.

<div align="center">
  <img src="filtering_process.png">
  <img src="applying_methods.png">
  <img src="applying_logic.png">
</div>

### **lane_guidance_node**

Associated file: **lane_guidance.py**

Associated Topics:
- Subscribes to the **centroid** [**Topics**](#topics)
- Publishes to the **cmd_vel** [**Topics**](#topics)

This node subscribes to the centroid topic, calculates the throttle and steering
based on the centroid value, and then publish them to their corresponding topics.
Throttle is based on error threshold specified in the [**camera navigation calibration**](#camera-navigation-calibration-launch)

| Error Threshold State| Throttle Response |
| ------ | ------ |
| error < error_threshold | car goes **faster** |
| error > error_threshold | car goes **slower** |

Steering is based on a proportional controller and the error threshold by calculating the error between the centroid (found in [**lane_detection_node**](#lane_detection_node)) and the heading of the car. 

| Error Threshold State| Steering Response |
| ------ | ------ |
| error < error_threshold | car steers **straight** |
| error > error_threshold | car steers **toward error** |

### **calibration_node**

Associated file: **calibration_node.py**

Associated Topics:
- Subscribes to the **centroid** [**Topics**](#topics)
- Publishes to the **cmd_vel** [**Topics**](#topics)


**These values are saved automatically to a configuration file, so just press** `control-c` **when the car is calibrated.**

Calibrate the camera, throttle and steering in this node by using the sliders to find:
- the right color filter 
- desired image dimmensions
- throttle values for both the optimal condtion (error = 0) and the non optimal condtion (error !=0) AKA go fast when error=0 and go slow if error !=0
- steering sensitivty change the Kp value to adjust the steering sensitivty. A value too high or low can make the car go unstable (oscillations in the cars trajectory)

| Kp value | Steering Response |
| ------ | ------ |
| as Kp --> 1 | steering more responsive |
| as Kp --> 0 | steering less responsive |


| Property   | Info |
| ----------  | --------------------- |
| lowH, highH | Setting low and high values for Hue  | 
| lowS, highS | Setting low and high values for Saturation | 
| lowV, highV | Setting low and high values for Value | 
| Inverted_filter | Specify to create an inverted color tracker | 
| min_width, max_width | Specify the width range of the line to be detected  | 
| number_of_lines | Specify the number of lines to be detected  | 
| error_threshold | Specify the acceptable error the robot will consider as approximately "no error" | 
| frame_width | Specify the width of image frame (horizontal cropping) | 
| rows_to_watch | Specify the number of rows (in pixels) to watch (vertical cropping) | 
| rows_offset | Specify the offset of the rows to watch (vertical pan) | 
| Steering_sensitivity | Specify the proportional gain of the steering | 
| Steering_value | Specify the steering value | 
| Throttle_mode | Toggle this slider at the end of calibration to the following 3 modes. |
| Throttle_mode 0 | zero_throttle_mode (find value where car does not move) 
| Throttle_mode 1 | zero_error_throttle_mode (find value for car to move when there is **no error** in steering)
| Throttle_mode 2 | error_throttle_mode(find value for car to move when there is **some error** in steering)| 
| Throttle_value | Specify the throttle value to be set in each of the throttle modes| 

More morphological transfromations and examples can be found <a href="https://docs.opencv.org/3.4/db/df6/tutorial_erosion_dilatation.html" >here</a> and <a href="https://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html" >here</a>

<div align="center">

## Topics

</div>

| Nodes | Subscribed Topics | Published Topics |
| ------ | ------ | ------ |
| lane_detection_node | /camera/color/image_raw | /centroid |
| lane_guidance_node  | /centroid               | /cmd_vel |
| calibration_node    | /camera/color/image_raw | /cmd_vel  |

<div align="center">

## Launch

</div>

#### **camera navigation calibration**

Associated file: **ros_racer_calibration_launch.launch**

This file will launch the [**calibration_node**](#calibration_node) 

`ros2 launch ucsd_robocar_navigation_pkg camera_nav_calibration.launch.py`

#### **camera navigation**

Associated file: **ros_racer_calibration_launch.launch**

This file will launch both [**lane_detection_node**](#lane_detection_node), [**lane_guidance_node**](#lane_guidance_node) and load the parameters determined using the [calibration_node](#calibration_node)

**Before launching, please calibrate the robot first while on the stand!**

`ros2 launch ucsd_robocar_navigation_pkg camera_nav.launch.py`

<div align="center">

## Tools 

</div>

#### ROS1&2 Guide Books

For help with using ROS in the terminal and in console scripts, check out these google doc below to see tables of ROS commands and plenty of examples of using ROS in console scripts.

- <a href="https://docs.google.com/document/d/1DJgVLnu_vN-IXKD3QrQVF3W-JC6RiQPVugHeFAioB58/edit?usp=sharing" >ROS2 Guide Book</a>

#### Calibration Tutorial

Check out this google doc <a href="https://docs.google.com/document/d/1YS5YGbo8evIo9Mlb0J-w2r3bZfju37Zl4UmdaN2CD2A/edit#heading=h.rxwmnnbbd9xn" >here</a> for a tutorial that will walk through the calibration process step by step and explain more about the sliders and their effects.


<div align="center">

## Troubleshooting

</div>

#### **Camera not working** 

See troubleshooting guide <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_sensor2_pkg#troubleshooting" >here</a>

#### **Throttle and steering not working** 

See troubleshooting guide <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_actuator2_pkg#troubleshooting" >here</a>

<div align="center">

## **Demonstration videos** 

</div>

<div align="center">

#### Lane detection example with yellow filter

[![lane detection example with yellow filter](https://j.gifs.com/6WRqXN.gif)](https://youtu.be/f4VrncQ7HU)

</div>

<div align="center">

#### Number of lines to detect
[![Number of lines to detect](https://j.gifs.com/qQ7Lvk.gif)](https://youtu.be/5AVL68BTD0U)

</div>

<div align="center">

#### Error threshold
[![Error threshold](https://j.gifs.com/k28Xmv.gif)](https://youtu.be/ied1TDvpDK4)

</div>

<div align="center">

#### Blue color detection

[![Blue color detection](https://j.gifs.com/PjZoj6.gif)](https://youtu.be/c9rkRHGGea0)

</div>

<div align="center">

#### Yellow color detection and line width specification

[![Yellow color detection and line width specification](https://j.gifs.com/BrLJro.gif)](https://youtu.be/l2Ngtd461DY)

</div>

<div align="center">

#### Throttle and steering

[![Throttle and steering](https://j.gifs.com/362n6p.gif)](https://youtu.be/lhYzs5v6jtI)

</div>

<div align="center">

#### Manipulating image dimensions

[![Manipulating image dimensions](https://j.gifs.com/lR5oR1.gif)](https://youtu.be/_DhG6dduYPs)

</div>

