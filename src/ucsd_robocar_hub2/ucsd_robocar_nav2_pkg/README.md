# ucsd_robocar_nav2_pkg 

<img src="ucsd_ros2_logos.png">

<div>

## Table of Contents

  - [**Enable X11 forwarding**](#7-enable-x11-forwarding)
  - [**Work Flow To Use This Repository**](#work-flow-to-use-this-repository)
  - [**Config**](#config)
    - [car_config](#car_config)
    - [pkg_locations_ucsd](#pkg_locations_ucsd)
    - [ros_racer_calibration](#ros_racer_calibration)
  - [**Navigation Packages**](#navigation-packages)
    - [ucsd_robocar_lane_detection2_pkg](#ucsd_robocar_lane_detection2_pkg)
    - [ucsd_robocar_sensor2_pkg](#ucsd_robocar_sensor2_pkg)
    - [ucsd_robocar_actuator2_pkg](#ucsd_robocar_actuator2_pkg)
  - [**Launch**](#launch)
    - [all components](#all_components_launch_py)
    - [camera navigation calibration](#camera-navigation-calibration)
    - [camera navigation](#camera-navigation)
  - [**Tools**](#tools)
    - [ROS2 Guide Book](#ros2-guide-book)
  - [**Troubleshooting**](#troubleshooting)

## **Enable X11 forwarding**

Associated file: **x11_forwarding_steps.txt**

Some jetsons may not have this enabled, so if needed please read the steps <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_nav2_pkg/-/blob/master/x11_forwarding_steps.txt" >here</a> to setup X11 forwarding

<div align="center">

## **Work Flow To Use This Repository**

</div>

1. Pull docker image <a href="https://hub.docker.com/r/djnighti/ucsd_robocar" >**here**</a> for this repo. The docker image contains all dependecies for plug-n-play use and also provides neccessary instructions for how to run the docker container.
2. Make sure that the [**car_config**](#car_config) has been updated to reflect your specific cars configuration of sensors and actuators. Once this updated, make sure to recompile the packages in the ros2_ws.
3. Calibrate the camera, throttle and steering values using the [**calibration_node**](#calibration_node)

`ros2 launch ucsd_robocar_nav2_pkg camera_nav_calibration.launch.py`

4. Launch [**camera navigation**](#camera_nav_launch_py)

`ros2 launch ucsd_robocar_nav2_pkg camera_nav.launch.py`

5. Tune parameters in the calibration until desired behavior is achieved


<div align="center">

## Config

</div>

### **car_config**

- Associated package: **ucsd_robocar_nav2_pkg** (This Package)
- Associated file: **car_config.yaml**

This file contains a list of all the components (sensors/actuators) available for ucsd_robocar framework and acts as a "switch" (1:true, 0:false) to activate the corresponding nodes in this navigation package.


ex. **car_config.yaml**
```
sick: 1
livox: 0
bpearl: 0
rp_lidar: 0
ld06: 0
webcam: 1
intelD455: 0
oakd: 0
zed: 0
artemis: 0
ublox: 1
vesc: 1
adafruit: 0
esp32: 0
stm32: 0
bldc_sensor: 1
bldc_no_sensor: 0
```

This config corresponds to the following nodes being activated:
- sick
- webcam
- ublox
- vesc
- bldc_sensor

### **pkg_locations_ucsd**

- Associated package: **ucsd_robocar_nav2_pkg** (This Package)
- Associated file: **pkg_locations_ucsd.yaml**

This file contains a list of all corresponding packages, launch files, sensor type, and desired published topics for all the components (sensors/actuators) available for ucsd_robocar framework. The only parameter that should be changed (if needed) is the ['topics_published'] arguement to suit the needs of the specific problem.

This file is formatted the way it is because it is used to create a dynmically built launch file (more details in the [**all components**](#all_components_launch_py) launch file).

ex. **pkg_locations_ucsd.yaml**
```
# 
# param: ['package', 'launch_file', 'sensor_type', 'topics_published']
#

# Lidars
sick : ['ucsd_robocar_sensor2_pkg', 'lidar_sicktim.launch.py','lidar', '/scan']
livox : ['ucsd_robocar_sensor2_pkg','lidar_livox.launch.py','lidar', '/scan']
bpearl : ['ucsd_robocar_sensor2_pkg','lidar_bpearl.launch.py','lidar', '/scan']
rp_lidar : ['ucsd_robocar_sensor2_pkg', 'lidar_rp.launch.py','lidar', '/scan']
ld06 : ['ucsd_robocar_sensor2_pkg','lidar_ld06.launch.py','lidar', '/scan']

# IMU
artemis: ['ucsd_robocar_sensor2_pkg', 'razor-pub.launch', 'imu', '/imu']

# GPS
ublox: ['ucsd_robocar_sensor2_pkg', 'ublox.launch.py', 'gps','/gps_topic_name']

# Cameras
webcam: ['ucsd_robocar_sensor2_pkg', 'camera_webcam.launch.py', 'camera', '/camera/color/image_raw']
intel455: ['ucsd_robocar_sensor2_pkg', 'camera_intel455.launch.py', 'camera', '/camera/color/image_raw']

# Actuators
vesc_steering: ['ucsd_robocar_actuator2_pkg', 'vesc_steering.launch.py','motor', '/steering']
vesc_throttle: ['ucsd_robocar_actuator2_pkg', 'vesc_throttle.launch.py','motor', '/throttle']
adafruit_steering: ['ucsd_robocar_actuator2_pkg', 'adafruit_steering.launch.py','motor', '/steering']
adafruit_throttle: ['ucsd_robocar_actuator2_pkg', 'adafruit_throttle.launch.py','motor', '/throttle']
esp32: ['ucsd_robocar_actuator2_pkg', 'esp32.launch.py','motor', '/esp32/throttle']
stm32: ['ucsd_robocar_actuator2_pkg', 'stm32.launch.py','motor', '/stm32/throttle']

```

### **ros_racer_calibration**

- Associated package: [**ucsd_robocar_lane_detection2_pkg**](#ucsd_robocar_lane_detection2_pkg)
- Associated file: **ros_racer_calibration.yaml**

See details <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_lane_detection2_pkg/-/tree/master#ros_racer_calibration" >**here**</a> that explain this config file.

<div align="center">

## Navigation Packages

</div>

### ucsd_robocar_lane_detection2_pkg

All the details for this package about all the nodes, topics and launch files are exaplained in **explcit** detail <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_lane_detection2_pkg" >**here**</a> from its official git repo. **HIGHLY RECOMMENDED TO GO THROUGH THE README TO UNDERSTAND HOW TO USE IT.**

| Nodes | Subscribed Topics | Published Topics |
| ------ | ------ | ------ |
| lane_detection_node | /camera/color/image_raw | /centroid |
| lane_guidance_node  | /centroid               | /cmd_vel |
| calibration_node    | /camera/color/image_raw | /cmd_vel  |

### ucsd_robocar_sensor2_pkg

All the details for this package about all the nodes, topics and launch files are exaplained in **explcit** detail <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_sensor2_pkg" >**here**</a> from its official git repo. **HIGHLY RECOMMENDED TO GO THROUGH THE README TO UNDERSTAND HOW TO USE IT.**

| Nodes |  Msg Type | Published Topics |
| ------ | ------ | ------ |
| webcam_node           | sensor_msgs.msg.Image       | /camera/color/image_raw |
| realsense_ros2_camera | sensor_msgs.msg.Image       | /camera/color/image_raw |
| ldlidar               | sensor_msgs.msg.LaserScan   | /scan |
| rplidar_composition   | sensor_msgs.msg.LaserScan   | /scan |
| sick_generic_caller   | sensor_msgs.msg.LaserScan   | /scan |

### ucsd_robocar_actuator2_pkg

All the details for this package about all the nodes, topics and launch files are exaplained in **explcit** detail <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_actuator2_pkg" >**here**</a> from its official git repo. **HIGHLY RECOMMENDED TO GO THROUGH THE README TO UNDERSTAND HOW TO USE IT.**

| Nodes |  Msg Type | Subscribed Topics | info |
| ------ | ------ | ------ | ------ |
| adafruit_steering_node | std_msgs.msg.Float32 | /steering | value range: [-1,1] |
| adafruit_throttle_node | std_msgs.msg.Float32 | /throttle | value range: [-1,1] |
| vesc_steering_node     | std_msgs.msg.Float32 | /steering | value range: [-1,1] |
| vesc_rpm_node          | std_msgs.msg.Float32 | /throttle | value range: [-1,1] |
| adafruit_twist_node    | geometry_msgs.msg.Twist | /cmd_vel | linear.x (forwards/backwards) angular.z (steering) ranges: [-1,1] |
| vesc_twist_node        | geometry_msgs.msg.Twist | /cmd_vel | linear.x (forwards/backwards) angular.z (steering) ranges: [-1,1] |

<div align="center">

## Launch

</div>

#### **all components**

- Associated package: **ucsd_robocar_nav2_pkg** (This Package)
- Associated file: **all_components.launch.py**
- Associated nodes: **nodes for all sensors and actuators** (which are variables set in [**car_config**](#car_config))

This file will launch all the sensors and actuators specified as **active** in [**car_config**](#car_config)

`ros2 launch ucsd_robocar_nav2_pkg all_components.launch.py`

#### **camera navigation calibration**

- Associated package: [**ucsd_robocar_lane_detection2_pkg**](#ucsd_robocar_lane_detection2_pkg)
- Associated file: **camera_nav_calibration.launch.py**
- Associated nodes: **lane_detection_node** and **nodes for camera sensor and actuators** (which are variables set in [**car_config**](#car_config))

This file will launch the **calibration_node** from [**ucsd_robocar_lane_detection2_pkg**](#ucsd_robocar_lane_detection2_pkg) as well as [**all components**](#all_components_launch_py) from this package (ucsd_robocar_nav2_pkg). 

`ros2 launch ucsd_robocar_nav2_pkg camera_nav_calibration.launch.py`

#### **camera navigation**

- Associated package: [**ucsd_robocar_lane_detection2_pkg**](#ucsd_robocar_lane_detection2_pkg)
- Associated file: **camera_nav.launch.py**
- Associated nodes: **lane_detection_node**, **lane_guidance_node** and **nodes for camera sensor and actuators** (which are variables set in [**car_config**](#car_config))

This file will launch both the **lane_detection_node** and **lane_guidance_node** and load the parameters determined using the **calibration_node** from [**ucsd_robocar_lane_detection2_pkg**](#ucsd_robocar_lane_detection2_pkg) and launch [**all components**](#all_components_launch_py)


**Before launching, please calibrate the robot first while on the stand!**

`ros2 launch ucsd_robocar_nav2_pkg camera_nav.launch.py`

<div align="center">

## Tools 

</div>

#### ROS2 Guide Book

For help with using ROS in the terminal and in console scripts, check out this google doc below to see tables of ROS commands and plenty of examples of using ROS in console scripts.

<a href="https://docs.google.com/document/d/1DJgVLnu_vN-IXKD3QrQVF3W-JC6RiQPVugHeFAioB58/edit?usp=sharing" >**ROS2 Guide Book**</a>


<div align="center">

## Troubleshooting

</div>

[**ucsd_robocar_lane_detection2_pkg**](#ucsd_robocar_lane_detection2_pkg) : see troubleshooting guide <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_lane_detection2_pkg/-/tree/master#troubleshooting" >here</a>

[**ucsd_robocar_sensor2_pkg**](#ucsd-robocar-sensor2-pkg) : see troubleshooting guide <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_sensor2_pkg#troubleshooting" >here</a>

[**ucsd_robocar_actuator2_pkg**](#ucsd-robocar-actuator2-pkg) : see troubleshooting guide <a href="https://gitlab.com/ucsd_robocar2/ucsd_robocar_actuator2_pkg#troubleshooting" >here</a>


<div align="center">
