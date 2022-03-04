# This is an auto generated Dockerfile for ros:desktop
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:foxy-ros-base-focal

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils
    librealsense2-utils \
    librealsense2-dev



ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO foxy

WORKDIR /

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop=0.9.2-1* \
    ros-foxy-ros1-bridge \
    ros-foxy-realsense2-camera \
    ros-foxy-realsense2-camera-msgs \
    ros-foxy-librealsense2 \ 
    ros-foxy-message-filters \
    ros-foxy-image-transport \
    ros-foxy-sick-scan2 \
    ros-foxy-teleop-twist-joy \
    ros-foxy-joy \
    ros-foxy-joy-teleop \
    ros-foxy-rviz-default-plugins \
    ros-foxy-rviz-rendering \
    && rm -rf /var/lib/apt/lists/*


# Not sure if these are needed...
#    ros-foxy-rviz2 \
#    ros-foxy-rviz-common \

#
# intel debian packages
#
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev

apt-get update && apt-get install -y --no-install-recommends libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev


#
# install ros1 packages
#
RUN apt-get update && apt-get install -y \
        ros-noetic-rosbash \
        ros-noetic-navigation \
        ros-noetic-sick-scan \ 
        ros-noetic-hector-slam \
        ros-noetic-scan-tools \
        ros-noetic-razor-imu-9dof \
        ros-noetic-driver-base \
        ros-noetic-map-server \
        ros-noetic-teleop-twist-joy \
        ros-noetic-joy \
        ros-noetic-joy-teleop \
        ros-noetic-realsense2-camera \
        ros-noetic-realsense2-description && \
    rm -rf /var/lib/apt/lists/*

#
# install useful packages
# 
WORKDIR /
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    nano \
    iputils-ping \
    x11-apps \
    nautilus \
    firefox \
    git-all \
    cheese \
    jstest-gtk \
    joystick


#
# Create sensor, actuator and nav workspaces
#
WORKDIR /
# ros2 ws
RUN mkdir -p /home/projects/ros2_ws/src
WORKDIR /
# ros1 ws
RUN mkdir -p /home/projects/ros1_ws/src
# f110_ws
RUN mkdir -p /home/projects/f110_ws/src
# Sensor pkg 
RUN mkdir -p /home/projects/sensor1_ws/src
RUN mkdir -p /home/projects/sensor2_ws/src

##### ROS1 Sensors #####
# Lidar inventory
RUN mkdir -p /home/projects/sensor1_ws/src/lidars/rplidar/ros1/src
RUN mkdir -p /home/projects/sensor1_ws/src/lidars/ld06/ros1/src

# Camera inventory

# IMU inventory

# GPS inventory

##### ROS2 Sensors #####
# Lidar inventory
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/bpearl/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/rplidar/ros2/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/livox/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/sicktim/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/ld06/ros2/src

# Camera inventory
RUN mkdir -p /home/projects/sensor2_ws/src/cameras/zed/src
RUN mkdir -p /home/projects/sensor2_ws/src/cameras/intelD455/src
RUN mkdir -p /home/projects/sensor2_ws/src/cameras/intelSDK/src
RUN mkdir -p /home/projects/sensor2_ws/src/cameras/oakd/src

# IMU inventory

# GPS inventory
RUN mkdir -p /home/projects/sensor2_ws/src/gps/ublox/src



#
# Get RS Bpearl --ROS2
#
WORKDIR /home/projects/sensor2_ws/src/lidars/bpearl/src
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git && \
    cd rslidar_sdk && \
    git submodule init && \
    git submodule update

###
# TODO: FIX CONFIG FILES TO COMPILE WITH COLCON
###

#WORKDIR /home/projects/sensor2_ws/lidars/rslidar
#RUN [\
#    "/bin/bash", \
#    "-c", \
#    "source /opt/ros/foxy/setup.bash && \
#    colcon build && \
#    source install/setup.bash"\
#    ]


#
# Get RPLidar --ROS2
#
WORKDIR /home/projects/sensor2_ws/src/lidars/rplidar/ros2/src
RUN git clone https://github.com/youngday/rplidar_ros2.git
WORKDIR /home/projects/sensor2_ws/src/lidars/rplidar
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    colcon build && \
    source install/setup.bash"\
    ]

#
# Get RPLidar --ROS1
#
WORKDIR /home/projects/sensor1_ws/src/lidars/rplidar/ros1/src
RUN git clone https://github.com/Slamtec/rplidar_ros.git
WORKDIR /home/projects/sensor1_ws/src/lidars/rplidar
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/noetic/setup.bash && \
    catkin_make && \
    source devel/setup.bash"\
    ]


#
# Get livox --ROS2
#
WORKDIR /home/projects/sensor2_ws/src/lidars/livox/src
RUN git clone https://github.com/Livox-SDK/livox_ros2_driver.git
WORKDIR /home/projects/sensor2_ws/src/lidars/livox
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    colcon build && \
    source install/setup.bash"\
    ]


#
# Get SickTim5xx --ROS2
#
WORKDIR /home/projects/sensor2_ws/src/lidars/sicktim/src
RUN git clone https://github.com/SICKAG/sick_scan2.git
WORKDIR /home/projects/sensor2_ws/src/lidars/sicktim
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    colcon build && \
    source install/setup.bash"\
    ]



#
# Get LD06 lidar --ROS2
#
WORKDIR /home/projects/sensor2_ws/src/lidars/ld06/ros2/src
RUN git clone https://github.com/linorobot/ldlidar.git
WORKDIR /home/projects/sensor2_ws/src/lidars/ld06/ros2
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]

#
# Get LD06 lidar --ROS1
#
WORKDIR /home/projects/sensor1_ws/src/lidars/ld06/ros1/src
RUN git clone https://github.com/AlessioMorale/ld06_lidar.git
WORKDIR /home/projects/sensor1_ws/src/lidars/ld06/ros1
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/noetic/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    catkin_make && \
    source devel/setup.bash"\
    ]


#
# Get Intel SDK
#
WORKDIR /home/projects/sensor2_ws/src/cameras/intelSDK/src
RUN git clone -b ros2debian https://github.com/IntelRealSense/librealsense
#WORKDIR /home/projects/sensor2_ws/src/cameras/intelSDK
#RUN [\
#    "/bin/bash", \
#    "-c", \
#    "source /opt/ros/foxy/setup.bash && \
#    apt-get update && \
#    rosdep update && rosdep install --from-path src --ignore-src -y && \
#    colcon build --base-paths src/librealsense && \
#    source install/setup.bash"\
#    ]

#
# Get Intel D455 --ROS2
#
WORKDIR /home/projects/sensor2_ws/src/cameras/intelD455/src
RUN git clone -b foxy https://github.com/IntelRealSense/realsense-ros.git
#WORKDIR /home/projects/sensor2_ws/src/cameras/intelD455
#RUN [\
#    "/bin/bash", \
#    "-c", \
#    "source /opt/ros/foxy/setup.bash && \
#    rosdep update && rosdep install --from-path src --ignore-src -y && \
#    colcon build && \
#    source install/local_setup.bash"\
#    ]

#
# Get OAK-D --ROS2
#
WORKDIR /home/projects/sensor2_ws/src/cameras/oak/src
RUN git clone https://github.com/tiiuae/depthai_ctrl.git
#WORKDIR /home/projects/sensor2_ws/src/cameras/oak
#RUN [\
#    "/bin/bash", \
#    "-c", \
#    "source /opt/ros/foxy/setup.bash && \
#    colcon build && \
#    source install/setup.bash"\
#    ]

#
# Get Ublox GPS --ROS2
#
WORKDIR /home/projects/sensor2_ws/src/gps/ublox/src
RUN git clone -b foxy-devel https://github.com/KumarRobotics/ublox.git
WORKDIR /home/projects/sensor2_ws/src/gps/ublox
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]


#
# Vesc
#
RUN apt-get update
RUN pip3 install git+https://github.com/LiamBindle/PyVESC
    

#
# adafruit servokit
#
WORKDIR /
RUN apt-get update
RUN pip3 install \
        adafruit-circuitpython-pca9685 \
        adafruit-circuitpython-servokit \
        Jetson.GPIO && \
    groupadd -f -r gpio && \
    usermod -a -G gpio root


#
# Get ucsd_robocar_navigation_pkg
#
WORKDIR /home/projects/ros2_ws/src
RUN git clone https://gitlab.com/djnighti/ucsd_robocar_nav_pkg.git
WORKDIR /home/projects/ros2_ws
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    colcon build && \
    source install/setup.bash"\
    ]


#
# Get f110
#
WORKDIR /home/projects/f110_ws/src
RUN git clone https://github.com/f1tenth/f110_system
WORKDIR /home/projects/f110_ws/
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/noetic/setup.bash && \
    catkin_make && \
    source devel/setup.bash"\
    ]


#
# Get ucsd_robocar_hub1
#
WORKDIR /home/projects/ros1_ws/src
RUN git clone https://gitlab.com/ucsd_robocar/ucsd_robocar_hub1.git
WORKDIR /home/projects/ros1_ws
RUN git submodule update --init --recursive
RUN git submodule update --remote --merge
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/noetic/setup.bash && \
    catkin_make && \
    source devel/setup.bash"\
    ]


#
# Get ucsd_robocar_hub2
#
WORKDIR /home/projects/ros2_ws/src
RUN git clone https://gitlab.com/ucsd_robocar2/ucsd_robocar_hub2.git
WORKDIR /home/projects/ros2_ws
RUN git submodule update --init --recursive
RUN git submodule update --remote --merge
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    colcon build && \
    source install/setup.bash"\
    ]


###
# TODO: ADD CUDA enabled openCV see: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.opencv
###

###
# TODO: make sensor and actuator pkgs submodules of the nav pkg
###


###
# TODO: above is still WIP, fix sourcing below at the end
###
#
# Source all builds
#
#WORKDIR /
#RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
#    echo 'source /home/projects/rplidar_ws/devel/setup.bash' >> /root/.bashrc && \
#    echo 'source /home/projects/rslidar_ws/devel/setup.bash' >> /root/.bashrc && \
#    echo 'cd /home/projects/catkin_ws/' >> /root/.bashrc && \
#    echo 'colcon build' >> /root/.bashrc && \
#    echo 'source install/setup.bash' >> /root/.bashrc
#RUN [\
#    "/bin/bash", \
#    "-c", \
#    "source ~/.bashrc"\
#    ]
#


#
# Place user in projects
#
WORKDIR /home/projects/
