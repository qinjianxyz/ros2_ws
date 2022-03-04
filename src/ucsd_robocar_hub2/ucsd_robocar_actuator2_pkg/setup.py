from setuptools import setup
import os
from glob import glob


package_name = 'ucsd_robocar_actuator2_pkg'
submodule = str(package_name +"/vesc_submodule")

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('ucsd_robocar_actuator2_pkg/vesc_client.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adafruit_steering_node = ucsd_robocar_actuator2_pkg.adafruit_steering_node:main',
            'adafruit_throttle_node = ucsd_robocar_actuator2_pkg.adafruit_throttle_node:main',
            'vesc_rpm_node = ucsd_robocar_actuator2_pkg.vesc_rpm_node:main',
            'vesc_steering_node = ucsd_robocar_actuator2_pkg.vesc_steering_node:main',
            'vesc_twist_node = ucsd_robocar_actuator2_pkg.vesc_twist_node:main',
            'adafruit_twist_node = ucsd_robocar_actuator2_pkg.adafruit_twist_node:main',
            'adafruit_servo_node = ucsd_robocar_actuator2_pkg.adafruit_servo_node:main'
        ],
    },
)
