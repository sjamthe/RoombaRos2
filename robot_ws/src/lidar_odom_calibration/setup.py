from setuptools import setup
import os
from glob import glob

package_name = 'lidar_odom_calibration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shirish Jamthe',
    maintainer_email='sjamthe@yahoo.com',
    description='A ROS2 package for calibrating odometry using lidar data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_odom_calibration_node = lidar_odom_calibration.lidar_odom_calibration:main',
        ],
    },
)
