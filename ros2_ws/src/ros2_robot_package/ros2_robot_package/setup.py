from setuptools import setup
from glob import glob
import os

package_name = 'ros2_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='joseph.nomiddlename@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_publisher = ros2_robot_package.encoder_publisher:main',
            'velocity_computation = ros2_robot_package.velocity_computation:main',
            'motor_control = ros2_robot_package.motor_control:main',
            'camera_publisher = ros2_robot_package.camera_publisher:main',
        ],
    },
)