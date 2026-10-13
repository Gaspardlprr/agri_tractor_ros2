from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'agri_tractor_ros2'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Enzo Sousa',
    maintainer_email='enzo.sousa77@gmail.com',
    description='Motoculteur VTR : Gazebo Harmonic + ROS 2 Jazzy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tractor_controller = agri_tractor_ros2.tractor_controller:main',
            'turning_test = agri_tractor_ros2.turning_test:main',
            'vtr_node = agri_tractor_ros2.vtr_node:main',
        ],
    },
)
