from setuptools import setup
import os
from glob import glob

package_name = 'handle_estimation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_data={
        package_name: ['__pycache__/*.pyc'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.rviz')),
        ('share/' + package_name + '/model', ['model/handler_model.pt']),
        ('lib/' + package_name, [
            'handle_estimation/handle_pose_estimation_node',
            'handle_estimation/handle_pose_estimation_node.py',
        ]),
    ],
    scripts=[
        'handle_estimation/handle_pose_estimation_node',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Handle Team',
    maintainer_email='user@handle.com',
    description='Handle pose estimation using YOLO segmentation + RealSense',
    license='MIT',
    tests_require=['pytest'],
)