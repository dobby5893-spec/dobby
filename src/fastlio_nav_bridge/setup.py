from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fastlio_nav_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dobby',
    maintainer_email='dobby@todo.todo',
    description='FAST-LIO integration for Go2 navigation and 3D mapping',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ground_prior_resolver = fastlio_nav_bridge.ground_prior_resolver:main',
            'exploration_checkpoint = fastlio_nav_bridge.exploration_checkpoint:main',
            'fastlio_tf_bridge = fastlio_nav_bridge.fastlio_tf_bridge:main',
            'fastlio_lidar_frame_relay = fastlio_nav_bridge.lidar_frame_relay:main',
        ],
    },
)
