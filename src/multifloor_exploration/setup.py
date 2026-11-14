from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'multifloor_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
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
    description='Autonomous two-floor exploration coordinator for Go2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'multifloor_coordinator = multifloor_exploration.coordinator:main',
        ],
    },
)
