from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_scenario_runner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EugeSlepynina',
    maintainer_email='slepynina.eu@gmail.com',
    description='PX4 SITL drone mission flight for field_robots_lab',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_waypoint_navigator = drone_scenario_runner.drone_waypoint_navigator:main',
        ],
    },
)
