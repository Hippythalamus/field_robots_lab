from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'scenario_runner'

setup(
    name=package_name,
    version='0.3.0',
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
    description='Repeatable scenario orchestration for field_robots_lab',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = scenario_runner.waypoint_navigator:main',
            'mission_orchestrator = scenario_runner.mission_orchestrator:main',
        ],
    },
)
