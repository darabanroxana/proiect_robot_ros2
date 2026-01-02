from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_football'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Include all world files in the install
data_files.append(
    ('share/' + package_name + '/worlds', glob('worlds/*.world'))
)

# Include all launch files in the install
data_files.append(
    ('share/' + package_name + '/launch', glob('launch/*.py'))
)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='TurtleBot3 football simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = turtlebot3_football.vision_node:main',
            'ball_chaser = turtlebot3_football.ball_chaser:main',
        ],
    },
)