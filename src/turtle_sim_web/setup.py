from setuptools import setup
import os
from glob import glob

package_name = 'turtle_sim_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shaked Sabag',
    maintainer_email='sabag975@gmail.com',
    description='Headless turtlesim for web control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_node = turtle_sim_web.turtle_node:main',
        ],
    },
)