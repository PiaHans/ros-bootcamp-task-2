import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'map_merger'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Utkarsh Gupta',
    maintainer_email='utkarsh@example.com',
    description='Multi-robot occupancy grid map merger and TF broadcaster',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_merger_node = map_merger.map_merger_node:main',
        ],
    },
)
