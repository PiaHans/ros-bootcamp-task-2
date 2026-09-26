import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'exploration'

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
    description='Frontier-based autonomous exploration node for multi-robot system',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'frontier_explorer = exploration.frontier_explorer:main',
        ],
    },
)
