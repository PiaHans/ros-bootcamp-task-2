from setuptools import setup

package_name = 'multi_bot_core'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jrtics',
    maintainer_email='jrtics@todo.todo',
    description='Map merging and coordinated frontier exploration for a robot fleet.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_merger = multi_bot_core.map_merger:main',
            'explorer = multi_bot_core.explorer:main',
        ],
    },
)
