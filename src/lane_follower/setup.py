from setuptools import find_packages, setup
import os

package_name = 'lane_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required package metadata
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Add launch file(s)
        (os.path.join('share', package_name, 'launch'), [
            'launch/turbo_blue_launch.py'
        ]),

        # Add urdf and world files
        (os.path.join('share', package_name, 'urdf'), [
            'urdf/turbo_blue.urdf'
        ]),
        (os.path.join('share', package_name, 'worlds'), [
            'worlds/orange_igvc.world'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrewt',
    maintainer_email='andrewt@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_following_node = lane_follower.lane_following_node:main',
        ],
    },
)


