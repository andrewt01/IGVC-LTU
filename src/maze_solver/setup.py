from setuptools import setup

package_name = 'maze_solver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    author='Andrew Turner',
    author_email='aturner2@ltu.edu',
    description='A simple Python node package for ROS 2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            "maze_solver = maze_solver.maze_solver:main"
        ],
    },
)

