import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Load URDF into ROS parameter server
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(
                '/home/user/ros2_ws/src/my_robot_package/urdf/robot_description.urdf').read()}]
        ),
        
        # Launch Gazebo simulation with the IGVC world
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
