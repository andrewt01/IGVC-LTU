from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('lane_follower')
    urdf_path = os.path.join(pkg_path, 'urdf', 'turbo_blue.urdf')
    world_path = os.path.join(pkg_path, 'worlds', 'orange_igvc.world')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Path to the gazebo_ros package and launch file
    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )

    return LaunchDescription([
        # Launch Gazebo with the world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={'world': world_path}.items()  # Pass the world path if needed
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'turbo_blue'],
            output='screen'
        )
    ])

