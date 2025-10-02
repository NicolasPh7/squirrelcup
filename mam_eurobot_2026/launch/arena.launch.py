from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare('mam_eurobot_2026')
    
    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            pkg_path
        ),
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution([pkg_path, 'worlds', 'arena_world.sdf'])),
        ExecuteProcess(
            cmd=["ign", "gazebo", "-r", LaunchConfiguration("world"),"-v", "4"],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/crate",
                "-name", "crate",
                "-x", "-0.20", "-y", "0", "-z", "0.05"
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/simple_robot",
                "-name", "simple_robot",
                "-x", "0.05", "-y", "0", "-z", "0.05", "-Y", "3.1415"
            ],
            output="screen"
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])