from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction

from mam_eurobot_2026.helpers import load_aruco_tags, load_crates, load_balises_fixes

def generate_launch_description():
    pkg_path = FindPackageShare('mam_eurobot_2026')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            pkg_path
        ),
        SetEnvironmentVariable(
            'GZ_LOG_LEVEL',
            "debug"
        ),
        SetEnvironmentVariable(
            'IGN_LOG_LEVEL',
            "trace"
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                pkg_path, 'worlds', 'arena_world.sdf'
            ])
        ),

        TimerAction(
            period=2.0,  # wait seconds
            actions=[
                OpaqueFunction(function=load_aruco_tags),
                OpaqueFunction(function=load_balises_fixes),
                OpaqueFunction(function=load_crates),
            ]
        ),

        
        DeclareLaunchArgument(
            'rviz_config_path',
            default_value=PathJoinSubstitution([
                pkg_path, 'resource', 'simu.rviz'
            ]),
        ),
        ExecuteProcess(
            cmd=[
                'ign', 'gazebo', '-r', LaunchConfiguration('world'), '-v', '4'
            ],
            output='screen'
        ),
        # ExecuteProcess(
        #     cmd=[
        #         "ros2", "run", "ros_gz_sim", "create",
        #         "-file", "file://models/crate",
        #         "-name", "crate",
        #         "-x", "1.5", "-y", "1.0", "-z", "0.05"
        #     ],
        #     output="screen"
        # ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/bird_eye",
                "-name", "bird_eye",
                "-x", "1.5", "-y", "0.0", "-z", "0.9", "-R", "0.0" , "-P", "1.07", "-Y", "1.57" 
            ],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-file", "file://models/robot_v2",
                "-name", "robot_v2",
                "-x", "2.7", "-y", "1.6", "-z", "0.06", "-Y", "3.1415",
                "--ros-args", "--log-level", "debug"
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
            name='arm_joint_bridge',
            output='screen',
            arguments=[
                '/arm_joint/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double'
            ]
        ),

        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='odom_bridge',
        #     output='screen',
        #     arguments=['/model/robot_v1/odometry@gz.msgs.Odometry@nav_msgs/msg/Odometry']
        # ),


        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/bird_eye@sensor_msgs/msg/Image@gz.msgs.Image'],
        ),

        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='lidar_3d_bridge',
        #     output='screen',
        #     arguments=['/lidar_3d@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan']
        # ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_3d_pc_bridge',
            output='screen',
            arguments=['/lidar_3d/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=['0', '0', '0.12', '0', '0', '0', 'base_link', 'robot_v2/base_link/lidar_3d'],
            output='screen'
        ),

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     parameters=[{
        #         'robot_description': Command(['cat', urdf_path])
        #     }]
        # ),

        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui'
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config_path')],
        ),
        Node(
            package='mam_eurobot_2026',
            executable='inertial_odometry',
            name='inertial_odometry',
            output='screen',
        ),
        Node(
            package='mam_eurobot_2026',
            executable='aruco_localization',
            name='aruco_localization',
            output='screen',
        ),
        # Node(
        #     package='mam_eurobot_2026',
        #     executable='nut_identifier',
        #     name='nut_identifier',
        #     output='screen',
        # ),
        Node(
            package='mam_eurobot_2026',
            executable='object_detector',
            name='object_detector',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        )

    ])
