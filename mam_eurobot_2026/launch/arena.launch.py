from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare('mam_eurobot_2026')
    # urdf_path = PathJoinSubstitution([pkg_path, 'models', 'simple_robot.urdf'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            pkg_path
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                pkg_path, 'worlds', 'arena_world.sdf'
            ])
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
                "-x", "0.80", "-y", "-1.15", "-z", "0.00", "-Y", "3.14"
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

        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='odom_bridge',
        #     output='screen',
        #     arguments=['/model/simple_robot/odometry@gz.msgs.Odometry@nav_msgs/msg/Odometry']
        # ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
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
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'simple_robot/chassis/lidar_3d'],
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
            executable='nut_identifier',
            name='nut_identifier',
            output='screen',
        ),
        Node(
            package='mam_eurobot_2026',
            executable='trajectory_planner',
            name='trajectory_planner',
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
