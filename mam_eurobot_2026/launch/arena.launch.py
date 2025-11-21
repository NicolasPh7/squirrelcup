from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction

from mam_eurobot_2026.helpers import load_aruco_tags, load_crates, load_balises_fixes, start_robot_state_publisher_node

def generate_launch_description():
    pkg_path = FindPackageShare('mam_eurobot_2026')
    robot_v3_path = FindPackageShare('robot_v3')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            pkg_path
        ),
        # SetEnvironmentVariable(
        #     'GAZEBO_MODEL_PATH',
        #     value=PathJoinSubstitution([pkg_path, 'models'])
        # ),
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
                "-file", PathJoinSubstitution([robot_v3_path, "urdf", "model.urdf"]),
                "-name", "robot_v3",
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
                '/joint_1/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_2/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_3/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_4/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                '/joint_5/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                '/arm_1_joint/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                '/arm_2_joint/position_cmd@std_msgs/msg/Float64@ignition.msgs.Double'
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='joint_state_bridge',
            output='screen',
            arguments=[
                '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model'
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
            arguments=['0', '0', '0.12', '0', '0', '0', 'base_link', 'robot_v3/base_link/lidar_3d'],
            output='screen'
        ),

        OpaqueFunction(function=start_robot_state_publisher_node),

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
        #     executable='arm_controller',
        #     name='arm_controller',
        #     output='screen',
        # ),
        
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
