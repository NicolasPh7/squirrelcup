import os
import yaml
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory

def load_aruco_tags(context):
    pkg_path = FindPackageShare('mam_eurobot_2026').perform(context)
    config_path = PathJoinSubstitution([pkg_path, 'config', 'aruco_tags.yaml']).perform(context)

    print(f"Loading ArUco tags from: {config_path}")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        print(f"Loaded tags: {config.get('aruco_tags', [])}")

    tag_processes = []
    for tag in config.get('aruco_tags', []):
        print(f"Loading tags {tag['name']}")
        x, y, z, roll, pitch, yaw = tag['pose']
        tag_processes.append(
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "ros_gz_sim", "create",
                    "-file", tag['model_path'],
                    "-name", tag['name'],
                    "-x", str(x), "-y", str(y), "-z", str(z),
                    "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)
                ],
                output="screen"
            )
        )
    return tag_processes

def load_crates(context):
    pkg_path = FindPackageShare('mam_eurobot_2026').perform(context)
    config_path = PathJoinSubstitution([pkg_path, 'config', 'crates.yaml']).perform(context)

    print(f"Loading crates from: {config_path}")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        crates = config.get("crates", {})
        print(f"Loaded tags: {config.get('crates', {})}")


    crate_processes = []

    
    for color in ['blue_crates', 'yellow_crates', 'black_crates']:
        for crate in crates.get(color, []):
            print(f"Loading crate {crate['name']}")
            x, y, z, roll, pitch, yaw = crate['pose']
            model_name = color[:-1]
            crate_processes.append(
                ExecuteProcess(
                    cmd=[
                        "ros2", "run", "ros_gz_sim", "create",
                        "-file", f"file://models/{model_name}",
                        "-name", crate['name'],
                        "-x", str(x), "-y", str(y), "-z", str(z),
                        "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)
                    ],
                    output="screen"
                )
            )
    return crate_processes


def load_balises_fixes(context):
    pkg_path = FindPackageShare('mam_eurobot_2026').perform(context)
    config_path = PathJoinSubstitution([pkg_path, 'config', 'fix_balises.yaml']).perform(context)

    print(f"Loading balises fixes from: {config_path}")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        print(f"Loaded balises: {config.get('balises_fixes', [])}")

    balise_processes = []
    for balise in config.get('balises_fixes', []):
        print(f"Loading balise {balise['name']}")
        x, y, z, roll, pitch, yaw = balise['pose']
        balise_processes.append(
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "ros_gz_sim", "create",
                    "-file", balise['model_path'],
                    "-name", balise['name'],
                    "-x", str(x), "-y", str(y), "-z", str(z),
                    "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)
                ],
                output="screen"
            )
        )
    return balise_processes


def wait_for_clock(context, *args, **kwargs):
    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node('clock_waiter')
    received = False

    def callback(msg):
        nonlocal received
        received = True

    sub = node.create_subscription(Clock, '/clock', callback, 10)

    print("[Launch] Waiting for /clock to publish...")
    timeout_sec = 10.0
    start = node.get_clock().now().seconds_nanoseconds()[0]
    while not received and node.get_clock().now().seconds_nanoseconds()[0] - start < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if not received:
        print("[Launch] WARNING: /clock did not publish within timeout.")
    else:
        print("[Launch] /clock is active — proceeding.")

def start_robot_state_publisher_node(context):
    pkg_path = get_package_share_directory('mam_eurobot_2026')
    robot_v3_path = get_package_share_directory('robot_v3')

    with open(os.path.join(robot_v3_path, 'urdf', 'model.urdf'), 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    return [
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui'
        # ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        )
    ]
