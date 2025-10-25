import yaml
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

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

