import os
import time
import pytest
import rclpy
from nav_msgs.msg import Odometry
import psutil

def terminate_process_tree(pid):
    try:
        parent = psutil.Process(pid)
        children = parent.children(recursive=True)
        for child in children:
            child.terminate()
        gone, alive = psutil.wait_procs(children, timeout=5)
        parent.terminate()
        parent.wait(timeout=5)
    except Exception as e:
        print(f"Error terminating process tree: {e}")


def get_install_share(package_name):
    return os.path.join(
        os.getcwd(), 'install', package_name, 'share', package_name
    )


@pytest.mark.timeout(15)
def test_robot_goes_cirlc_and_odometry_doesnt_drift():
    # Start rclpy for subscribing
    rclpy.init()
    node = rclpy.create_node('test_odometry_node')

    odom_msgs = []

    def odom_cb(msg):
        odom_msgs.append(msg)

    sub = node.create_subscription(Odometry, '/odom', odom_cb, 10)

    # Launch arena.launch.py via ros2 launch in background using subprocess
    import subprocess

    launch_cmd = ['ros2', 'launch', 'mam_eurobot_2026', 'arena.launch.py']
    proc = subprocess.Popen(launch_cmd)

    time.sleep(2.0)
    
    # declare color_proc so finally block can reference it safely
    color_proc = None

    try:

        # delay launching cpp_test by 3s - start it separately so the
        # test controls the delay
        time.sleep(3.0)



        color_proc = subprocess.Popen(
            ['ros2', 'run', 'mam_eurobot_2026', 'cpp_test']
        )

        t0 = time.time()
        while time.time() - t0 < 2.0 and not odom_msgs:
            rclpy.spin_once(node, timeout_sec=0.1)


        # record start pose
        start_pose = odom_msgs[-1]
        last_pose = odom_msgs[-1]

        sx = start_pose.pose.pose.position.x
        sy = start_pose.pose.pose.position.y

        deadline = time.time() + 10.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if odom_msgs:
                last_pose = odom_msgs[-1]

        lx = last_pose.pose.pose.position.x
        ly = last_pose.pose.pose.position.y
        dx = lx - sx
        dy = ly - sy
        dist = (dx * dx + dy * dy) ** 0.5
        print(f"last_x = {lx}, last_y = {ly}, relative_dist to target = {dist}\n")

        assert dist < 0.05, f"To much drift between start and end dist= {dist}. x = {lx}, y = {ly}"
    finally:
        if color_proc is not None:
            terminate_process_tree(color_proc.pid)
        terminate_process_tree(proc.pid)
        node.destroy_node()
        rclpy.shutdown()
