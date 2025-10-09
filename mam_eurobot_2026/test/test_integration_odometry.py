import os
import time
import pytest
import rclpy
from nav_msgs.msg import Odometry


def get_install_share(package_name):
    return os.path.join(
        os.getcwd(), 'install', package_name, 'share', package_name
    )


@pytest.mark.timeout(15)
def test_robot_reaches_crate():
    # Start rclpy for subscribing
    rclpy.init()
    node = rclpy.create_node('test_integration_node')

    odom_msgs = []

    def odom_cb(msg):
        odom_msgs.append(msg)

    sub = node.create_subscription(Odometry, '/odom', odom_cb, 10)

    # Launch arena.launch.py via ros2 launch in background using subprocess
    import subprocess

    launch_file = 'ci_arena.launch.py' if os.getenv('CI') == 'true' else 'arena.launch.py'
    launch_cmd = ['ros2', 'launch', 'mam_eurobot_2026', launch_file]
    proc = subprocess.Popen(launch_cmd)

    time.sleep(2.0)
    
    try:
        # delay launching color_follower by 3s - start it separately so the
        # test controls the delay
        time.sleep(3.0)
        color_proc = subprocess.Popen(
            ['ros2', 'run', 'mam_eurobot_2026', 'color_follower']
        )

        # record start pose
        start_pose = None
        t0 = time.time()
        while time.time() - t0 < 2.0 and not odom_msgs:
            rclpy.spin_once(node, timeout_sec=0.1)

        if odom_msgs:
            start_pose = odom_msgs[-1]

    # Now wait up to 10s total for robot to reach crate position. The crate
    # position is known from models/simple_robot/model.sdf; we assume the
    # crate is at x=-0.2, y=0
        target_x = -0.20
        target_y = 0.0

        reached = False
        deadline = time.time() + 15.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if odom_msgs:
                last = odom_msgs[-1]
                dx = last.pose.pose.position.x - target_x
                dy = last.pose.pose.position.y - target_y
                dist = (dx * dx + dy * dy) ** 0.5
                if dist < 0.05:  # tolerance
                    reached = True
                    print(f"Target reached at dist {dist}. x = {last.pose.pose.position.x}, y = {last.pose.pose.position.y}")
                    break

        assert reached, "Robot did not reach crate position within 10s"
    finally:

        try:
            if 'color_proc' in locals():
                color_proc.terminate()
                color_proc.wait(timeout=5)
        except Exception:
            pass
        proc.terminate()
        proc.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
