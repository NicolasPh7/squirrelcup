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
    
    # declare color_proc so finally block can reference it safely
    color_proc = None

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

    # Now wait up to 10s total for robot to reach crate position. The crate
    # position is known from models/simple_robot/model.sdf; we assume the
    # crate is at x=-0.2, y=0
        target_x = -0.20
        target_y = 0.0

        start_x = 0.78
        start_y = 1.20

        ddx = target_x - start_x
        ddy = target_y - start_y
        target_dist = (ddx * ddx + ddy * ddy) ** 0.5
        
        reached = False
        deadline = time.time() + 15.0
        dist = target_dist
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if odom_msgs:
                last = odom_msgs[-1]
                lx = last.pose.pose.position.x
                ly = last.pose.pose.position.y

                dist = (lx * lx + ly * ly) ** 0.5
                dist = target_dist - dist
                print(f"last_x = {lx}, last_y = {ly}, relative_dist to target = {dist}\n")
                if dist < 0.05:  # tolerance
                    reached = True
                    print(f"Target reached at relative dist {dist}. x = {lx}, y = {ly}")
                    break

        assert reached, f"Robot did not reach crate position within timeout. dist = {dist}"
    finally:

        if color_proc is not None:
            try:
                color_proc.terminate()
                color_proc.wait(timeout=5)
            except Exception:
                pass
        proc.terminate()
        proc.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
