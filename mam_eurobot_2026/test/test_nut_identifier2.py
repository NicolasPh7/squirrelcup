import rclpy
import pytest
import time
import launch
import launch_ros.actions
import launch_testing
from rclpy.node import Node
from sensor_msgs.msg import Image
from nut_identifier_msgs.msg import NutPositionArray
import numpy as np
import cv2
from cv_bridge import CvBridge


@pytest.mark.launch_test
def generate_test_description():
    """Lance le node nut_identifier2 avant d'exécuter le test."""
    nut_identifier_node = launch_ros.actions.Node(
        package='mam_eurobot_2026',
        executable='nut_identifier2',
        name='nut_identifier2',
        output='screen'
    )

    return (
        launch.LaunchDescription([
            nut_identifier_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'nut_identifier_node': nut_identifier_node},
    )


@pytest.mark.launch_test
def test_nut_identifier2_launch(nut_identifier_node, proc_output):
    """Vérifie que le node démarre correctement."""
    # S'assure que le node s'est bien lancé
    proc_output.assertWaitFor("NutIdentifier2 node started.", timeout=10)


@pytest.mark.launch_test
def test_nut_identifier2_detection(nut_identifier_node):
    """Teste la détection et l'estimation de distance des nuts."""
    rclpy.init()
    node = Node('test_nut_identifier2')
    bridge = CvBridge()
    received = []

    def cb(msg):
        received.append(msg)

    sub = node.create_subscription(NutPositionArray, '/nuts_detected', cb, 10)
    pub = node.create_publisher(Image, '/camera', 10)

    # Attendre un peu que le node ROS soit prêt
    time.sleep(1.5)

    # Image synthétique avec une nut bleue au centre
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(img, (320, 240), 40, (255, 0, 0), -1)  # cercle bleu
    ros_img = bridge.cv2_to_imgmsg(img, encoding='bgr8')

    pub.publish(ros_img)

    # Attendre la détection
    for _ in range(25):
        rclpy.spin_once(node, timeout_sec=0.2)
        if received:
            break

    assert received, "❌ Aucune nut détectée — vérifier le lancement du node"
    assert len(received[0].nuts) > 0, "❌ La liste des nuts est vide"

    nut = received[0].nuts[0]

    # Vérifie la distance attendue (~0.17 m)
    expected_distance = 0.17
    assert abs(nut.distance - expected_distance) < 0.03, (
        f"❌ Distance estimée incorrecte: {nut.distance:.2f} m, attendue: {expected_distance:.2f} m"
    )

    # Vérifie que la nut est centrée
    assert abs(nut.angle) < 0.1, "❌ Nut pas au centre"

    rclpy.shutdown()