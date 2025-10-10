import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nut_identifier_msgs.msg import NutPositionArray
import numpy as np
import cv2
from cv_bridge import CvBridge

def test_nut_identifier2():
    rclpy.init()
    node = Node('test_nut_identifier2')
    bridge = CvBridge()
    received = []

    def cb(msg):
        received.append(msg)

    sub = node.create_subscription(NutPositionArray, '/nuts_detected', cb, 10)
    pub = node.create_publisher(Image, '/camera', 10)

    # Image synthétique avec nut bleu au centre
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(img, (320, 240), 40, (255, 0, 0), -1)
    ros_img = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    pub.publish(ros_img)

    # Attendre la réponse
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.2)
        if received:
            break

    assert received, "Aucune nut détectée"
    assert len(received[0].nuts) > 0, "La liste des nuts est vide"
    nut = received[0].nuts[0]

    # Vérification de la distance attendue
    # nut_real_diameter_mm = 17.0, camera_focal_px = 800.0
    # Le cercle a un rayon de 40px, donc un diamètre de 80px.
    # distance_mm = (17.0 * 800.0) / 80.0 = 170.0 mm
    # distance_m = 0.17 m
    expected_distance = 0.17
    assert abs(nut.distance - expected_distance) < 0.02, f"Distance estimée incorrecte: {nut.distance}, attendue: {expected_distance}"
    assert abs(nut.angle) < 0.1, "Nut pas au centre"

    rclpy.shutdown()