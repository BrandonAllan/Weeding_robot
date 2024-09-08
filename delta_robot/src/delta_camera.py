import os
import rclpy
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np
import torch

class DeltaCamera(Node):
    def __init__(self):
        super().__init__('delta_camera')

        self.get_logger().info('Initializing Crop Detector')
        self.image_in_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_out_pub = self.create_publisher(Image, '/image_out', 5)
        self.crop_pub = self.create_publisher(Point, "/detected_weed", 10)

        self.declare_parameter('threshold', 0.2)
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        self.model_path = os.path.join('.', 'src', 'delta_robot', 'config', 'weed_weights.pt')
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

    def extract_crop_coordinates(self, results):
        crop_coordinates = []
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result
            if score > self.threshold and class_id == 0:  # Only collect coordinates for weeds (class 0)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                crop_coordinates.append((center_x, center_y))
        return crop_coordinates

    def draw_rectangles(self, image, results):
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result
            if score > self.threshold:
                color = (0, 255, 0) if class_id == 1 else (0, 0, 255)  # Green for lettuce (class 1), Red for weeds (class 0)
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), color, 4)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        results = self.model(cv_image)[0]
        crop_coordinates = self.extract_crop_coordinates(results)

        self.draw_rectangles(cv_image, results)

        # Publish detected weed coordinates
        if len(crop_coordinates) > 0:
            crop_msg = Point()
            crop_msg.x, crop_msg.y = crop_coordinates[0]
            self.crop_pub.publish(crop_msg)

        img_out = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_out_pub.publish(img_out)

def main(args=None):
    rclpy.init(args=args)
    delta_camera = DeltaCamera()
    rclpy.spin(delta_camera)
    delta_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
