import os
import rclpy
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class CropDetector(Node):
    def __init__(self):
        super().__init__('crop_detector')

        self.get_logger().info('Initializing Crop Detector')
        self.image_in_sub = self.create_subscription(Image, '/image_in', self.image_callback, 10)
        self.image_out_pub = self.create_publisher(Image, '/image_out', 10)
        self.crop_pub  = self.create_publisher(Point,"/detected_crop",10)

        self.model_path = os.path.join('.', 'src', 'robot_navigation', 'config', 'last.pt')
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # Threshold for confidence score
        self.threshold = 0.6

    def extract_crop_coordinates(self, results):
        crop_coordinates = []
        for result in results.boxes.data.tolist():  # Assuming YOLO results are in the format (x1, y1, x2, y2, conf, class)
            x1, y1, x2, y2, score, class_id = result
            if score > self.threshold and class_id == 0:  # Assuming crops are labeled as class 0
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                crop_coordinates.append((center_x, center_y))
        return crop_coordinates

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        results = self.model(cv_image)[0]
        crop_coordinates = self.extract_crop_coordinates(results)

        '''# Publish the image with bounding boxes (optional)
        img_out = self.bridge.cv2_to_imgmsg(results.render(), "bgr8")
        self.image_out_pub.publish(img_out)'''

        # Publish detected crop coordinates
        if len(crop_coordinates) > 0:
            crop_msg = Point()
            crop_msg.x, crop_msg.y = crop_coordinates[0]  # For simplicity, assuming only one crop detected
            self.crop_pub.publish(crop_msg)

def main(args=None):
    rclpy.init(args=args)
    crop_detector = CropDetector()
    rclpy.spin(crop_detector)
    crop_detector.destroy_node()
    rclpy.shutdown()   

if __name__ == '__main__':
    main()

 