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

class CropDetector(Node):
    def __init__(self):
        super().__init__('crop_detector')

        self.get_logger().info('Initializing Crop Detector')
        self.image_in_sub = self.create_subscription(Image, '/image_in', self.image_callback, 10)
        self.image_out_pub = self.create_publisher(Image, '/image_out', 5)
        self.crop_pub  = self.create_publisher(Point,"/detected_crop",10)
        self.line_pub = self.create_publisher(Float32MultiArray, 'line_parameters', 10)

        self.declare_parameter('threshold', 0.4)
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        self.model_path = os.path.join('.', 'src', 'robot_navigation', 'config', 'last.pt')
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # Threshold for confidence score
        
        # Define ROI parameters
        self.roi_start_x = 150  # Starting x-coordinate of ROI
        self.roi_end_x = 650    # Ending x-coordinate of ROI
        self.roi_start_y = 0    # Starting y-coordinate of ROI
        self.roi_end_y = 600    # Ending y-coordinate of ROI

    def extract_crop_coordinates(self, results):
        crop_coordinates = []
        for result in results.boxes.data.tolist(): 
            x1, y1, x2, y2, score, class_id = result
            if score > self.threshold and class_id == 0:  
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                crop_coordinates.append((center_x, center_y))
        return crop_coordinates

    def draw_rectangles(self, image, results):
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result
            if score > self.threshold and class_id == 0:
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 4)

    def fit_line(self, crop_coordinates, dist_type=cv2.DIST_L2, param=0, reps=0.01, aeps=0.01):
        points = np.array(crop_coordinates, dtype=np.float32).reshape(-1, 1, 2)
        [vx, vy, x0, y0] = cv2.fitLine(points, dist_type, param, reps, aeps)
        return vx, vy, x0, y0
    
    def plot_fitted_line(self, image, vx, vy, x0, y0):
        rows, cols = image.shape[:2]
        lefty = int((-x0 * vy / vx) + y0)
        righty = int(((cols - x0) * vy / vx) + y0)
        cv2.line(image, (cols - 1, righty), (0, lefty), (0, 255, 0), 2)
        return image
    
    def plot_roi(self, image):
        cv2.rectangle(image, (self.roi_start_x, self.roi_start_y), 
                      (self.roi_end_x, self.roi_end_y), 
                      (255, 0, 0), 2)  # Blue rectangle for ROI
        
    def normalize_coordinates(self, image, x, y):
        rows = float(image.shape[0])
        cols = float(image.shape[1])
        center_x    = 0.5*cols
        center_y    = 0.5*rows
        x_norm = (x - center_x) / center_x
        y_norm = (y - center_y) / center_y
        return x_norm, y_norm


    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Apply ROI to the image
        roi_image = cv_image[self.roi_start_y:self.roi_end_y, self.roi_start_x:self.roi_end_x]

        results = self.model(roi_image)[0]
        crop_coordinates = self.extract_crop_coordinates(results)

        # Draw bounding boxes on the ROI image
        self.draw_rectangles(roi_image, results)


        # Draw ROI boundaries on the original image
        self.plot_roi(cv_image)

        # Publish detected crop coordinates within the ROI
        if len(crop_coordinates) > 0:
            
            vx, vy, x0, y0 = self.fit_line(crop_coordinates)
            x0_norm, y0_norm = self.normalize_coordinates(roi_image, x0, y0)

            img_with_line = self.plot_fitted_line(roi_image, vx, vy, x0, y0)

            line_params_msg = Float32MultiArray()
            line_params_msg.data = [float(vx), float(vy), float(x0_norm), float(y0_norm)]
            self.line_pub.publish(line_params_msg)

            crop_msg = Point()
            crop_msg.x, crop_msg.y = crop_coordinates[0]
            x_norm, y_norm = self.normalize_coordinates(roi_image, crop_msg.x, crop_msg.y)
            crop_msg.x = float(x0_norm)
            crop_msg.y = float(y0_norm)
            self.crop_pub.publish(crop_msg)
        else:
            img_with_line = cv_image

        img_out = self.bridge.cv2_to_imgmsg(img_with_line, "bgr8")
        self.image_out_pub.publish(img_out)

def main(args=None):
    rclpy.init(args=args)
    crop_detector = CropDetector()
    rclpy.spin(crop_detector)
    crop_detector.destroy_node()
    rclpy.shutdown()   

if __name__ == '__main__':
    main()