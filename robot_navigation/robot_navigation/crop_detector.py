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
        self.threshold = 0.4
        
        # Define ROI parameters
        self.roi_start_x = 200  # Starting x-coordinate of ROI
        self.roi_end_x = 440    # Ending x-coordinate of ROI
        self.roi_start_y = 0    # Starting y-coordinate of ROI
        self.roi_end_y = 640    # Ending y-coordinate of ROI

    def extract_crop_coordinates(self, results):
        crop_coordinates = []
        for result in results.boxes.data.tolist(): 
            x1, y1, x2, y2, score, class_id = result
            if score > self.threshold and class_id == 0:  
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                # Check if the detected crop is within the ROI
                #if self.roi_start_x <= center_x <= self.roi_end_x and self.roi_start_y <= center_y <= self.roi_end_y:
                crop_coordinates.append((center_x, center_y))
        return crop_coordinates

    def draw_rectangles(self, image, results):
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result
            if score > self.threshold and class_id == 0:
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 4)

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
        cv2.rectangle(cv_image, (self.roi_start_x, self.roi_start_y), (self.roi_end_x, self.roi_end_y), (255, 0, 0), 2)

        # Convert the annotated image back to ROS image message
        img_out = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_out_pub.publish(img_out)

        # Publish detected crop coordinates within the ROI
        if len(crop_coordinates) > 0:
            crop_msg = Point()
            crop_msg.x, crop_msg.y = crop_coordinates[0]

            # Normalize coordinates to have center of camera at (0, 0)
            normalized_x = 2 * ((crop_msg.x) / (self.roi_end_x - self.roi_start_x)) - 1
            normalized_y = -2 * ((crop_msg.y) / (self.roi_end_y - self.roi_start_y)) + 1


            # Assign normalized coordinates to crop_msg
            crop_msg.x = normalized_x
            crop_msg.y = normalized_y

            self.crop_pub.publish(crop_msg)


def main(args=None):
    rclpy.init(args=args)
    crop_detector = CropDetector()
    rclpy.spin(crop_detector)
    crop_detector.destroy_node()
    rclpy.shutdown()   

if __name__ == '__main__':
    main()