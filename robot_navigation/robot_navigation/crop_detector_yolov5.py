import os
import rclpy
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import torch

class CropDetector(Node):
    def __init__(self):
        super().__init__('crop_detector')

        self.get_logger().info('Initializing Crop Detector')
        self.image_in_sub = self.create_subscription(Image, '/image_in', self.image_callback, 10)
        self.image_out_pub = self.create_publisher(Image, '/image_out', 10)
        self.crop_pub = self.create_publisher(Point, "/detected_crop", 10)
        self.flag_pub = self.create_publisher(String, "/flag_detected", 1)  # New publisher for flags

        self.weights = os.path.join('.', 'src', 'robot_navigation', 'config', 'real_test_weights.pt')
        self.model = self.load_model(self.weights)
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("\n\nDevice Used:", self.device)
        self.bridge = CvBridge()

        # Threshold for confidence score
        self.threshold = 0.5

        # Define ROI parameters
        self.roi_start_x = 200  # Starting x-coordinate of ROI
        self.roi_end_x = 440    # Ending x-coordinate of ROI
        self.roi_start_y = 0    # Starting y-coordinate of ROI
        self.roi_end_y = 480    # Ending y-coordinate of ROI

    def load_model(self, weights_path):
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path)
        return model

    def draw_rectangles(self, results, frame):
        for *xyxy, conf, cls in results.xyxy[0]:
            if conf >= self.threshold:
                if cls == 0:  # Assuming class_id 0 is for lettuces
                    color = (0, 0, 255)  # Red color for lettuces
                elif cls == 1:  # Assuming class_id 1 is for flags
                    color = (0, 255, 0)  # Green color for flags
                self.plot_one_box(xyxy, frame, color=color, line_thickness=2)
        return frame

    def plot_one_box(self, x, img, color=(0, 0, 255), line_thickness=None):
        # Plots one bounding box on image img
        tl = line_thickness or int(round(0.002 * max(img.shape[0:2])))  # line thickness
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)

    def extract_crop_coordinates(self, results):
        crop_coordinates = []
        for *xyxy, conf, cls in results.xyxy[0]:
            if conf > self.threshold and cls == 0:  # Assuming class_id 0 is for lettuces
                x1, y1, x2, y2 = xyxy
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                # Check if the detected crop is within the ROI
                crop_coordinates.append((float(center_x), float(center_y)))
        return crop_coordinates

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Apply ROI to the image
        roi_image = cv_image[self.roi_start_y:self.roi_end_y, self.roi_start_x:self.roi_end_x]

        results = self.model(roi_image)

        # Draw bounding boxes on the ROI image
        self.draw_rectangles(results, roi_image)

        # Extract crop coordinates
        crop_coordinates = self.extract_crop_coordinates(results)

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

        # Check if any flags are detected
        flag_detected = any(conf > self.threshold and cls == 1 for *xyxy, conf, cls in results.xyxy[0])
        flag_msg = String()
        if flag_detected:
            flag_msg.data = "flag_detected"
        else:
            flag_msg.data = "no_flag"
        self.flag_pub.publish(flag_msg)

def main(args=None):
    rclpy.init(args=args)
    crop_detector = CropDetector()
    rclpy.spin(crop_detector)
    crop_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
