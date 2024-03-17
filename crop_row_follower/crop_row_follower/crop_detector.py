import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO  # Assuming you have a yolov5 package for detection

class CropDetector(Node):

    def __init__(self):
        super().__init__('crop_detector')

        self.get_logger().info('Initializing crop detector...')
        self.image_sub = self.create_subscription(Image, "/camera/image_in", self.image_callback, 10)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.crop_pub = self.create_publisher(Point, "/detected_crop", 1)

        # Load YOLOv5 model
        self.model = detect.load_model("/path/to/weights.pt")  # Provide the path to your weights.pt file
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Perform crop detection
        detections = detect.detect_objects(self.model, cv_image)  

        # Publish the coordinates of detected crops
        for detection in detections:
            crop_point = Point()
            crop_point.x = detection['x']  # Example: x-coordinate of the detected crop
            crop_point.y = detection['y']  # Example: y-coordinate of the detected crop
            crop_point.z = 0.0  # Assuming z-coordinate is not applicable
            self.crop_pub.publish(crop_point)

def main(args=None):
    rclpy.init(args=args)
    crop_detector = CropDetector()
    rclpy.spin(crop_detector)
    crop_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
