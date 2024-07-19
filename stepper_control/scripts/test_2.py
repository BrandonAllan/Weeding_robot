#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')
        self.publisher_ = self.create_publisher(String, '/testObjectcoordinates', 10)
        self.timer = self.create_timer(1, self.publish_coordinates)

    def publish_coordinates(self):
        # Example coordinates
        coordinates =  ('G'+(str(0)+'A')+(str(0)+'B')+(str(-400)+'C'))
        msg = String()
        msg.data = coordinates
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published coordinates: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
