import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time

class CropFollower(Node):
    def __init__(self):
        super().__init__('crop_follower')
        self.subscription = self.create_subscription(Point, '/detected_crop', self.crop_navigation_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_follower', 10)

        self.declare_parameter("angular_chase_multiplier", 0.07)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.7)
        self.declare_parameter("target_lost_timeout", 2.0)

        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value        
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
        self.target_lost_timeout = self.get_parameter('target_lost_timeout').get_parameter_value().double_value

        self.timer = self.create_timer(0.1, self.crop_following_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time()

    def crop_navigation_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1 - f)
        self.target_dist = self.target_dist * f + msg.z * (1 - f)
        self.lastrcvtime = time.time()    

    def crop_following_callback(self):
        msg = Twist()
        deadband = 0.2  # Define the deadband range

        current_time = time.time()
        if current_time - self.lastrcvtime < self.target_lost_timeout:
            self.get_logger().info('Target: {}'.format(self.target_val))

            if self.target_dist < self.max_size_thresh:
                msg.linear.x = self.forward_chase_speed

            # Implementing the deadband
            if abs(self.target_val) > deadband:
                msg.angular.z = -self.angular_chase_multiplier * self.target_val
            else:
                msg.angular.z = 0.0  # No turning within the deadband

            self.publisher_.publish(msg)

        elif current_time - self.lastrcvtime <= self.target_lost_timeout + 2.0:
            msg.linear.x = self.forward_chase_speed
            self.publisher_.publish(msg)
        else:
            self.get_logger().info('target lost')


def main(args=None):
    rclpy.init(args=args)
    crop_follower = CropFollower()
    rclpy.spin(crop_follower)
    crop_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
