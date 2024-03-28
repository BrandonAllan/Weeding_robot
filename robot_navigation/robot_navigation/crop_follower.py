import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import time

class CropFollower(Node):
    def __init__(self):
        super().__init__('crop_follower')
        self.subscription = self.create_subscription(Point, '/detected_crop', self.crop_navigation_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.2)
        self.declare_parameter("forward_chase_speed", 0.2)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)

        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value


        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.crop_following_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000


    def crop_navigation_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1-f)
        self.target_dist = self.target_dist * f + msg.z * (1-f)
        self.lastrcvtime = time.time()

    def crop_following_callback(self):
        msg = Twist()
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            self.get_logger().info('Target: {}'.format(self.target_val))
            print(self.target_dist)
            if (self.target_dist < self.max_size_thresh):
                msg.linear.x = self.forward_chase_speed
            msg.angular.z = -self.angular_chase_multiplier*self.target_val
        else:
            self.get_logger().info('Target lost')
            # Stop the robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    crop_follower = CropFollower()
    rclpy.spin(crop_follower)
    crop_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()