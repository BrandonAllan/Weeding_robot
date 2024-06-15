import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class CropFollower(Node):
    def __init__(self):
        super().__init__('crop_follower')
        self.subscription = self.create_subscription(Point, '/detected_crop', self.crop_navigation_callback, 10)
        self.crop_row_status_publisher = self.create_publisher(String, '/crop_row_status', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_follower', 10)

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.07)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.7)
        self.declare_parameter("continue_duration_secs", 2.0)  # New parameter for continuation duration

        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value        
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
        self.continue_duration_secs = self.get_parameter('continue_duration_secs').get_parameter_value().double_value

        self.timer = self.create_timer(0.1, self.crop_following_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000
        self.continue_moving = False
        self.stop_time = time.time()


    def crop_navigation_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1 - f)
        self.target_dist = self.target_dist * f + msg.z * (1 - f)
        self.lastrcvtime = time.time()
        self.continue_moving = False  # Reset continue_moving when a new target is received
        self.stop_time = time.time() + self.continue_duration_secs  # Update stop time based on current time


    def crop_following_callback(self):
        msg = Twist()
        deadband = 0.2  # Define the deadband range

        if time.time() - self.lastrcvtime < self.rcv_timeout_secs:
            self.get_logger().info('Target: {}'.format(self.target_val))
            
            if self.target_dist < self.max_size_thresh:
                msg.linear.x = self.forward_chase_speed
            
            # Implementing the deadband
            if abs(self.target_val) > deadband:
                msg.angular.z = -self.angular_chase_multiplier*self.target_val
            else:
                msg.angular.z = 0.0  # No turning within the deadband

            self.publisher_.publish(msg)
            self.continue_moving = False  # Reset the flag when receiving a target

        else:
            if not self.continue_moving:
                self.continue_moving = True  # Start the continuation phase
                self.stop_time = time.time() + self.continue_duration_secs  # Set stop time once

            if time.time() < self.stop_time:
                self.get_logger().info('Continuing to move for a bit longer...')
                msg.linear.x = self.forward_chase_speed
                msg.angular.z = 0.0  # Assuming you want to move straight ahead
                self.publisher_.publish(msg)
            else:
                self.get_logger().info('Stopping the robot and ceasing to send commands.')
                self.continue_moving = False  # Reset the flag

                # Do not publish any more messages


def main(args=None):
    rclpy.init(args=args)
    crop_follower = CropFollower()
    rclpy.spin(crop_follower)
    crop_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
