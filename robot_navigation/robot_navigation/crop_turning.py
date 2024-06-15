import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class CropTurning(Node):

    def __init__(self):
        super().__init__('crop_turning')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_turning', 10)
        self.flag_sub = self.create_subscription(String, '/flag_detected', self.flag_callback, 10)  # Subscription for flags
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)  # Subscription for odometry
        
        self.flag_detected = True
        self.turn_direction = 'right'  # Track the current direction of turning
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer for periodic checking

    def flag_callback(self, msg):
        if msg.data == 'flag_detected':
            self.flag_detected = True
        else:
            self.flag_detected = False

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.get_logger().info('Yaw angle: {:.2f}'.format(yaw))

    def turn_right(self):
        msg = Twist()
        linear_vel = 0.15
        radius = 0.6
        msg.linear.x = linear_vel
        msg.angular.z = -linear_vel / radius  # Negative for turning right
        self.cmd_vel_pub.publish(msg)

    def turn_left(self):
        msg = Twist()
        linear_vel = 0.15
        radius = 0.6
        msg.linear.x = linear_vel
        msg.angular.z = linear_vel / radius  # Positive for turning left
        self.cmd_vel_pub.publish(msg)

    def stop_turning(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def timer_callback(self):
        if self.flag_detected:
            if self.turn_direction == 'right':
                self.turn_right()
            else:
                self.turn_left()
                self.turn_direction = 'right'  # Switch to right for the next detection
        else:
            self.stop_turning()

def main(args=None):
    rclpy.init(args=args)
    crop_turning = CropTurning()
    rclpy.spin(crop_turning)
    crop_turning.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
