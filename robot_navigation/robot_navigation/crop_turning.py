import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CropTurning(Node):

    def __init__(self):
        super().__init__('crop_turning')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_turning', 10)
        self.sub = self.create_subscription(String, '/crop_row_status', self.crop_detection_callback, 10)
        self.turn_direction = 'left'
        self.u_turn_flag = True
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer for periodic checking

    def crop_detection_callback(self, msg):
        self.get_logger().info('Received message: %s' % msg.data)
        if msg.data == 'end_of_crop_row':
            self.u_turn_flag = True

    def turn_left(self):
        msg = Twist()
        linear_vel = 0.1
        radius = 0.5
        msg.linear.x = linear_vel
        msg.angular.z = linear_vel / radius  # Negative for turning left
        self.cmd_vel_pub.publish(msg)

    def turn_right(self):
        msg = Twist()
        linear_vel = 0.1
        radius = 0.7
        msg.linear.x = linear_vel
        msg.angular.z = -linear_vel / radius  # Positive for turning right
        self.cmd_vel_pub.publish(msg)

    def timer_callback(self):
        if self.u_turn_flag:
            if self.turn_direction == 'right':
                self.turn_right()
            elif self.turn_direction == 'left':
                self.turn_left()

def main(args=None):
    rclpy.init(args=args)
    crop_turning = CropTurning()
    rclpy.spin(crop_turning)
    crop_turning.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
