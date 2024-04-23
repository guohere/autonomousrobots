import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class TurtleBot4KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_keyboard_teleop')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.twist = Twist()
        self.linear_speed = 0.3  # Adjust this value to set the default linear speed
        self.angular_speed = 0.5  # Adjust this value to set the default angular speed
        self.is_moving = False

    def on_press(self, key):
        if key == keyboard.Key.w:
            self.twist.linear.x = self.linear_speed
            self.get_logger().info('Turtlebot4 moving forward')
        elif key == keyboard.Key.s:
            self.twist.linear.x = -self.linear_speed
            self.get_logger().info('Turtlebot4 moving back')
        elif key == keyboard.Key.a:
            self.twist.angular.z = self.angular_speed
            self.get_logger().info('Turtlebot4 moving left')
        elif key == keyboard.Key.d:
            self.twist.angular.z = -self.angular_speed
            self.get_logger().info('Turtlebot4 moving right')
        elif key == keyboard.Key.z:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.is_moving = False
            self.get_logger().info('Turtlebot4 stopped moving')
            return False
        self.is_moving = True
        self.cmd_vel_publisher.publish(self.twist)

    def on_release(self, key):
        if key == keyboard.Key.w or key == keyboard.Key.s:
            self.twist.linear.x = 0.0
        elif key == keyboard.Key.a or key == keyboard.Key.d:
            self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4KeyboardTeleopNode()
    node.run()

if __name__ == '__main__':
    main()
