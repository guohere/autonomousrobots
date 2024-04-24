import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class TurtleBot4Controller(Node):
    def __init__(self):
        super().__init__('turtlebot4_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.rate = self.create_rate(10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed_factor = 0.5  # Initial speed factor

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def control_loop(self):
        while rclpy.ok():
            key = self.get_key()
            if key == 'w':
                self.twist.linear.x = self.speed_factor
                self.twist.angular.z = 0.0
            elif key == 's':
                self.twist.linear.x = -self.speed_factor
                self.twist.angular.z = 0.0
            elif key == 'a':
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.speed_factor
            elif key == 'd':
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.speed_factor
            elif key == 'q':
                self.twist.linear.x = self.speed_factor
                self.twist.angular.z = self.speed_factor
            elif key == 'e':
                self.twist.linear.x = self.speed_factor
                self.twist.angular.z = -self.speed_factor
            elif key == 'j':
                self.speed_factor += 0.1  # Increase speed factor
                self.get_logger().info(f'Speed factor increased to {self.speed_factor}')
            elif key == 'k':
                self.speed_factor -= 0.1  # Decrease speed factor
                if self.speed_factor < 0.1:
                    self.speed_factor = 0.1  # Minimum speed factor
                self.get_logger().info(f'Speed factor decreased to {self.speed_factor}')
            elif key == 'z':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                self.get_logger().info('Stopping TurtleBot4...')
                rclpy.shutdown()
                break
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0

            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBot4Controller()
    controller.control_loop()

if __name__ == '__main__':
    main()
