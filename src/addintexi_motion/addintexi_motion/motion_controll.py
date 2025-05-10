import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class PIDController(Node):
    def __init__(self):
        super().__init__('addintexi_motion_pid_controller')

        # Velocity Publisher
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Command Subscriber
        self.create_subscription(Int32, '/motion_command', self.command_callback, 10)

        self.get_logger().info("PID Controller Node Initialized.")

    def command_callback(self, msg):
        command = msg.data
        twist = Twist()

        if command == 0:  # 전진
            twist.linear.x = 0.5
            twist.angular.z = 0.0

        elif command == 1:  # 좌회전
            twist.linear.x = 0.5
            twist.angular.z = 1.0

        elif command == 2:  # 우회전
            twist.linear.x = 0.5
            twist.angular.z = -1.0

        elif command == 3:  # 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.vel_pub.publish(twist)
        self.get_logger().info(f"Executing Command: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
