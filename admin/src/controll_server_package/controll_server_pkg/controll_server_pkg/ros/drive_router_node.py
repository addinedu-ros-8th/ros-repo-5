import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from ai_server_package_msgs.msg import CmdAndPinkyNum

class DriveRouterNode(Node):
    def __init__(self):
        super().__init__('drive_router_node')

        # QoS 설정: RELIABLE + depth 10
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # /drive 구독
        self.subscription = self.create_subscription(
            CmdAndPinkyNum,
            'drive',
            self.listener_callback,
            qos_profile
        )

        # /cmd_vel 퍼블리셔
        self.cmd_vel_pub_pinky1 = self.create_publisher(Twist, '/pinky1/cmd_vel', qos_profile)
        self.cmd_vel_pub_pinky2 = self.create_publisher(Twist, '/pinky2/cmd_vel', qos_profile)

        self.get_logger().info("🚦 Drive Router Node Started")

    def listener_callback(self, msg):
        try:
            self.get_logger().info(
                f"📬 Received: Robot ID={msg.pinky_num}, X={msg.x}, Y={msg.y}"
            )

            twist = Twist()
            twist.linear.x = float(msg.x)
            twist.linear.y = float(msg.y)
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            self.get_logger().info("✅ Twist prepared, publishing...")
            
            if msg.pinky_num == 1:
                self.cmd_vel_pub_pinky1.publish(twist)
            elif msg.pinky_num == 2:
                self.cmd_vel_pub_pinky2.publish(twist)

            self.get_logger().info(
                f"🚀 Published to /cmd_vel: linear.x={twist.linear.x}, linear.y={twist.linear.y}"
            )
        except Exception as e:
            self.get_logger().error(f"❌ Error in listener_callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DriveRouterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
