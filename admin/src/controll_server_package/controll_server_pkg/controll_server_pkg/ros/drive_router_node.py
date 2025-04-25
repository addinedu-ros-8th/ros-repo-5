import rclpy
from rclpy.node import Node
from ai_server_package_msgs.msg import CmdAndPinkyNum

class DriveRouterNode(Node):
    def __init__(self):
        super().__init__('drive_router_node')
        self.subscription = self.create_subscription(
            CmdAndPinkyNum,
            'drive',  # 예시 topic
            self.listener_callback,
            10
        )
        self.get_logger().info("🚦 Drive Router Node Started")

    def listener_callback(self, msg):
        self.get_logger().info(
            f"📬 Received: Robot ID={msg.pinky_num}, X={msg.x}, Y={msg.y}"
        )

def main(args=None):
    node = DriveRouterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
