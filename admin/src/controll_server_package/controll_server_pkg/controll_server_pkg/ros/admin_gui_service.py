import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controll_server_package_msgs.srv import CmdMoveTo

class AdminServiceNode(Node):
    def __init__(self, manager):
        super().__init__('admin_service_node')
        self.manager = manager
        self.manager.set_ros_admin_service(self)

        # 서비스 생성
        self.srv = self.create_service(
            CmdMoveTo,
            'admin_move_to_robot',
            self.handle_request
        )

        # 퍼블리셔 생성
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("🛠️ Admin Service Node Ready - 'admin_move_to_robot' 서비스 대기 중")

    def handle_request(self, request, response):
        self.get_logger().info(
            f"📨 이동 요청 수신: Robot ID={request.pinky_num}, X={request.x}, Y={request.y}"
        )

        twist = Twist()
        twist.linear.x = float(request.x)
        twist.linear.y = float(request.y)

        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"🚀 /cmd_vel 퍼블리시 완료: x={twist.linear.x}, y={twist.linear.y}"
        )

        response.success = True
        response.message = f"✅ 로봇 {request.pinky_num} 이동 명령 전송됨"
        return response

    def handle_message(self, msg):
        self.get_logger().info(f"[AdminServiceNode] 외부 메시지 수신: {msg}")

# 🟡 단독 실행할 경우를 위한 main
def main(args=None):
    from controll_server_pkg.common.manager import ServiceManager
    manager = ServiceManager()

    rclpy.init(args=args)
    node = AdminServiceNode(manager)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
