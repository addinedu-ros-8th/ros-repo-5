import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controll_server_package_msgs.srv import CmdMoveTo

class AdminServiceNode(Node):
    def __init__(self):
        super().__init__('admin_service_node')

        # 서비스 생성
        self.srv = self.create_service(
            CmdMoveTo,
            'admin_move_to_robot',
            self.handle_request
        )

        # 퍼블리셔 생성 (/cml 토픽으로 Twist 전송)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("🛠️ Admin Service Node Ready - 'admin_move_to_robot' 서비스 대기 중")

    def handle_request(self, request, response):
        self.get_logger().info(
            f"📨 이동 요청 수신: Robot ID={request.pinky_num}, X={request.x}, Y={request.y}"
        )

        # Twist 메시지 생성
        twist = Twist()
        twist.linear.x = float(request.x)
        twist.linear.y = float(request.y)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        # 퍼블리시
        self.cmd_pub.publish(twist)
        self.get_logger().info(f"🚀 /cml 퍼블리시 완료: linear.x={twist.linear.x}, linear.y={twist.linear.y}")

        # 응답 구성
        response.success = True
        response.message = f"✅ 로봇 {request.pinky_num} 이동 명령 전송됨"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AdminServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
