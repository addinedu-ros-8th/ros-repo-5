import rclpy
from rclpy.node import Node
from controll_server_package_msgs.srv import CmdMoveTo  # 정확한 경로 사용

class AdminServiceNode(Node):
    def __init__(self):
        super().__init__('admin_service_node')
        self.srv = self.create_service(CmdMoveTo, 'admin_move_to_robot', self.handle_request)

    def handle_request(self, request, response):
        self.get_logger().info(f"📨 요청받음: pinky_num={request.pinky_num}, x={request.x}, y={request.y}")
        response.success = True
        response.message = f"로봇 {request.pinky_num} 이동 명령 수락됨"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AdminServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
