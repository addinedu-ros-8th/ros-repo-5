from threading import Thread
from controll_server_pkg.api.rest_server import run_rest
from controll_server_pkg.socket.socket_server import run_tcp_server
from controll_server_pkg.ros.admin_gui_service import AdminServiceNode
from controll_server_pkg.ros.drive_router_node import DriveRouterNode

import rclpy
from rclpy.executors import MultiThreadedExecutor

def run_ros_nodes():
    rclpy.init()
    executor = MultiThreadedExecutor()
    admin_node = AdminServiceNode()
    drive_node = DriveRouterNode()
    executor.add_node(admin_node)
    executor.add_node(drive_node)
    print("🚦 ROS 노드 실행 중 (admin + drive)")
    executor.spin()
    rclpy.shutdown()

def main():
    print("🚀 통합 서버 실행 시작")
    print("🧠 REST API 실행 중...")
    Thread(target=run_rest, daemon=True).start()
    
    print("🧠 TCP 서버 실행 중...")
    Thread(target=run_tcp_server, daemon=True).start()
    
    print("🧠 ROS 서비스 실행 중...")
    run_ros_nodes()
