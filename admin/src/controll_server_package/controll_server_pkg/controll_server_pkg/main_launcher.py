from threading import Thread

from controll_server_pkg.api.rest_server import RestServer
from controll_server_pkg.socket.socket_server import SocketServer
from controll_server_pkg.ros.admin_gui_service import AdminServiceNode
from controll_server_pkg.ros.drive_router_node import DriveRouterNode
from controll_server_pkg.ros.taxi_state_node import TaxiStateNode  
from controll_server_pkg.ros.taxi_event_service import TaxiEventServiceNode  
from controll_server_pkg.ros.admin_gui_topic import AdminGuiTopicPublisher
from controll_server_pkg.common.manager import ServiceManager
from controll_server_pkg.common.ArucoMarker import PinkyLocation

import rclpy
from rclpy.executors import MultiThreadedExecutor

def run_rest(manager):
    rest = RestServer(manager)
    rest.run()

def run_tcp_server(manager):
    tcp = SocketServer(manager)
    tcp.run()

def run_location_server(manager):
    loc = PinkyLocation(manager)
    loc.update()

def run_ros_nodes(manager):
    rclpy.init()
    admin_node = AdminServiceNode(manager)
    taxi_state_node = TaxiStateNode(manager)  
    taxi_event_node = TaxiEventServiceNode(manager)
    drive_router_node = DriveRouterNode(manager)
    admin_gui_topic = AdminGuiTopicPublisher(manager)
    location_node = PinkyLocation(manager)

    executor = MultiThreadedExecutor()
    executor.add_node(admin_node)
    executor.add_node(taxi_state_node)
    executor.add_node(taxi_event_node)
    executor.add_node(drive_router_node)
    executor.add_node(admin_gui_topic)
    executor.add_node(location_node)
    
    print("🚦 ROS 노드 실행 중 (admin + drive + taxi_state)")
    executor.spin()
    rclpy.shutdown()

def main():
    print("🚀 통합 서버 실행 시작")

    # 🔗 전역 통신 허브 생성
    manager = ServiceManager()

    print("🧠 REST API 실행 중...")
    Thread(target=run_rest, args=(manager,), daemon=True).start()

    print("🧠 TCP 서버 실행 중...")
    Thread(target=run_tcp_server, args=(manager,), daemon=True).start()

    print("🧠 Location 서버 실행 중...")
    Thread(target=run_location_server, args=(manager,), daemon=True).start()

    print("🧠 ROS 서비스 실행 중...")
    run_ros_nodes(manager)
