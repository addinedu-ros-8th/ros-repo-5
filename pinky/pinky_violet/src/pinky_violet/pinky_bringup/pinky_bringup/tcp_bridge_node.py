import socket
import threading
import time
import rclpy
from rclpy.node import Node
from controll_server_package_msgs.srv import TaxiEvent


class TcpBridgeNode(Node):
    def __init__(self):
        super().__init__('tcp_bridge_node')

        # 🔹 초기화: 서버 ↔ ESP32 통신 및 ROS 서비스 연동 설정

        # ACK 수신 및 이벤트 관리용 플래그
        self.ack_event = threading.Event()
        self.last_ack_message = None

        # [1] 서버 내부에서 호출되는 ROS 서비스 등록 (→ server_to_esp32_send_command 호출됨)
        self.get_logger().info("💡 set_event_state 서비스 등록 시도")
        self.create_service(TaxiEvent, 'set_event_state', self.server_to_esp32_send_command)
        self.get_logger().info("✅ set_event_state 서비스 서버 등록 완료")

        # [2] ESP32 → 서버 이벤트 전송 시 사용할 클라이언트 (→ esp32_to_server_send_event에서 사용)
        self.taxi_event_client = self.create_client(TaxiEvent, 'TaxiEvent')
        threading.Thread(
            target=self.server_wait_for_service_thread,
            args=(self.taxi_event_client, 'TaxiEvent'),
            daemon=True
        ).start()

        # [3] ESP32와 TCP 연결 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', 9000))
        self.sock.listen(1)
        self.get_logger().info("📡 ESP32로부터의 TCP 연결 대기 중 (포트 9000)")

        # [4] ESP32 연결 수신 및 메시지 처리 스레드 시작 → esp32_to_server_handle_message 호출됨
        self.conn = None
        threading.Thread(
            target=self.esp32_to_server_listen_message_thread,
            daemon=True
        ).start()

        # [5] 헬스 체크용 주기적 ping 스레드 시작 → ESP32와 연결 상태 확인
        threading.Thread(
            target=self.esp32_ping_check_thread,
            daemon=True
        ).start()

    def server_wait_for_service_thread(self, client, name):
        """[서비스 준비 대기] 서버가 ROS 서비스 준비 완료될 때까지 대기함."""
        # 호출 위치: __init__()
        # 다음 이동: 연결 완료 후 client.call_async() 가능 상태됨
        logged = False
        while not client.wait_for_service(timeout_sec=1.0):
            if not logged:
                self.get_logger().info(f'⏳ {name} 서비스 연결 대기 중...')
                logged = True
        self.get_logger().info(f"✅ {name} 서비스 연결 완료")

    def esp32_to_server_listen_message_thread(self):
        """[ESP32 메시지 수신 루프] TCP 연결 수락 후 지속적으로 메시지 수신."""
        # 호출 위치: __init__()
        # 다음 이동: 수신된 메시지를 esp32_to_server_handle_message 로 전달
        while True:
            try:
                self.get_logger().info("🔄 ESP32 재연결 대기 중...")
                conn, addr = self.sock.accept()
                self.conn = conn
                self.get_logger().info(f"🔌 ESP32 연결됨: {addr}")

                while True:
                    try:
                        data = self.recv_fixed_length_message_from_esp32(self.conn, 12)
                        if not data or len(data) != 12:
                            self.get_logger().warn(f"❗ 잘못된 메시지 수신 또는 연결 종료 (len={len(data)})")
                            break

                        msg = data.decode('utf-8', errors='ignore')
                        if "ACK" in msg[4:]:
                            # 수신 메시지가 ACK인 경우 → ack_event 플래그 설정 후 종료
                            self.last_ack_message = msg
                            self.ack_event.set()
                            self.get_logger().info(f"✅ ACK 수신됨: '{msg}'")
                            continue

                        self.get_logger().info(f"📨 ESP32 → 서버 수신: '{msg}'")

                        # 다음 이동: ESP32에 ACK 응답 후 메시지 처리 함수 호출
                        ack_message = f"{msg[0:4]}ACK{msg[7:12]}"
                        self.conn.sendall(ack_message.encode())

                        self.esp32_to_server_handle_message(msg)
                    except (ConnectionResetError, socket.timeout) as e:
                        self.get_logger().warn(f"⚠️ ESP32 연결 끊김 또는 오류: {e}")
                        break
                    except Exception as e:
                        self.get_logger().error(f"❌ 수신 처리 오류: {e}")
                        break

            except Exception as e:
                self.get_logger().error(f"❌ ESP32 연결 수락 중 오류: {e}")
                time.sleep(1)

            finally:
                # 연결 정리
                if self.conn:
                    try:
                        self.conn.shutdown(socket.SHUT_RDWR)
                    except Exception:
                        pass
                    try:
                        self.conn.close()
                    except Exception:
                        pass
                    self.get_logger().warn("🔌 ESP32 연결 종료 (정리 완료)")
                    self.conn = None

    def recv_fixed_length_message_from_esp32(self, sock, length):
        """[수신 유틸] 정확한 길이의 데이터를 수신하여 반환."""
        # 호출 위치: esp32_to_server_listen_message_thread()
        buffer = b""
        while len(buffer) < length:
            chunk = sock.recv(length - len(buffer))
            if not chunk:
                return None
            buffer += chunk
        return buffer

    def esp32_to_server_handle_message(self, msg):
        """[메시지 파싱 핸들러] ESP32 메시지를 vehicle_id, event_type, data로 분해 후 전달."""
        # 호출 위치: esp32_to_server_listen_message_thread()
        # 다음 이동: esp32_to_server_send_event()
        try:
            vehicle_id = int(msg[0:2])
            event_type = int(msg[2:4])
            data = msg[4:12]
            self.esp32_to_server_send_event(vehicle_id, event_type, data)
        except Exception as e:
            self.get_logger().error(f"❌ 파싱 오류: {e}")

    def esp32_to_server_send_event(self, vehicle_id, event_type, data):
        """[서버 이벤트 호출] ROS 서비스 호출을 통해 서버에 이벤트 전달."""
        # 호출 위치: esp32_to_server_handle_message()
        # 다음 이동: 서버 노드의 TaxiEvent 서비스 실행
        req = TaxiEvent.Request()
        req.vehicle_id = vehicle_id
        req.event_type = event_type
        req.data = data

        log_event = {
            1: "승차", 2: "하차", 3: "대기", 4: "운행", 5: "충전",
            6: "좌측 지시등", 7: "우측 지시등", 8: "비상 지시등", 9: "지시등 해제",
            10: "문 열림", 11: "문 닫힘", 12: "RFID 태그"
        }.get(event_type, f"기타 이벤트({event_type})")

        self.get_logger().info(f"🚖 ESP32 → 서버 이벤트 전달: ID='{vehicle_id}', 이벤트='{log_event}', 데이터='{data}'")

        future = self.taxi_event_client.call_async(req)

        def log_callback(fut):
            try:
                result = fut.result()
                self.get_logger().info(f"📬 서버 응답 수신: 성공={result.result}")
            except Exception as e:
                self.get_logger().error(f"❌ 서버 응답 실패: {e}")

        future.add_done_callback(log_callback)

    def server_to_esp32_send_command(self, request, response):
        """[명령 송신] ROS 서비스 요청을 받아 ESP32로 명령 전송 후 ACK 확인."""
        # 호출 위치: set_event_state ROS 서비스 → 외부 서버나 노드에서 호출됨
        # 다음 이동: ESP32로 12바이트 명령 송신 및 ACK 대기
        if not self.conn:
            self.get_logger().warn("❗ ESP32 미연결 상태")
            response.result = False
            return response

        try:
            self.ack_event.clear()
            self.last_ack_message = None

            message = f"{request.vehicle_id:02d}{request.event_type:02d}{request.data:<8}"[:12]
            self.conn.sendall(message.encode())
            self.get_logger().info(f"📤 서버 → ESP32 명령 전송: '{message}'")

            if self.ack_event.wait(timeout=3.0):
                self.get_logger().info(f"✅ ESP32 응답 수신: '{self.last_ack_message}'")
                response.result = True
            else:
                self.get_logger().warn("⚠️ ACK 응답 타임아웃")
                response.result = False

        except Exception as e:
            self.get_logger().error(f"❌ ESP32 전송 중 오류: {e}")
            try:
                self.conn.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None
            response.result = False

        return response

    def esp32_ping_check_thread(self):
        """[헬스 체크] ESP32에 주기적으로 PING 메시지 전송."""
        # 호출 위치: __init__()
        # 다음 이동: 5초 간격으로 ESP32에 PING 전송
        while True:
            time.sleep(15)
            if self.conn:
                try:
                    self.conn.sendall(b"0000PING    ")
                    self.get_logger().debug("💓 ESP32 헬스 체크: 0000PING 전송")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ ESP32 헬스 체크 실패, 연결 종료: {e}")
                    try:
                        self.conn.shutdown(socket.SHUT_RDWR)
                    except Exception:
                        pass
                    try:
                        self.conn.close()
                    except Exception:
                        pass
                    self.conn = None

def main(args=None):
    rclpy.init(args=args)
    node = TcpBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()