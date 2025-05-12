import socket
import threading
from struct import Struct

HOST = '0.0.0.0'
PORT = 9000

header_struct = Struct('@i')  # 메시지 코드
format_map = {
    1: Struct('@iiiii'),  # 타입 1: x1, y1, x2, y2, count
    2: Struct('@i'),       # 타입 2: command_code
}
response_struct = Struct('@ii')  # 응답: command_code, result_value

class SocketServer:
    def __init__(self, manager):
        self.manager = manager
        self.manager.set_socket(self)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_threads = []

    def handle_client(self, conn, addr):
        print(f"🧩 연결됨: {addr}")
        try:
            while True:
                code_data = conn.recv(header_struct.size)
                if not code_data:
                    break

                msg_code = header_struct.unpack(code_data)[0]
                print(f"🆔 수신된 메시지 코드: {msg_code}")

                if msg_code in format_map:
                    payload_struct = format_map[msg_code]
                    payload_data = conn.recv(payload_struct.size)
                    if not payload_data:
                        break

                    if msg_code == 1:
                        x1, y1, x2, y2, count = payload_struct.unpack(payload_data)
                        print(f"📨 [타입1] 출발=({x1},{y1}), 도착=({x2},{y2}), 인원={count}")

                        # 예: ROS 드라이브 노드 호출
                        if self.manager.ros_drive:
                            self.manager.ros_drive.handle_message({
                                "pinky_num": 1,  # 임의 고정, 필요시 count 기준 분기 가능
                                "x": x2,
                                "y": y2
                            })

                        command_code = 1
                        result_value = x2 + y2

                    elif msg_code == 2:
                        command_code, = payload_struct.unpack(payload_data)
                        print(f"📨 [타입2] 명령 코드 수신: {command_code}")
                        result_value = command_code + 1000

                    response = response_struct.pack(command_code, result_value)
                    conn.send(response)
                else:
                    print(f"⚠️ 알 수 없는 메시지 코드: {msg_code}")
                    break

        except Exception as e:
            print(f"❌ 예외 발생: {e}")
        finally:
            conn.close()
            print(f"❌ 연결 종료: {addr}")

    def run(self):
        self.server.bind((HOST, PORT))
        self.server.listen()
        print(f"🚀 TCP 서버 실행 중 (포트 {PORT})")

        while True:
            conn, addr = self.server.accept()
            thread = threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True)
            thread.start()
            self.client_threads.append(thread)

    def handle_message(self, msg):
        print(f"[SocketServer] 외부로부터 메시지 수신: {msg}")

# 🟡 단독 실행용 main()
def main():
    from controll_server_pkg.common.manager import ServiceManager
    manager = ServiceManager()
    server = SocketServer(manager)
    server.run()

if __name__ == "__main__":
    main()
