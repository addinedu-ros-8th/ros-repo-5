import socket
import threading
import time
import json
from controll_server_pkg.common.database import Database

HOST = '0.0.0.0'
PORT = 9000

class SocketServer:
    def __init__(self, manager=None):
        self.manager = manager
        if self.manager:
            self.manager.set_socket(self)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_threads = []

    def handle_client(self, conn, addr):
        print(f"🧩 연결됨: {addr}")
        try:
            # 초기 JSON 수신
            raw = conn.recv(1024).decode().strip()
            if raw == "disconnect":
                print(f"🔌 클라이언트가 즉시 종료 요청")
                return

            try:
                payload = json.loads(raw)
                vehicle_id = payload["vehicle_id"]
                print(f"📨 초기 수신: vehicle_id={vehicle_id}")
            except Exception as e:
                print(f"[❌ JSON 파싱 실패] {e} - 원본: {raw}")
                conn.close()
                return

            # 🔁 주기적으로 Taxi 정보 전송
            while True:
                conn.settimeout(0.1)
                try:
                    signal = conn.recv(1024).decode().strip()
                    if signal == "disconnect":
                        print(f"🔚 클라이언트 종료 요청 수신: {addr}")
                        break
                except socket.timeout:
                    pass
                
                # ✅ Taxi 조회 (RestServer와 동일한 방식)
                taxi = self.manager.get_taxi(vehicle_id)

                if not taxi:
                    print(f"🚫 존재하지 않는 vehicle_id: {vehicle_id}")
                    conn.sendall(json.dumps({"error": f"Taxi {vehicle_id} not found"}) .encode())
                    conn.close()
                    return
            
                taxi_data = taxi.to_dict()
                conn.sendall((json.dumps(taxi_data) + "\n").encode())
                print(f"[📤 전송됨] {taxi_data}")
                time.sleep(2)

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

def main():
    server = SocketServer()
    server.run()

if __name__ == "__main__":
    main()
