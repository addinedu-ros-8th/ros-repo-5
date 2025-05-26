import socket
import threading
import time
import json
import os
import signal
import subprocess
from controll_server_pkg.common.database import Database

HOST = '0.0.0.0'
PORT = 9000

def kill_process_on_port(port):
    """9000번 포트를 점유 중인 프로세스를 찾아 강제 종료"""
    try:
        result = subprocess.run(
            ["lsof", "-ti", f"tcp:{port}"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True
        )
        pids = result.stdout.strip().split('\n')
        for pid in pids:
            if pid:
                print(f"🔌 PID {pid} 종료 중...")
                os.kill(int(pid), signal.SIGKILL)
        if pids and pids[0]:
            print("✅ 기존 포트 점유 프로세스 종료 완료")
        else:
            print("✅ 포트 9000을 사용하는 프로세스 없음")
    except Exception as e:
        print(f"❌ 포트 종료 중 오류 발생: {e}")

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
                conn.settimeout(1.0)
                try:
                    signal = conn.recv(1024).decode().strip()
                    if signal == "disconnect":
                        print(f"🔚 클라이언트 종료 요청 수신: {addr}")
                        break
                except socket.timeout:
                    pass
                
                taxi = self.manager.get_taxi(vehicle_id)
                if not taxi:
                    print(f"🚫 존재하지 않는 vehicle_id: {vehicle_id}")
                    conn.sendall(json.dumps({"error": f"Taxi {vehicle_id} not found"}).encode())
                    conn.close()
                    return

                taxi_data = taxi.to_dict()
                conn.sendall((json.dumps(taxi_data) + "\n").encode())
                print(f"[📤 전송됨] {taxi_data}")
                time.sleep(1)

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
    print("🧹 9000번 포트 정리 시도 중...")
    kill_process_on_port(PORT)
    time.sleep(1)  # 포트 정리 후 대기

    server = SocketServer()
    server.run()

if __name__ == "__main__":
    main()
