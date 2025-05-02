import socket
import threading
from struct import Struct

HOST = '0.0.0.0'
PORT = 9000

# Struct 정의
header_struct = Struct('@i')        # MSG_CODE
format_map = {
    1: Struct('@iiiii'),  # x1, y1, x2, y2, count
    2: Struct('@i'),       # command_code
}

response_struct = Struct('@i')  # 응답은 항상 int 1개

def handle_client(conn, addr):
    print(f"🧩 연결됨: {addr}")
    try:
        while True:
            # Step 1. 메시지 코드 먼저 받기
            code_data = conn.recv(header_struct.size)
            if not code_data:
                break

            msg_code = header_struct.unpack(code_data)[0]
            print(f"🆔 수신된 메시지 코드: {msg_code}")

            # Step 2. 해당 메시지 타입 파싱
            if msg_code in format_map:
                payload_struct = format_map[msg_code]
                payload_data = conn.recv(payload_struct.size)
                if not payload_data:
                    break

                # 코드 1: 좌표 데이터
                if msg_code == 1:
                    x1, y1, x2, y2, count = payload_struct.unpack(payload_data)
                    print(f"📨 [타입1] 출발=({x1},{y1}), 도착=({x2},{y2}), 인원={count}")
                    response = response_struct.pack(x2 + y2)

                # 코드 2: 단일 명령
                elif msg_code == 2:
                    command_code, = payload_struct.unpack(payload_data)
                    print(f"📨 [타입2] 명령 코드 수신: {command_code}")
                    response = response_struct.pack(command_code + 1000)

                conn.send(response)

            else:
                print(f"⚠️ 알 수 없는 메시지 코드: {msg_code}")
                break

    except Exception as e:
        print(f"❌ 예외 발생: {e}")
    finally:
        conn.close()
        print(f"❌ 연결 종료: {addr}")

def run_tcp_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen()
    print(f"🚀 TCP 서버 실행 중 (포트 {PORT})")

    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
        thread.start()
