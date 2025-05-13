import socket
import json
import time
HOST = '127.0.0.1'
PORT = 9000

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        client.connect((HOST, PORT))
        print(":white_check_mark: 서버에 연결됨")
        init_msg = {
            "user_id": 42,
            "taxi_id": 1
        }
        client.sendall((json.dumps(init_msg) + "\n").encode())
        print(f":outbox_tray: 초기 전송: {init_msg}")
        start_time = time.time()
        try:
            while True:
                data = client.recv(1024)
                if not data:
                    break
                lines = data.decode().splitlines()
                for line in lines:
                    msg = json.loads(line)
                    print(f":inbox_tray: 수신: {msg}")
                # 30초가 지나면 disconnect 신호 전송 후 종료
                if time.time() - start_time > 30:
                    print(":clock3: 30초 경과, 연결 종료 요청")
                    client.sendall(b"disconnect\n")
                    break
        except Exception as e:
            print(f"[:warning: 예외] {e}")
        finally:
            print(":x: 클라이언트 종료됨")
if __name__ == '__main__':
    main()