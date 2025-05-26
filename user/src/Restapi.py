import requests
from UserSession import UserSession

API_SERVER_IP = "192.168.1.3"
API_SERVER_PORT = 8000

# # ----------------------------------------------
# # RestAPIManage - HTTP 전용 (우분투용)
# # ----------------------------------------------
class RestAPIManager:
    def __init__(self):
        self.server_ip = API_SERVER_IP
        self.server_port = API_SERVER_PORT
        self.base_url = f"http://{self.server_ip}:{self.server_port}"
        self.timeout = 5  # 5초 타임아웃
        self.session = requests.Session()
        self.session.headers.update({"Content-Type": "application/json"})
        self.max_retries = 3  # 자동 재시도 횟수

    def send_post_request(self, endpoint: str, data: dict):
        """
        서버로 POST 요청을 전송하고 응답을 반환.
        """       
        url = f"{self.base_url}{endpoint}"
        print(f"[INFO] POST 요청 URL: {url}")
        print(f"[INFO] 요청 데이터: {data}")

        try:
            response = self.session.post(url, json=data, timeout=self.timeout)
            if response.status_code == 200:
                print(f"[INFO] 서버 응답: {response.json()}")
                return response.json()
            else:
                print(f"[ERROR] 서버 오류: {response.status_code} - {response.text}")
                return None
        except requests.RequestException as e:
            print(f"[ERROR] 서버 연결 실패: {e}")

        print("[CRITICAL] 서버에 연결할 수 없습니다.")
        return None


    def send_get_request(self, endpoint: str):
        """
        서버로 GET 요청을 전송하고 응답을 반환.
        """
        url = f"{self.base_url}{endpoint}"
        print(f"[INFO] GET 요청 URL: {url}")

        for attempt in range(self.max_retries):
            try:
                response = self.session.get(url, timeout=self.timeout)
                if response.status_code == 200:
                    print(f"[INFO] 서버 응답: {response.json()}")
                    return response.json()
                else:
                    print(f"[ERROR] 서버 오류: {response.status_code} - {response.text}")
                    return None
            except requests.RequestException as e:
                print(f"[ERROR] 서버 연결 실패")

        print("[CRITICAL] 서버에 연결할 수 없습니다.")
        return None

    # 로그인
    def login(self, user_id: str, password: str):
        response = self.send_post_request("/login", {"id": user_id, "password": password})
        if response and response.get("status") == "ok":
            UserSession.login(response.get("passenger_id"))
            print(f"[INFO] 로그인 성공 - 사용자 ID: {user_id} -> passenger_id: {response.get('passenger_id')}")
            return True
        else:
            print("[ERROR] 로그인 실패")
            return False
