from flask import Flask, jsonify, request
from controll_server_pkg.common.dispatcher import assign_taxi

class RestServer:
    def __init__(self, manager):
        self.manager = manager
        self.app = Flask(__name__)
        self._setup_routes()
        self.manager.set_api(self)

    def _setup_routes(self):
        @self.app.route('/ping')
        def ping():
            return jsonify({"message": "pong"})

        @self.app.route('/move_drive', methods=['POST'])
        def move_drive():
            data = request.get_json()
            if not data:
                return jsonify({"error": "Invalid JSON"}), 400

            # ROS 드라이브 노드로 메시지 전달
            if self.manager.ros_drive:
                self.manager.ros_drive.handle_message(data)
                return jsonify({"status": "drive command sent"})
            else:
                return jsonify({"error": "ros_drive not available"}), 500
        

        @self.app.route('/call_taxi', methods=['POST'])
        def call():
            data = request.get_json()
            if not data:
                print("❌ 요청 실패: JSON 형식 아님")
                return jsonify({"error": "Invalid JSON"}), 400

            required_keys = ['start_x', 'start_y', 'dest_x', 'dest_y', 'passenger_count', 'client_id']
            if not all(key in data for key in required_keys):
                missing = [k for k in required_keys if k not in data]
                print(f"❌ 요청 실패: 필드 누락 - {missing}")
                return jsonify({"error": "Missing required fields", "missing": missing}), 400

            print(f"📨 배차 요청 수신: {data}")

            # 배차 시도
            taxi = assign_taxi(self.manager.taxis, data)

            if taxi:
                print(f"✅ 택시 {taxi.taxi_id} 배차됨 → 위치: {taxi.location} → 목적지: {taxi.destination}")
                return jsonify({
                    "status": "taxi assigned",
                    "taxi_id": taxi.taxi_id,
                    "location": taxi.location,
                    "destination": taxi.destination,
                    "client_id": taxi.client_id
                })
            else:
                print("⚠️ 배차 실패: 사용 가능한 택시 없음")
                # 디버깅용 상세 로그
                for tid, t in self.manager.taxis.items():
                    print(f"🚕 택시 {tid}: state={t.state}, battery={t.battery}, location={t.location}")
                return jsonify({"error": "No available taxi"}), 500

        @self.app.route('/update_taxi', methods=['POST'])
        def update_taxi():
            data = request.get_json()
            if not data:
                return jsonify({"error": "Invalid JSON"}), 400

            required_keys = ['taxi_id']
            if not all(key in data for key in required_keys):
                return jsonify({"error": "Missing 'taxi_id'"}), 400

            taxi_id = data['taxi_id']
            taxi = self.manager.get_taxi(taxi_id)

            if not taxi:
                return jsonify({"error": f"Taxi {taxi_id} not found"}), 404

            updates = []
            
            if 'x' in data and 'y' in data:
                taxi.update_location(data['x'], data['y'])
                updates.append(f"location=({data['x']}, {data['y']})")

            if 'battery' in data:
                taxi.update_battery(data['battery'])
                updates.append(f"battery={data['battery']}")

            if not updates:
                return jsonify({"warning": "No updates applied"}), 400

            print(f"🛠️ 택시 {taxi_id} 수동 수정: {', '.join(updates)}")

            return jsonify({
                "status": "updated",
                "taxi_id": taxi_id,
                "updated": updates,
                "taxi": taxi.to_dict()
            })

        @self.app.route('/reset_taxi', methods=['POST'])
        def reset_taxi():
            data = request.get_json() or {}

            taxi_id = data.get('taxi_id', None)  # 생략하면 전체 초기화

            target_taxis = (
                [self.manager.get_taxi(taxi_id)] if taxi_id else self.manager.taxis.values()
            )

            reset_count = 0
            for taxi in target_taxis:
                if not taxi:
                    continue
                taxi.update_location(0.0, 0.0)
                taxi.update_battery(0.0)
                taxi.update_state("ready")
                taxi.start = (0.0, 0.0)
                taxi.destination = (0.0, 0.0)
                taxi.client_id = None
                taxi.passenger_count = 0
                reset_count += 1

            if reset_count == 0:
                return jsonify({"error": "No taxi found"}), 404

            return jsonify({
                "status": "reset complete",
                "reset_taxi_count": reset_count
            })

        @self.app.route('/status_taxis', methods=['GET'])
        def status_taxis():
            taxis_info = {
                taxi_id: taxi.to_dict()
                for taxi_id, taxi in self.manager.taxis.items()
            }
            return jsonify({
                "status": "ok",
                "taxis": taxis_info
            })


    def run(self):
        print("🚀 REST 서버 실행 시작")
        self.app.run(host='0.0.0.0', port=8000)

    def handle_message(self, msg):
        print(f"[REST] 메시지 수신: {msg}")
