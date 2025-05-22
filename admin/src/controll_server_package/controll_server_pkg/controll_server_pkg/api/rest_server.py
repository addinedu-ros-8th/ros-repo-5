from flask import Flask, jsonify, request
from controll_server_pkg.common.dispatcher import dispatch
from controll_server_pkg.common.database import Database
import hashlib

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
        

        @self.app.route('/login', methods=['POST'])
        def login():
            
            id = request.json.get('id')
            password = request.json.get('password')            
            
            password_hash = hash_password(password)

            db = Database()

            passenger_info = db.execute_select(
                "SELECT * FROM Passengers WHERE id = %s AND password = %s",
                (id,password_hash)
            )

            if not passenger_info:
                return jsonify({"error": "회원정보 없음"}), 400                
            
            passenger_info = passenger_info[0]

            return jsonify({
                "status": "ok",
                "passenger_id": passenger_info['passenger_id']
            })

        # ✅ 해시 함수
        def hash_password(password):
            # ✅ 고정된 salt 문자열
            SALT = "addintaxi_salt"
            return hashlib.sha256((password + SALT).encode()).hexdigest()

        @self.app.route('/call_taxi', methods=['POST'])
        def call():
            data = request.get_json()
            if not data:
                print("❌ 요청 실패: JSON 형식 아님")
                return jsonify({"error": "Invalid JSON"}), 400

            db = Database()
                
            required_keys = ['start_x', 'start_y', 'dest_x', 'dest_y', 'passenger_count', 'passenger_id']
            if not all(key in data for key in required_keys):
                missing = [k for k in required_keys if k not in data]
                print(f"❌ 요청 실패: 필드 누락 - {missing}")
                return jsonify({"error": "Missing required fields", "missing": missing}), 400

            print(f"📨 배차 요청 수신: {data}")

            call_id = db.execute_insert(
                """INSERT INTO `Call` (
                    passenger_id, 
                    start_position, 
                    end_position, 
                    passenger_count, 
                    estimated_fare, 
                    call_date
                ) VALUES (
                    %s, %s, %s, %s, %s, NOW()
                )""",
                (
                    int(data['passenger_id']),
                    f"{data['start_x']} , {data['start_y']}",
                    f"{data['dest_x']} , {data['dest_y']}",
                    data['passenger_count'],
                    0
                )
            )

            # 배차 시도
            taxi = dispatch(self.manager.taxis, data, call_id)

            if taxi:
                print(f"✅ 택시 {taxi.vehicle_id} 배차됨 → 위치: {taxi.location} → 목적지: {taxi.start} → 승객 수: {taxi.passenger_count}")               
                self.manager.drive_router_node(taxi.vehicle_id, 13, taxi.start_node)
                return jsonify({
                    "status": "taxi dispatch",
                    "vehicle_id": taxi.vehicle_id,
                    "location": taxi.location,
                    "start": taxi.start,
                    "passenger_id": taxi.passenger_id
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

            required_keys = ['vehicle_id']
            if not all(key in data for key in required_keys):
                return jsonify({"error": "Missing 'vehicle_id'"}), 400

            vehicle_id = data['vehicle_id']
            taxi = self.manager.get_taxi(vehicle_id)

            if not taxi:
                return jsonify({"error": f"Taxi {vehicle_id} not found"}), 404

            updates = []
            
            if 'x' in data and 'y' in data:
                taxi.update_location(data['x'], data['y'])
                updates.append(f"location=({data['x']}, {data['y']})")

            if 'battery' in data:
                taxi.update_battery(data['battery'])
                updates.append(f"battery={data['battery']}")
            
            if 'state' in data:
                taxi.update_state(data['state'])
                updates.append(f"state={data['state']}")

            if not updates:
                return jsonify({"warning": "No updates applied"}), 400

            print(f"🛠️ 택시 {vehicle_id} 수동 수정: {', '.join(updates)}")

            return jsonify({
                "status": "updated",
                "vehicle_id": vehicle_id,
                "updated": updates,
                "taxi": taxi.to_dict()
            })

        @self.app.route('/reset_taxi', methods=['POST'])
        def reset_taxi():
            data = request.get_json() or {}

            vehicle_id = data.get('vehicle_id', None)  # 생략하면 전체 초기화

            target_taxis = (
                [self.manager.get_taxi(vehicle_id)] if vehicle_id else self.manager.taxis.values()
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
                taxi.passenger_id = None
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
                vehicle_id: taxi.to_dict()
                for vehicle_id, taxi in self.manager.taxis.items()
            }
            return jsonify({
                "status": "ok",
                "taxis": taxis_info
            })

        @self.app.route('/pay', methods=['POST'])
        def pay():
            passenger_id = request.json.get('passenger_id')
            pay_amount = request.json.get('pay_amount')
            
            db = Database()

            passenger_info = db.execute_select(
                "SELECT * FROM Passengers WHERE passenger_id = %s",
                (passenger_id,)
            )
            if not passenger_info:
                return jsonify({"error": "회원정보 없음"}), 400
            


            passenger_info = passenger_info[0]
            if passenger_info['money'] < pay_amount:
                return jsonify({
                    "status": "ok",
                    "passenger_id": passenger_id,
                    "remaining amount"  : passenger_info['money'],
                    "message": "잔액 부족"
                })
            
            db.execute_update("UPDATE Passengers SET money = %s WHERE passenger_id = %s",(passenger_info['money'] - pay_amount, passenger_id))

            print(f" 수신 {passenger_id} , {pay_amount}")
            return jsonify({
                "status": "ok",
                "passenger_id": passenger_id,                
                "message": "결제 완료"
            })
        
        @self.app.route('/charge', methods=['POST'])
        def charge():
            passenger_id = request.json.get('passenger_id')
            pay_amount = request.json.get('pay_amount')
            
            db = Database()

            passenger_info = db.execute_select(
                "SELECT * FROM Passengers WHERE passenger_id = %s",
                (passenger_id,)
            )
            if not passenger_info:
                return jsonify({"error": "회원정보 없음"}), 400
            print(passenger_info)
            passenger_info = passenger_info[0]            
            
            db.execute_update("UPDATE Passengers SET money = %s WHERE passenger_id = %s",(passenger_info['money'] + pay_amount, passenger_id))

            print(f" 수신 {passenger_id} , {pay_amount}")
            return jsonify({
                "status": "ok",
                "passenger_id": passenger_id,                
                "message": "결제 충전"
            })

        @self.app.route('/get_balance', methods=['POST'])
        def get_balance():
            passenger_id = request.json.get('passenger_id')

            db = Database()

            passenger_info = db.execute_select(
                "SELECT * FROM Passengers WHERE passenger_id = %s",
                (passenger_id,)
            )
            if not passenger_info:
                return jsonify({"error": "회원정보 없음"}), 400

            return jsonify({
                "status": "ok",
                "user_id": passenger_id,
                "remaining amount"  : passenger_info[0]['money']
            })
        
        @self.app.route('/check_boarding', methods=['POST'])
        def check_boarding():
            vehicle_id = request.json.get('vehicle_id')
            event_type = request.json.get('event_type')
            data = request.json.get('data')
            
            status = self.manager.taxi_event_service(vehicle_id, event_type, data)

            return jsonify({
                "status": status
            })
        
        @self.app.route('/check_landing', methods=['POST'])
        def check_landing():
            vehicle_id = request.json.get('vehicle_id')
            event_type = request.json.get('event_type')
            data = request.json.get('data')
            
            status = self.manager.taxi_event_service(vehicle_id, event_type, data)

            return jsonify({
                "status": status
            })
        
        @self.app.route('/handle_message', methods=['POST'])
        def handle_message():
            vehicle_id = request.json.get('vehicle_id')
            event_type = request.json.get('event_type')
            data = request.json.get('data')
            
            status = self.manager.taxi_event_service(vehicle_id, event_type, data)

            return jsonify({
                "status": status
            })

    def run(self):
        print("🚀 REST 서버 실행 시작")
        self.app.run(host='0.0.0.0', port=8000)

    def handle_message(self, msg):
        print(f"[REST] 메시지 수신: {msg}")
