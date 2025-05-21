import networkx as nx
import numpy as np
from controll_server_pkg.common.database import Database

# 🔹 노드 위치 정보
positions = {
    "A": (0.262, 0.161), "B": (0.241, 0.196), "C": (0.215, 0.229), "D": (0.17, 0.217), "E": (0.177, 0.192),
    "F": (0.205, 0.127), "G": (0.183, 0.146), "H": (0.139, 0.16), "I": (0.102, 0.148),
    "J": (0.127, 0.057), "K": (0.08, 0.069), "L": (0.1, 0.097), "M": (0.039, 0.094),
    "N": (0.063, 0.005), "O": (0.031, 0.002), "P": (-0.013, 0.015), "Q": (-0.022, 0.033),
    "R": (-0.015, -0.078), "S": (-0.036, -0.055), "T": (-0.005, -0.017), "U": (-0.055, -0.018), "V": (-0.045, 0.021)

}

# 🔹 휴리스틱 함수
def heuristic(n1, n2):
    x1, y1 = positions[n1]
    x2, y2 = positions[n2]
    return np.hypot(x2 - x1, y2 - y1)

# 🔹 그래프 정의
G = nx.DiGraph()
G.add_edges_from([
    ('B', 'A'), ('A', 'R'), ('C', 'B'), ('C', 'E'), ('E', 'K'), ('D', 'C'),
    ('F', 'G'), ('G', 'K'), ('H', 'I'), ('I', 'M'),
    ('N', 'J'), ('N', 'J'),('J', 'F'), ('K', 'U'), ('K', 'O'), ('L', 'B'), ('L', 'H'), ('I', 'M'), ('M', 'Q'), ('Q', 'P'),
    ('O', 'N'), ('P', 'L'), ('P', 'L'),
    ('R', 'S'), ('S', 'T'), ('S', 'U'), ('T', 'L'), ('U', 'V'), ('V', 'D')
])
for u, v in G.edges:
    G[u][v]['weight'] = heuristic(u, v)

# 🔹 가장 가까운 노드 찾기
def find_closest_node(x, y):
    return min(positions.items(), key=lambda item: np.hypot(item[1][0] - x, item[1][1] - y))[0]

# 🔹 A* 거리 및 경로 구하기
def get_astar_distance(x1, y1, x2, y2):
    try:
        start_node = find_closest_node(x1, y1)
        end_node = find_closest_node(x2, y2)
        path = nx.astar_path(G, start_node, end_node, heuristic=heuristic)
        distance = sum(G[u][v]['weight'] for u, v in zip(path[:-1], path[1:]))
        return distance, path
    except Exception:
        return float('inf'), []

# 🔹 배차 로직
def dispatch(taxis: dict, request: dict, call_id: int):
    db = Database()
    
    start_x, start_y = request["start_x"], request["start_y"]
    dest_x, dest_y = request["dest_x"], request["dest_y"]
    required_passengers = request["passenger_count"]

    available = [
        taxi for taxi in taxis.values()
        if (
            taxi.state == "ready" and
            taxi.battery >= 60.0 and
            taxi.max_passengers >= required_passengers
        )
    ]

    if not available:
        print("⚠️ 배차 실패: 사용 가능한 택시 없음")
        return None

    taxi_distances = {}
    for taxi in available:
        dist, path = get_astar_distance(taxi.location[0], taxi.location[1], start_x, start_y)
        taxi_distances[taxi.vehicle_id] = (dist, path)
        print(f"🚕 택시 {taxi.vehicle_id}: 예상 거리={dist:.1f}, 경로={path}")

    best_taxi = min(available, key=lambda t: taxi_distances[t.vehicle_id][0])
    best_dist, best_path = taxi_distances[best_taxi.vehicle_id]

    dispatches_id = db.execute_insert(
        """INSERT INTO `Dispatches` (
            call_id, 
            vehicle_id,                  
            end_point,
            fare
        ) VALUES (
            %s, %s, %s, %s
        )""",
        (
            call_id,
            best_taxi.vehicle_id,
            f"{dest_x} , {dest_y}",
            0
        )
    )

    best_taxi.dispatch(
        start_x=start_x,
        start_y=start_y,
        start_node=find_closest_node(start_x, start_y),
        dest_x=dest_x,
        dest_y=dest_y,
        destination_node=find_closest_node(dest_x, dest_y),
        passenger_count=request["passenger_count"],
        passenger_id=request["passenger_id"],
        dispatches_id=dispatches_id
    )

    print(f"✅ 택시 {best_taxi.vehicle_id} 배차됨 → 거리: {best_dist:.1f} → 경로: {best_path}")

    return best_taxi
