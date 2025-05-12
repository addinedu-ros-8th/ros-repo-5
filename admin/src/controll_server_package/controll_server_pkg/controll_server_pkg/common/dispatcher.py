import networkx as nx
import numpy as np

# 🔹 노드 위치 정보
positions = {
    "A": (192, 170), "B": (621, 88), "C": (1130, 64), "D": (1500, 115), "E": (1150, 159),
    "F": (282, 244), "G": (631, 216), "H": (1138, 274), "I": (1387, 328),
    "J": (252, 472), "K": (780, 502), "L": (904, 515), "M": (1466, 560),
    "N": (267, 694), "O": (598, 734), "P": (1114, 817), "Q": (1448, 770),
    "R": (210, 944), "S": (555, 984), "T": (575, 857), "U": (1156, 941), "V": (1512, 904)
}

# 🔹 휴리스틱 함수
def heuristic(n1, n2):
    x1, y1 = positions[n1]
    x2, y2 = positions[n2]
    return np.hypot(x2 - x1, y2 - y1)

# 🔹 그래프 정의
G = nx.DiGraph()
G.add_edges_from([
    ('A', 'R'), ('B', 'A'), ('C', 'B'), ('C', 'E'), ('E', 'K'), ('D', 'C'),
    ('F', 'G'), ('G', 'K'), ('H', 'I'), ('I', 'M'),
    ('J', 'F'), ('K', 'O'), ('L', 'B'), ('L', 'H'), ('M', 'Q'), ('Q', 'P'),
    ('N', 'J'), ('O', 'N'), ('P', 'L'), ('P', 'L'), ('K', 'U'),
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
def assign_taxi(taxis: dict, request: dict):
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
        dist, path = get_astar_distance(taxi.location[0], taxi.location[1], dest_x, dest_y)
        taxi_distances[taxi.taxi_id] = (dist, path)
        print(f"🚕 택시 {taxi.taxi_id}: 예상 거리={dist:.1f}, 경로={path}")

    best_taxi = min(available, key=lambda t: taxi_distances[t.taxi_id][0])
    best_dist, best_path = taxi_distances[best_taxi.taxi_id]

    best_taxi.assign(
        start_x=request["start_x"],
        start_y=request["start_y"],
        dest_x=dest_x,
        dest_y=dest_y,
        passenger_count=request["passenger_count"],
        client_id=request["client_id"]
    )

    print(f"✅ 택시 {best_taxi.taxi_id} 배차됨 → 거리: {best_dist:.1f} → 경로: {best_path}")

    return best_taxi
