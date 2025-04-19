import heapq
import math
import time

# Data kota dan koordinatnya
cities = {
    "A": (0, 0),
    "B": (2, 1),
    "C": (4, 2),
    "D": (5, 5),
    "E": (1, 4)
}

# Data jalan antar kota (grafik)
roads = {
    "A": ["B", "E"],
    "B": ["A", "C"],
    "C": ["B", "D"],
    "D": ["C"],
    "E": ["A", "D"]
}

# Fungsi euclidean heuristic 
def euclidean(a, b):
    (x1, y1) = cities[a]
    (x2, y2) = cities[b]
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

# A* 
def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = {city: float('inf') for city in cities}
    g_score[start] = 0

    f_score = {city: float('inf') for city in cities}
    f_score[start] = euclidean(start, goal)

    visited_nodes = 0

    while open_set:
        current = heapq.heappop(open_set)[1]
        visited_nodes += 1

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, visited_nodes

        for neighbor in roads[current]:
            tentative_g = g_score[current] + euclidean(current, neighbor)
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + euclidean(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None, visited_nodes

# GBFS
def gbfs(start, goal):
    open_set = []
    heapq.heappush(open_set, (euclidean(start, goal), start))
    
    came_from = {}
    visited = set()
    visited_nodes = 0

    while open_set:
        current = heapq.heappop(open_set)[1]
        visited_nodes += 1

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, visited_nodes

        visited.add(current)

        for neighbor in roads[current]:
            if neighbor not in visited:
                came_from[neighbor] = current
                heapq.heappush(open_set, (euclidean(neighbor, goal), neighbor))
                visited.add(neighbor)

    return None, visited_nodes

# Start dan goal 
start = "A"
goal = "D"

# menjalankan GBFS
start_time = time.time()
gbfs_path, gbfs_nodes = gbfs(start, goal)
end_time = time.time()
gbfs_time = (end_time - start_time) * 1000  

# menjalankan A*
start_time = time.time()
astar_path, astar_nodes = a_star(start, goal)
end_time = time.time()
astar_time = (end_time - start_time) * 1000  

# hasil
print("\n=== SHORTEST PATH RESULTS ===")
print("GBFS  : ", " -> ".join(gbfs_path), f"(Time: {gbfs_time:.3f} ms, Nodes: {gbfs_nodes})")
print("A*    : ", " -> ".join(astar_path), f"(Time: {astar_time:.3f} ms, Nodes: {astar_nodes})")

# perbandingan waktu eksekusi
print("\n=== COMPARISON BASED ON EXECUTION TIME (ms) ===")
print(f"{'Algorithm':<10} | {'Time (ms)':>12}")
print(f"{'-'*25}")
print(f"{'GBFS':<10} | {gbfs_time:>12.3f}")
print(f"{'A*':<10} | {astar_time:>12.3f}")
 
# perbandingan jumlah node
print("\n=== COMPARISON BASED ON NUMBER OF NODES VISITED ===")
print(f"{'Algorithm':<10} | {'Visited Nodes':>20}")
print(f"{'-'*34}")
print(f"{'GBFS':<10} | {gbfs_nodes:>20}")
print(f"{'A*':<10} | {astar_nodes:>20}")
