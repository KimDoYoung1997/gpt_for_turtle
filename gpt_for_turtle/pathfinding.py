import math
import heapq
from collections import defaultdict

def find_path_a_star(start_node, goal_node, waypoints, connections):
    """A* 알고리즘을 이용한 waypoint 간 경로 탐색"""
    if start_node == goal_node:
        return [start_node]
    
    # 휴리스틱 함수 (유클리드 거리)
    def heuristic(node1, node2):
        wp1, wp2 = waypoints[node1], waypoints[node2]
        return math.sqrt((wp1['x'] - wp2['x'])**2 + (wp1['y'] - wp2['y'])**2)
    
    # 실제 거리 계산
    def distance(node1, node2):
        wp1, wp2 = waypoints[node1], waypoints[node2]
        return math.sqrt((wp1['x'] - wp2['x'])**2 + (wp1['y'] - wp2['y'])**2)
    
    open_set = {start_node}
    came_from = {}
    g_score = defaultdict(lambda: float('inf'))
    g_score[start_node] = 0
    f_score = defaultdict(lambda: float('inf'))
    f_score[start_node] = heuristic(start_node, goal_node)
    
    while open_set:
        current = min(open_set, key=lambda x: f_score[x])
        
        if current == goal_node:
            # 경로 재구성
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_node)
            return path[::-1]
        
        open_set.remove(current)
        
        # 연결된 노드들 탐색
        if current in connections:
            for neighbor in connections[current]:
                tentative_g = g_score[current] + distance(current, neighbor)
                
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal_node)
                    open_set.add(neighbor)
    
    return []  # 경로를 찾을 수 없음

def grid_astar(grid_map, start_world, goal_world, robot_radius=0.3, verbose=False):
    """
    실제 occupancy grid를 고려한 A* 경로 계획 (조용한 모드)
    """
    # 월드 좌표를 그리드 좌표로 변환
    start_grid = grid_map.world_to_grid(start_world[0], start_world[1])
    goal_grid = grid_map.world_to_grid(goal_world[0], goal_world[1])
    
    # 시작점과 목표점이 유효한 자유공간인지 확인
    if not grid_map.is_free_cell(start_grid[0], start_grid[1]):
        if verbose:
            print(f"❌ Start position is in occupied space!")
        return []
    
    if not grid_map.is_free_cell(goal_grid[0], goal_grid[1]):
        if verbose:
            print(f"❌ Goal position is in occupied space!")
        return []
    
    # A* 알고리즘 구현
    def heuristic(node, goal):
        return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)
    
    open_set = []
    heapq.heappush(open_set, (0, start_grid))
    came_from = {}
    g_score = {start_grid: 0}
    f_score = {start_grid: heuristic(start_grid, goal_grid)}
    
    visited_nodes = 0
    max_iterations = grid_map.width * grid_map.height  # 무한루프 방지
    
    while open_set and visited_nodes < max_iterations:
        current = heapq.heappop(open_set)[1]
        visited_nodes += 1
        
        if current == goal_grid:
            # 경로 재구성
            path_grid = []
            while current in came_from:
                path_grid.append(current)
                current = came_from[current]
            path_grid.append(start_grid)
            path_grid.reverse()
            
            # 그리드 경로를 월드 좌표로 변환
            path_world = []
            for grid_x, grid_y in path_grid:
                world_x, world_y = grid_map.grid_to_world(grid_x, grid_y)
                path_world.append((world_x, world_y))
            
            return path_world
        
        for neighbor_x, neighbor_y, move_cost in grid_map.get_neighbors(current[0], current[1]):
            neighbor = (neighbor_x, neighbor_y)
            tentative_g = g_score[current] + move_cost * grid_map.resolution
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal_grid) * grid_map.resolution
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return [] 