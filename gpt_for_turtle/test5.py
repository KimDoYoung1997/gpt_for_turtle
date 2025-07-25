from openai import OpenAI
import base64
from PIL import Image
import io
import json
import os
import re
import math
import numpy as np
import yaml
import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import defaultdict, deque

# matplotlib 폰트 설정 (한글 문제 해결)
plt.rcParams['font.family'] = ['DejaVu Sans', 'Liberation Sans', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

api_key = os.getenv('OPENAI_API_KEY')

client = OpenAI(api_key=api_key)

# Occupancy Grid Map 클래스
class OccupancyGridMap:
    def __init__(self, pgm_path, yaml_path):
        self.load_map_data(pgm_path, yaml_path)
    
    def load_map_data(self, pgm_path, yaml_path):
        """PGM과 YAML 파일에서 맵 데이터 로드"""
        # YAML 파일에서 메타데이터 읽기
        with open(yaml_path, 'r') as f:
            self.map_metadata = yaml.safe_load(f)
        
        self.resolution = self.map_metadata['resolution']
        self.origin = self.map_metadata['origin']
        self.occupied_thresh = self.map_metadata.get('occupied_thresh', 0.65)
        self.free_thresh = self.map_metadata.get('free_thresh', 0.25)
        self.negate = self.map_metadata.get('negate', 0)
        
        # PGM 파일에서 grid 데이터 읽기
        image = Image.open(pgm_path)
        self.grid_data = np.array(image)
        
        # grid.py 방식: Y축을 뒤집어서 올바른 월드 좌표계로 변환
        self.grid_data = np.flipud(self.grid_data)
        
        self.height, self.width = self.grid_data.shape
    
    def world_to_grid(self, world_x, world_y):
        """월드 좌표를 그리드 좌표로 변환 (grid.py 방식 적용)"""
        # 월드 좌표에서 맵 원점을 기준으로 한 상대 좌표로 변환
        relative_x = world_x - self.origin[0]
        relative_y = world_y - self.origin[1]
        
        # 픽셀 좌표로 변환
        grid_x = int(relative_x / self.resolution)
        grid_y = int(relative_y / self.resolution)
        
        # grid.py 방식: 이미 flipud()로 Y축을 뒤집었으므로 추가 변환 불필요
        # 단지 범위 체크만 수행
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """그리드 좌표를 월드 좌표로 변환 (grid.py 방식 적용)"""
        # 픽셀 좌표를 월드 좌표로 변환
        world_x = self.origin[0] + grid_x * self.resolution
        world_y = self.origin[1] + grid_y * self.resolution
        
        return world_x, world_y
    
    def is_valid_cell(self, grid_x, grid_y):
        """유효한 그리드 셀인지 확인"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def is_free_cell(self, grid_x, grid_y):
        """자유 공간인지 확인"""
        if not self.is_valid_cell(grid_x, grid_y):
            return False
        
        pixel_value = self.grid_data[grid_y, grid_x]
        
        # negate 처리
        if self.negate:
            pixel_value = 255 - pixel_value
        
        # 0은 점유, 255는 자유공간, 중간값은 미지
        normalized_value = pixel_value / 255.0
        
        return normalized_value > self.free_thresh
    
    def is_occupied_cell(self, grid_x, grid_y):
        """점유 공간인지 확인"""
        if not self.is_valid_cell(grid_x, grid_y):
            return True  # 맵 밖은 점유로 간주
        
        pixel_value = self.grid_data[grid_y, grid_x]
        
        # negate 처리
        if self.negate:
            pixel_value = 255 - pixel_value
        
        normalized_value = pixel_value / 255.0
        
        return normalized_value < (1.0 - self.occupied_thresh)
    
    def get_neighbors(self, grid_x, grid_y):
        """8방향 이웃 셀 반환"""
        neighbors = []
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        for dx, dy in directions:
            nx, ny = grid_x + dx, grid_y + dy
            if self.is_valid_cell(nx, ny) and self.is_free_cell(nx, ny):
                # 대각선 이동의 경우 cost가 더 큼
                cost = math.sqrt(2) if abs(dx) + abs(dy) == 2 else 1.0
                neighbors.append((nx, ny, cost))
        
        return neighbors

# GPT 응답에서 waypoint 정보를 파싱하는 함수
def parse_waypoints_from_gpt_response(gpt_response):
    waypoints = {}
    connections = {}
    
    lines = gpt_response.split('\n')
    parsing_table = False
    
    for line in lines:
        # 테이블 시작 찾기
        if '번호' in line and '이름' in line and '좌표' in line:
            parsing_table = True
            continue
        
        # 테이블 데이터 파싱
        if parsing_table and '|' in line:
            parts = [p.strip() for p in line.split('|')]
            if len(parts) >= 6 and parts[1].isdigit():
                try:
                    node_id = int(parts[1])
                    name = parts[2]
                    
                    # 좌표 파싱 (x, y) 형태
                    coord_match = re.search(r'\(([^,]+),\s*([^)]+)\)', parts[3])
                    if coord_match:
                        x = float(coord_match.group(1))
                        y = float(coord_match.group(2))
                        
                        waypoints[node_id] = {
                            'name': name,
                            'x': x, 
                            'y': y,
                            'description': parts[4]
                        }
                        
                        # 연결 노드 파싱
                        connection_str = parts[5].replace(' ', '')
                        if connection_str and connection_str != '-':
                            connected_nodes = [int(n) for n in connection_str.split(',') if n.isdigit()]
                            connections[node_id] = connected_nodes
                
                except (ValueError, IndexError):
                    continue
    
    return waypoints, connections

# 주어진 좌표가 어떤 영역에 속하는지 판단하는 함수
def determine_region(x, y, waypoints):
    min_distance = float('inf')
    closest_waypoint = None
    
    for node_id, waypoint in waypoints.items():
        distance = math.sqrt((x - waypoint['x'])**2 + (y - waypoint['y'])**2)
        if distance < min_distance:
            min_distance = distance
            closest_waypoint = node_id
    
    return closest_waypoint

# A* 알고리즘을 이용한 경로 탐색
def find_path_a_star(start_node, goal_node, waypoints, connections):
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

# Grid 기반 A* 알고리즘 (조용한 모드)
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

# 경로 시각화 함수
def visualize_path(grid_map, waypoints, connections, full_path, start_pos, goal_pos, waypoint_path=None):
    """A* 경로 계획 결과를 시각화"""
    plt.figure(figsize=(12, 10))
    
    # 1. Occupancy Grid Map 표시
    extent = [
        grid_map.origin[0], 
        grid_map.origin[0] + grid_map.width * grid_map.resolution,
        grid_map.origin[1],
        grid_map.origin[1] + grid_map.height * grid_map.resolution
    ]
    
    plt.imshow(grid_map.grid_data, cmap='gray', 
               extent=extent, origin='lower', alpha=0.8)
    
    # 2. Waypoint들 표시
    waypoint_xs = [wp['x'] for wp in waypoints.values()]
    waypoint_ys = [wp['y'] for wp in waypoints.values()]
    plt.scatter(waypoint_xs, waypoint_ys, c='blue', s=100, marker='s', 
                alpha=0.7, label='Waypoints', zorder=5)
    
    # Waypoint 라벨 표시
    for node_id, wp in waypoints.items():
        plt.annotate(f"{node_id}", (wp['x'], wp['y']), 
                    xytext=(5, 5), textcoords='offset points',
                    fontsize=8, color='blue', weight='bold')
    
    # 3. Waypoint 간 연결선 표시 (점선)
    for node_id, connected_nodes in connections.items():
        if node_id not in waypoints:
            continue
        for connected_id in connected_nodes:
            if connected_id not in waypoints:
                continue
            start_wp = waypoints[node_id]
            end_wp = waypoints[connected_id]
            plt.plot([start_wp['x'], end_wp['x']], 
                    [start_wp['y'], end_wp['y']], 
                    'b--', alpha=0.3, linewidth=1, zorder=2)
    
    # 4. A* 경로 표시
    if full_path and len(full_path) > 1:
        path_xs = [pos[0] for pos in full_path]
        path_ys = [pos[1] for pos in full_path]
        
        # Waypoint 구간별로 다른 색상 사용
        if waypoint_path and len(waypoint_path) > 1:
            colors = ['red', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan']
            current_idx = 0
            
            for i, waypoint_id in enumerate(waypoint_path):
                target_wp = waypoints[waypoint_id]
                # 현재 waypoint까지의 경로 찾기
                next_idx = current_idx
                for j in range(current_idx, len(full_path)):
                    if abs(full_path[j][0] - target_wp['x']) < 0.1 and abs(full_path[j][1] - target_wp['y']) < 0.1:
                        next_idx = j + 1
                        break
                
                if next_idx > current_idx:
                    segment_xs = path_xs[current_idx:next_idx]
                    segment_ys = path_ys[current_idx:next_idx]
                    color = colors[i % len(colors)]
                    plt.plot(segment_xs, segment_ys, color=color, linewidth=2.5, 
                            alpha=0.8, label=f'Segment {i+1}', zorder=4)
                    current_idx = next_idx - 1
            
            # 마지막 구간 (목표점까지)
            if current_idx < len(full_path) - 1:
                segment_xs = path_xs[current_idx:]
                segment_ys = path_ys[current_idx:]
                color = colors[len(waypoint_path) % len(colors)]
                plt.plot(segment_xs, segment_ys, color=color, linewidth=2.5, 
                        alpha=0.8, label=f'Segment {len(waypoint_path)+1}', zorder=4)
        else:
            plt.plot(path_xs, path_ys, 'red', linewidth=2.5, alpha=0.8, 
                    label='A* Path', zorder=4)
    
    # 5. 시작점과 목표점 표시
    plt.scatter(start_pos[0], start_pos[1], c='lime', s=200, marker='o', 
                edgecolors='black', linewidth=2, label='Start', zorder=6)
    plt.scatter(goal_pos[0], goal_pos[1], c='red', s=200, marker='*', 
                edgecolors='black', linewidth=2, label='Goal', zorder=6)
    
    # 6. 범례와 제목
    plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
    plt.title('A* Path Planning Visualization\n(Gray: Occupied, White: Free Space)', fontsize=14, pad=20)
    plt.xlabel('X (m)', fontsize=12)
    plt.ylabel('Y (m)', fontsize=12)
    plt.grid(True, alpha=0.3)
    
    # 축 비율 맞추기
    plt.axis('equal')
    plt.tight_layout()
    
    # 통계 정보 텍스트 박스
    if full_path:
        total_distance = sum(math.sqrt((full_path[i+1][0] - full_path[i][0])**2 + 
                                      (full_path[i+1][1] - full_path[i][1])**2) 
                            for i in range(len(full_path)-1))
        stats_text = f'Path Length: {total_distance:.1f}m\nPath Points: {len(full_path)}'
        if waypoint_path:
            stats_text += f'\nWaypoints: {len(waypoint_path)}'
        
        plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
                fontsize=10, verticalalignment='top', 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.show()

# PGM 파일을 PNG로 변환 후 base64로 인코딩
def convert_pgm_to_base64_png(pgm_path):
    # PGM 파일을 PIL Image로 열기
    image = Image.open(pgm_path)
    
    # PNG 형식으로 변환하여 메모리 버퍼에 저장
    buffer = io.BytesIO()
    image.save(buffer, format='PNG')
    buffer.seek(0)
    
    # base64로 인코딩
    return base64.b64encode(buffer.read()).decode('utf-8')

# YAML 파일 내용 읽기
def read_yaml_content(yaml_path):
    with open(yaml_path, "r") as yaml_file:
        return yaml_file.read()

# JSON 파일 내용 읽기
def read_json_content(json_path):
    with open(json_path, "r") as json_file:
        return json.load(json_file)

# occupancy grid map 이미지를 PNG로 변환하여 인코딩
base64_image = convert_pgm_to_base64_png("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.pgm")

# YAML 파일 내용 읽기
yaml_content = read_yaml_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.yaml")

# Graph JSON 파일 내용 읽기
graph_data = read_json_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/obj_poses.json")

response = client.chat.completions.create(
    model="gpt-4.1-2025-04-14",
    messages=[
        {
            "role": "system",
            "content": """당신은 로봇 경로계획 전문가입니다. 

**전문 분야:**
- Occupancy Grid Map 분석
- SLAM 데이터 해석  
- 로봇 내비게이션 경로 최적화
- Door/Elevator 감지 데이터 후처리

**분석 방법론:**
1. 맵 구조 체계적 분석 (레이아웃, 장애물, 자유공간)
2. Door/Elevator 위치 검증
3. 구역별 연결성 분석
4. Strategic Waypoint 네트워크 생성

**출력 요구사항:**
- 단계별 상세 분석 과정 제시
- 최종 waypoint 테이블 (번호, 이름, 좌표, 역할, 연결노드)
- 명확한 네이밍 규칙과 연결성 정보
- A* 알고리즘 적용 가능한 형태

**고려사항:**
- 0.6m 로봇 접근성 확보
- 데이터의 잠재적 중복/오차 처리
- 시각적 연결 공간의 올바른 구역 분류"""
        },
        {
            "role": "user",
            "content": """간단한 맵 데이터를 분석하여 Strategic Waypoint 네트워크를 생성해주세요.

**맵 정보 (YAML):**
```yaml
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
```

**Door/Elevator 정보 (JSON):**
```json
{
  "door": [
    {
      "centroid": [2.5, 3.0, 0.0],
      "hull_points": [[2.4, 2.8], [2.6, 3.2]]
    }
  ],
  "elevator": [
    {
      "centroid": [8.0, 5.0, 0.0], 
      "hull_points": [[7.8, 4.8], [8.2, 5.2]]
    }
  ]
}
```

**요청:** 위 데이터와 맵 이미지를 분석하여 waypoint 네트워크를 설계해주세요."""
        },
        {
            "role": "assistant",
            "content": """## 🔍 **단계별 분석 결과**

**STEP 1: 맵 구조 분석**
- 건물 레이아웃: 직사각형 형태의 단일층 구조
- 구분되는 영역: 메인 복도 1개, 방 1개, 엘리베이터 구역 1개
- 자유공간: 흰색 영역으로 로봇 이동 가능
- 장애물: 검은색 벽체로 물리적 차단

**STEP 2: Door/Elevator 위치 분석**
- Door1 (2.5, 3.0): 메인 복도에서 방A로 연결하는 출입구
- Elevator1 (8.0, 5.0): 메인 복도에서 접근 가능한 엘리베이터

**STEP 3: 구역 정의 및 연결성**
- 메인 복도: Door1과 Elevator1을 연결하는 중앙 통로
- 방A: Door1을 통해 메인 복도와 연결
- 엘리베이터 내부: Elevator1을 통해 메인 복도와 연결

**STEP 4: Strategic Waypoint 생성**

## ✅ **최종 Waypoint 리스트**

| 번호 | 이름                     | 좌표(x,y)            | 역할/설명                         | 연결 가능 노드 |
|-----|-------------------------|---------------------|-----------------------------------|---------------|
| 1   | hub_center              | (5.0, 4.0)           | 중앙 허브/복도 중심              | 2,3           |
| 2   | door1_entry             | (2.0, 3.0)           | 문1 접근점 (방A 입구)           | 1, 4          |
| 3   | elevator1_entry         | (7.5, 5.0)           | 엘리베이터1 접근점                | 1, 5          |
| 4   | roomA_center            | (1.5, 2.5)           | 방A 내부 중심                   | 2             |
| 5   | elevator1_center        | (8.0, 5.0)           | 엘리베이터1 내부 중심                  | 3             |

**추가 정보:**
- **총 waypoint 개수**: 5개
- **주요 연결 경로**: hub_center를 중심으로 한 스타 네트워크 구조
- **로봇 접근성**: 모든 waypoint는 0.6m 로봇이 안전하게 접근 가능
- **A* 적용**: 각 waypoint 간 직선 거리 기반 휴리스틱 적용 가능"""
        },
        {
            "role": "user",
            "content": f"""다음 실제 맵 데이터를 분석하여 Strategic Waypoint 네트워크를 생성해주세요.

**맵 정보 (YAML):**
```yaml
{yaml_content}
```

**Door/Elevator 정보 (JSON):**
```json
{json.dumps(graph_data, indent=2)}
```

**📊 데이터 구조:**
- `door`: centroid 중심점 + hull_points 외곽점
- `elevator`: centroid 중심점 + hull_points 외곽점  
- `centroid`: 각 객체의 중심점 좌표 [x, y, z]
- `hull_points`: 객체의 실제 형태를 나타내는 외곽 점들

**⚠️ 중요한 고려사항:**
1. **데이터 후처리**: 이미 어느 정도 정제되었지만 여전히 중복이나 오차 가능성 존재
2. **중심점 기반 분석**: centroid를 주요 위치로 활용하되, hull_points로 실제 크기/형태 확인
3. **방 구획 정확성**: 시각적으로 연결된 공간을 별개 방으로 오인하지 말 것
4. **접근성 고려**: door/elevator 주변의 접근 가능한 영역 확인 필요
5. **waypoint 위치**: occupancy grid map 폐곡선 내부(자유공간)에 배치

**네이밍 규칙:**
- `hub_*`: 중앙 허브/메인 복도로, 다수의 문 또는 엘리베이터와 연결되어 방(room) 또는 엘리베이터 내부(center)로 이동할 수 있는 중심 위치
- `door*_entry`: 문 접근점
- `elevator*_entry`: 엘리베이터 접근점
- `room*_center`: 각 방 내부 중심으로, door와 연결되어 있음
- `elevator*_center`: 엘리베이터 중심으로, elevator와 연결되어 있음

위의 예시와 동일한 형식으로 단계별 분석과 최종 waypoint 테이블을 제공해주세요."""
        },
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url", 
                    "image_url": {
                        "url": f"data:image/png;base64,{base64_image}"
                    }
                }
            ]
        }
    ],
    temperature=0.0
)

# GPT 응답에서 waypoint 정보 파싱 (조용한 모드)
waypoints, connections = parse_waypoints_from_gpt_response(response.choices[0].message.content)

# Waypoint 테이블 출력 함수
def print_waypoint_table(gpt_response):
    """GPT 응답에서 waypoint 테이블 부분만 추출해서 출력"""
    lines = gpt_response.split('\n')
    printing = False
    
    for line in lines:
        # 테이블 시작 찾기
        if '최종 Waypoint' in line or ('번호' in line and '이름' in line and '좌표' in line):
            printing = True
            print(line)
            continue
        
        # 테이블 끝 찾기 (빈 줄이나 다른 섹션 시작)
        if printing and (line.strip() == '' or line.startswith('**')):
            if '추가 정보' in line or '총 waypoint' in line:
                print(line)
                continue
            else:
                break
        
        # 테이블 내용 출력
        if printing:
            print(line)

# Waypoint 테이블 출력
print("\n" + "="*80)
print("📋 GENERATED WAYPOINT NETWORK")
print("="*80)
print_waypoint_table(response.choices[0].message.content)

print(f"\n✅ Parsed {len(waypoints)} waypoints successfully")
print(f"✅ Connection info: {len(connections)} nodes")

# Occupancy Grid Map 로드 (조용한 모드)
try:
    grid_map = OccupancyGridMap(
        "/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.pgm",
        "/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.yaml"
    )
    
    # 조용한 검증 (검증만 수행하고 출력하지 않음)
    def validate_waypoints_quiet(waypoints, connections, grid_map):
        """GPT가 생성한 waypoint들이 실제로 접근 가능한지 조용히 검증"""
        invalid_waypoints = []
        invalid_connections = []
        
        # 각 waypoint가 자유공간에 있는지 확인
        for node_id, waypoint in waypoints.items():
            grid_x, grid_y = grid_map.world_to_grid(waypoint['x'], waypoint['y'])
            if not grid_map.is_free_cell(grid_x, grid_y):
                invalid_waypoints.append((node_id, waypoint['name'], waypoint['x'], waypoint['y']))
        
        # 연결된 waypoint 간의 경로가 실제로 갈 수 있는지 확인
        for node_id, connected_nodes in connections.items():
            if node_id not in waypoints:
                continue
            for connected_id in connected_nodes:
                if connected_id not in waypoints:
                    continue
                start_pos = (waypoints[node_id]['x'], waypoints[node_id]['y'])
                goal_pos = (waypoints[connected_id]['x'], waypoints[connected_id]['y'])
                path = grid_astar(grid_map, start_pos, goal_pos)
                if not path:
                    invalid_connections.append((
                        node_id, waypoints[node_id]['name'],
                        connected_id, waypoints[connected_id]['name']
                    ))
        
        return len(invalid_waypoints) == 0 and len(invalid_connections) == 0
    
    is_valid = validate_waypoints_quiet(waypoints, connections, grid_map)
    
    if is_valid:
        print(f"🎉 All waypoints validated successfully!")
    else:
        print(f"⚠️  Some waypoints may need adjustment")
        
except Exception as e:
    print(f"❌ Error loading map: {e}")
    grid_map = None

# 사용자 친화적인 대화형 경로 계획 함수
def interactive_path_planning_with_visualization():
    """시작점과 목적지를 입력받아 A* 경로 계획을 수행하고 즉시 시각화를 보여주는 함수"""
    print("\n" + "="*60)
    print("🚀 INTERACTIVE A* PATH PLANNING WITH VISUALIZATION")
    print("="*60)
    print("Enter start and goal positions to see A* path planning results!")
    print("The visualization will show waypoints, connections, and the calculated path.")
    print("Type 'quit' to exit.")
    
    if grid_map is None:
        print("❌ Grid map is not loaded. Cannot perform interactive path planning.")
        return
    
    path_count = 0
    
    while True:
        try:
            print(f"\n{'-'*40}")
            print(f"Path Planning #{path_count + 1}")
            print(f"{'-'*40}")
            
            # 시작점 입력
            start_input = input("🤖 Enter START position (x y): ").strip()
            if start_input.lower() == 'quit':
                break
                
            start_coords = list(map(float, start_input.split()))
            if len(start_coords) != 2:
                print("❌ Please enter exactly two numbers (x y)")
                continue
            start_x, start_y = start_coords
            
            # 목적지 입력  
            goal_input = input("🎯 Enter GOAL position (x y): ").strip()
            if goal_input.lower() == 'quit':
                break
                
            goal_coords = list(map(float, goal_input.split()))
            if len(goal_coords) != 2:
                print("❌ Please enter exactly two numbers (x y)")
                continue
            goal_x, goal_y = goal_coords
            
            print(f"\n📍 Planning path from ({start_x}, {start_y}) to ({goal_x}, {goal_y})")
            
            # A* 경로 계획 수행
            print(f"\n🔄 Computing A* path...")
            
            # 1. High-level waypoint 경로 찾기
            start_waypoint = determine_region(start_x, start_y, waypoints)
            goal_waypoint = determine_region(goal_x, goal_y, waypoints)
            
            print(f"   Starting area: {waypoints[start_waypoint]['name']} (Waypoint {start_waypoint})")
            print(f"   Goal area: {waypoints[goal_waypoint]['name']} (Waypoint {goal_waypoint})")
            
            high_level_path = find_path_a_star(start_waypoint, goal_waypoint, waypoints, connections)
            
            if not high_level_path:
                print("❌ No high-level path found between waypoints!")
                continue
            
            # 2. 상세 Grid 기반 경로 계획
            full_path = [(start_x, start_y)]
            current_pos = (start_x, start_y)
            
            for i, waypoint_id in enumerate(high_level_path):
                target_waypoint = waypoints[waypoint_id]
                target_pos = (target_waypoint['x'], target_waypoint['y'])
                
                segment_path = grid_astar(grid_map, current_pos, target_pos)
                
                if not segment_path:
                    print(f"❌ Grid path planning failed for segment {i+1}")
                    break
                
                full_path.extend(segment_path[1:])  # 첫 번째 점 제외하고 추가
                current_pos = target_pos
            
            # 최종 목표점까지
            if (goal_x, goal_y) != current_pos:
                final_segment = grid_astar(grid_map, current_pos, (goal_x, goal_y))
                if final_segment:
                    full_path.extend(final_segment[1:])
                else:
                    print(f"❌ Final segment path planning failed!")
                    continue
            
            if full_path:
                total_distance = sum(math.sqrt((full_path[i+1][0] - full_path[i][0])**2 + 
                                              (full_path[i+1][1] - full_path[i][1])**2) 
                                    for i in range(len(full_path)-1))
                
                print(f"\n✅ Path planning successful!")
                print(f"   📏 Total distance: {total_distance:.1f}m")
                print(f"   📍 Path points: {len(full_path)}")
                print(f"   🗺️  Waypoints used: {len(high_level_path)}")
                
                # 즉시 시각화 표시
                print(f"\n📊 Displaying path visualization...")
                try:
                    visualize_path(grid_map, waypoints, connections, full_path, 
                                  (start_x, start_y), (goal_x, goal_y), high_level_path)
                except Exception as e:
                    print(f"⚠️  Visualization error: {e}")
                
                path_count += 1
            else:
                print("❌ Path planning failed!")
            
            # 계속할지 물어보기
            print(f"\n" + "-"*40)
            continue_input = input("Continue with another path? (y/n): ").strip().lower()
            if continue_input not in ['y', 'yes']:
                break
                
        except ValueError:
            print("❌ Invalid input format. Please enter numbers separated by spaces.")
        except KeyboardInterrupt:
            print("\n\n👋 Exiting interactive mode.")
            break
    
    print(f"\n🎉 Interactive session completed. Planned {path_count} paths.")

# 대화형 경로 계획 실행
if __name__ == "__main__":
    interactive_path_planning_with_visualization()