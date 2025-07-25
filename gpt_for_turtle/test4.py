from openai import OpenAI
import base64
from PIL import Image
import io
import json
import os
import re
import math
from collections import defaultdict, deque
import numpy as np
import yaml
from heapq import heappush, heappop

api_key = os.getenv('OPENAI_API_KEY')

client = OpenAI(api_key=api_key)

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

# PGM 파일을 numpy 배열로 파싱
def parse_pgm_to_grid(pgm_path):
    """PGM 파일을 occupancy grid (numpy array)로 변환"""
    with open(pgm_path, 'rb') as f:
        # 헤더 읽기
        header = f.readline().decode('ascii').strip()
        if header != 'P5':
            raise ValueError("지원하지 않는 PGM 형식입니다")
        
        # 주석 건너뛰기
        line = f.readline().decode('ascii').strip()
        while line.startswith('#'):
            line = f.readline().decode('ascii').strip()
        
        # 크기 읽기
        width, height = map(int, line.split())
        
        # 최대값 읽기
        max_val = int(f.readline().decode('ascii').strip())
        
        # 이미지 데이터 읽기
        image_data = f.read()
        
    # numpy 배열로 변환
    grid = np.frombuffer(image_data, dtype=np.uint8)
    grid = grid.reshape((height, width))
    
    return grid, width, height

# YAML 파일에서 맵 메타데이터 읽기
def parse_yaml_metadata(yaml_path):
    """YAML 파일에서 resolution, origin 정보 읽기"""
    with open(yaml_path, 'r') as f:
        metadata = yaml.safe_load(f)
    
    resolution = metadata.get('resolution', 0.05)
    origin = metadata.get('origin', [0.0, 0.0, 0.0])
    
    return resolution, origin

# 실제 좌표를 grid 인덱스로 변환
def world_to_grid(x, y, resolution, origin):
    """실제 좌표를 grid map 인덱스로 변환"""
    grid_x = int((x - origin[0]) / resolution)
    grid_y = int((y - origin[1]) / resolution)
    return grid_x, grid_y

# Grid map에서 A* 경로 탐색
def astar_grid_search(grid, start_grid, goal_grid, obstacle_threshold=100):
    """
    Occupancy grid에서 A* 경로 탐색
    - grid: numpy array occupancy grid (0=free, 255=occupied)
    - obstacle_threshold: 이 값 이하면 자유공간으로 판단
    """
    height, width = grid.shape
    start_x, start_y = start_grid
    goal_x, goal_y = goal_grid
    
    # 경계 체크
    if (start_x < 0 or start_x >= width or start_y < 0 or start_y >= height or
        goal_x < 0 or goal_x >= width or goal_y < 0 or goal_y >= height):
        return []
    
    # 시작점이나 목표점이 장애물인 경우
    if grid[start_y, start_x] > obstacle_threshold or grid[goal_y, goal_x] > obstacle_threshold:
        return []
    
    # A* 알고리즘
    def heuristic(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    open_set = [(0, (start_x, start_y))]
    came_from = {}
    g_score = {(start_x, start_y): 0}
    f_score = {(start_x, start_y): heuristic((start_x, start_y), (goal_x, goal_y))}
    
    # 8방향 이동
    directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
    
    while open_set:
        current = heappop(open_set)[1]
        
        if current == (goal_x, goal_y):
            # 경로 재구성
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append((start_x, start_y))
            return path[::-1]
        
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # 경계 체크
            if (neighbor[0] < 0 or neighbor[0] >= width or 
                neighbor[1] < 0 or neighbor[1] >= height):
                continue
            
            # 장애물 체크
            if grid[neighbor[1], neighbor[0]] > obstacle_threshold:
                continue
            
            # 대각선 이동 시 코너 체크
            if dx != 0 and dy != 0:
                if (grid[current[1] + dy, current[0]] > obstacle_threshold or
                    grid[current[1], current[0] + dx] > obstacle_threshold):
                    continue
            
            tentative_g = g_score[current] + heuristic(current, neighbor)
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, (goal_x, goal_y))
                heappush(open_set, (f_score[neighbor], neighbor))
    
    return []  # 경로 없음

# Waypoint 연결 검증
def validate_waypoint_connections(waypoints, connections, grid, resolution, origin):
    """GPT가 생성한 waypoint 연결이 실제 grid map에서 가능한지 검증"""
    print("\n=== WAYPOINT CONNECTION VALIDATION ===")
    
    valid_connections = {}
    invalid_connections = []
    
    for node_id, connected_nodes in connections.items():
        if node_id not in waypoints:
            continue
            
        valid_connections[node_id] = []
        
        for connected_id in connected_nodes:
            if connected_id not in waypoints:
                continue
                
            # 두 waypoint의 grid 좌표 계산
            start_wp = waypoints[node_id]
            goal_wp = waypoints[connected_id]
            
            start_grid = world_to_grid(start_wp['x'], start_wp['y'], resolution, origin)
            goal_grid = world_to_grid(goal_wp['x'], goal_wp['y'], resolution, origin)
            
            # A* 경로 탐색으로 연결 가능성 검증
            path = astar_grid_search(grid, start_grid, goal_grid)
            
            if path:
                valid_connections[node_id].append(connected_id)
                path_length = len(path) * resolution
                print(f"✅ {start_wp['name']} → {goal_wp['name']}: 연결 가능 (경로 길이: {path_length:.2f}m)")
            else:
                invalid_connections.append((node_id, connected_id, start_wp['name'], goal_wp['name']))
                print(f"❌ {start_wp['name']} → {goal_wp['name']}: 연결 불가능 (장애물 차단)")
    
    return valid_connections, invalid_connections

# 검증된 연결로 waypoint 네트워크 업데이트
def update_connections_with_validation(waypoints, connections, grid, resolution, origin):
    """Grid map 검증을 통해 연결 관계 업데이트"""
    valid_connections, invalid_connections = validate_waypoint_connections(
        waypoints, connections, grid, resolution, origin
    )
    
    if invalid_connections:
        print(f"\n⚠️  총 {len(invalid_connections)}개의 잘못된 연결이 발견되었습니다.")
        print("GPT에게 피드백을 보내서 수정을 요청합니다...")
        
        # GPT에게 피드백 전송
        feedback_prompt = f"""Waypoint 연결 검증 결과 문제가 발견되었습니다.

**검증 실패한 연결들:**
"""
        for start_id, goal_id, start_name, goal_name in invalid_connections:
            feedback_prompt += f"- {start_name} (#{start_id}) → {goal_name} (#{goal_id}): 장애물로 인해 연결 불가능\n"

        feedback_prompt += f"""

**원인 분석:**
- Occupancy grid map에서 해당 waypoint들 사이에 장애물(벽, 가구 등)이 있음
- 직선 거리로는 가까워 보이지만 실제로는 우회 경로가 필요함

**수정 요청:**
1. 검증 실패한 연결들을 제거하고 대안 경로 제시
2. 필요시 중간 waypoint 추가로 우회 경로 생성  
3. 수정된 waypoint 테이블을 동일한 형식으로 제공

**현재 유효한 연결 상태:**
"""
        for node_id, valid_list in valid_connections.items():
            if node_id in waypoints:
                feedback_prompt += f"- {waypoints[node_id]['name']}: {[waypoints[c]['name'] for c in valid_list if c in waypoints]}\n"

        # 멀티턴 대화에 피드백 추가
        messages.append({
            "role": "user",
            "content": feedback_prompt
        })
        
        try:
            # GPT 응답 받기
            response = client.chat.completions.create(
                model="gpt-4.1-2025-04-14",
                messages=messages,
                temperature=0.1
            )
            
            gpt_response = response.choices[0].message.content
            print(f"\n🤖 GPT 피드백 응답:")
            print("=" * 60)
            print(gpt_response)
            print("=" * 60)
            
            # GPT 응답을 메시지 리스트에 추가
            messages.append({
                "role": "assistant",
                "content": gpt_response
            })
            
            # 새로운 waypoint 정보 파싱 시도
            new_waypoints, new_connections = parse_waypoints_from_gpt_response(gpt_response)
            
            if new_waypoints and new_connections:
                print(f"\n🔄 수정된 waypoint 네트워크 적용 중...")
                
                # 재검증
                final_valid_connections, final_invalid_connections = validate_waypoint_connections(
                    new_waypoints, new_connections, grid, resolution, origin
                )
                
                if final_invalid_connections:
                    print(f"⚠️  여전히 {len(final_invalid_connections)}개의 문제가 있습니다. 유효한 연결만 사용합니다.")
                    return new_waypoints, final_valid_connections
                else:
                    print(f"✅ 모든 연결이 검증되었습니다!")
                    return new_waypoints, new_connections
            else:
                print(f"⚠️  GPT 응답 파싱 실패. 유효한 연결만 사용합니다.")
                return waypoints, valid_connections
                
        except Exception as e:
            print(f"❌ GPT 피드백 요청 중 오류: {e}")
            return waypoints, valid_connections
    else:
        print(f"✅ 모든 waypoint 연결이 검증되었습니다!")
        return waypoints, connections

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

# Occupancy grid map 파싱
print("=== LOADING OCCUPANCY GRID MAP ===")
grid, grid_width, grid_height = parse_pgm_to_grid("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.pgm")
resolution, origin = parse_yaml_metadata("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.yaml")

print(f"✅ Grid map 로딩 완료:")
print(f"   - 크기: {grid_width} x {grid_height}")
print(f"   - 해상도: {resolution}m/pixel")
print(f"   - 원점: {origin}")
print(f"   - 자유공간 픽셀: {np.sum(grid <= 100)}")
print(f"   - 장애물 픽셀: {np.sum(grid > 100)}")

# 멀티턴 대화를 위한 메시지 리스트 초기화
messages = [
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
- 시각적 연결 공간의 올바른 구역 분류

**멀티턴 대화 지원:**
- 사용자의 추가 질문이나 요청에 응답
- 기존 분석 결과를 기반으로 한 개선사항 제안
- 경로 계획에 대한 상세 설명 및 대안 제시"""
    },
    {
        "role": "user",
        "content": f"""간단한 맵 데이터를 분석하여 Strategic Waypoint 네트워크를 생성해주세요.

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
]

# 초기 맵 분석 수행
response = client.chat.completions.create(
    model="gpt-4.1-2025-04-14",
    messages=messages,
    temperature=0.0
)

print("=== SYSTEMATIC HIGH-LEVEL PATH PLANNING ANALYSIS ===")
initial_response = response.choices[0].message.content
print(initial_response)

# GPT 응답을 메시지 리스트에 추가
messages.append({
    "role": "assistant",
    "content": initial_response
})

# GPT 응답에서 waypoint 정보 파싱
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

# 경로 계획 메인 함수 (실패 시 자동 리플래닝 포함)
def plan_high_level_path_with_replanning(robot_x, robot_y, goal_x, goal_y, waypoints, connections, max_attempts=3):
    print(f"\n=== HIGH-LEVEL PATH PLANNING ===")
    print(f"🤖 로봇 현재 위치: ({robot_x}, {robot_y})")
    print(f"🎯 목표 위치: ({goal_x}, {goal_y})")
    
    # 현재 위치와 목표 위치에서 가장 가까운 waypoint 찾기
    start_waypoint = determine_region(robot_x, robot_y, waypoints)
    goal_waypoint = determine_region(goal_x, goal_y, waypoints)
    
    print(f"📍 시작 영역: {waypoints[start_waypoint]['name']} (Waypoint {start_waypoint})")
    print(f"📍 목표 영역: {waypoints[goal_waypoint]['name']} (Waypoint {goal_waypoint})")
    
    failure_history = []
    
    for attempt in range(max_attempts):
        print(f"\n🔄 시도 {attempt + 1}/{max_attempts}")
        
        # 경로 탐색
        path = find_path_a_star(start_waypoint, goal_waypoint, waypoints, connections)
        
        if path:
            print(f"\n✅ 계획된 경로 ({len(path)}개 waypoint):")
            for i, node_id in enumerate(path):
                waypoint = waypoints[node_id]
                print(f"   {i+1}. {waypoint['name']} → ({waypoint['x']:.1f}, {waypoint['y']:.1f})")
                
            print(f"\n📋 상세 이동 계획:")
            for i in range(len(path)-1):
                current_wp = waypoints[path[i]]
                next_wp = waypoints[path[i+1]]
                distance = math.sqrt((current_wp['x'] - next_wp['x'])**2 + (current_wp['y'] - next_wp['y'])**2)
                print(f"   Step {i+1}: {current_wp['name']} → {next_wp['name']} (거리: {distance:.1f}m)")
                
            return path, waypoints, connections
        else:
            # 경로 계획 실패
            failure_info = {
                'attempt': attempt + 1,
                'robot_pos': (robot_x, robot_y),
                'goal_pos': (goal_x, goal_y),
                'start_waypoint': start_waypoint,
                'goal_waypoint': goal_waypoint,
                'start_name': waypoints[start_waypoint]['name'],
                'goal_name': waypoints[goal_waypoint]['name']
            }
            failure_history.append(failure_info)
            
            print(f"❌ 경로 계획 실패! (시도 {attempt + 1})")
            
            if attempt < max_attempts - 1:
                print(f"🔄 GPT에게 리플래닝 요청 중...")
                
                # GPT에게 실패 정보와 함께 리플래닝 요청
                replanning_prompt = f"""경로 계획이 실패했습니다. 리플래닝이 필요합니다.

**실패 정보:**
- 로봇 위치: ({robot_x}, {robot_y})
- 목표 위치: ({goal_x}, {goal_y})
- 시작 영역: {waypoints[start_waypoint]['name']} (Waypoint {start_waypoint})
- 목표 영역: {waypoints[goal_waypoint]['name']} (Waypoint {goal_waypoint})
- 시도 횟수: {attempt + 1}

**현재 Waypoint 연결 상태:**
{get_connection_summary(waypoints, connections)}

**실패 원인 분석 및 해결책:**
1. 두 영역 사이에 연결된 경로가 없는지 확인
2. 중간 연결점(waypoint)이 필요한지 분석  
3. 기존 waypoint의 연결 관계 수정이 필요한지 검토

**요청:**
- 위 실패 사례를 분석하여 개선된 waypoint 네트워크를 제안해주세요
- 기존 waypoint 테이블 형식과 동일하게 제공해주세요
- 연결성 문제를 해결할 수 있는 구체적인 방안을 제시해주세요"""

                # 멀티턴 대화에 실패 정보 추가
                messages.append({
                    "role": "user",
                    "content": replanning_prompt
                })
                
                try:
                    # GPT 응답 받기
                    response = client.chat.completions.create(
                        model="gpt-4.1-2025-04-14",
                        messages=messages,
                        temperature=0.1
                    )
                    
                    gpt_response = response.choices[0].message.content
                    print(f"\n🤖 GPT 리플래닝 응답:")
                    print("=" * 60)
                    print(gpt_response)
                    print("=" * 60)
                    
                    # GPT 응답을 메시지 리스트에 추가
                    messages.append({
                        "role": "assistant",
                        "content": gpt_response
                    })
                    
                    # 새로운 waypoint 정보 파싱 시도
                    new_waypoints, new_connections = parse_waypoints_from_gpt_response(gpt_response)
                    
                    if new_waypoints and new_connections:
                        print(f"\n🔄 새로운 waypoint 네트워크 적용 중...")
                        waypoints.update(new_waypoints)
                        connections.update(new_connections)
                        
                        # 업데이트된 영역 재계산
                        start_waypoint = determine_region(robot_x, robot_y, waypoints)
                        goal_waypoint = determine_region(goal_x, goal_y, waypoints)
                        
                        print(f"✅ Waypoint 업데이트 완료")
                        print(f"   - 총 waypoint: {len(waypoints)}개")
                        print(f"   - 연결 관계: {len(connections)}개")
                    else:
                        print(f"⚠️  새로운 waypoint 파싱 실패, 기존 데이터로 재시도")
                        
                except Exception as e:
                    print(f"❌ GPT 리플래닝 요청 중 오류: {e}")
            else:
                print(f"❌ 최대 시도 횟수 ({max_attempts})에 도달했습니다.")
                print("\n📊 실패 기록:")
                for i, fail in enumerate(failure_history):
                    print(f"   시도 {fail['attempt']}: {fail['start_name']} → {fail['goal_name']}")
    
    return [], waypoints, connections

# 연결 상태 요약 정보 생성
def get_connection_summary(waypoints, connections):
    summary = []
    for node_id, waypoint in sorted(waypoints.items()):
        connected = connections.get(node_id, [])
        connected_names = [waypoints[c]['name'] for c in connected if c in waypoints]
        summary.append(f"   {node_id}: {waypoint['name']} → 연결: {connected_names}")
    return "\n".join(summary)

# 기존 경로 계획 함수 (호환성 유지)
def plan_high_level_path(robot_x, robot_y, goal_x, goal_y, waypoints, connections):
    print(f"\n=== HIGH-LEVEL PATH PLANNING ===")
    print(f"🤖 로봇 현재 위치: ({robot_x}, {robot_y})")
    print(f"🎯 목표 위치: ({goal_x}, {goal_y})")
    
    # 현재 위치와 목표 위치에서 가장 가까운 waypoint 찾기
    start_waypoint = determine_region(robot_x, robot_y, waypoints)
    goal_waypoint = determine_region(goal_x, goal_y, waypoints)
    
    print(f"📍 시작 영역: {waypoints[start_waypoint]['name']} (Waypoint {start_waypoint})")
    print(f"📍 목표 영역: {waypoints[goal_waypoint]['name']} (Waypoint {goal_waypoint})")
    
    # 경로 탐색
    path = find_path_a_star(start_waypoint, goal_waypoint, waypoints, connections)
    
    if path:
        print(f"\n✅ 계획된 경로 ({len(path)}개 waypoint):")
        for i, node_id in enumerate(path):
            waypoint = waypoints[node_id]
            print(f"   {i+1}. {waypoint['name']} → ({waypoint['x']:.1f}, {waypoint['y']:.1f})")
            
        print(f"\n📋 상세 이동 계획:")
        for i in range(len(path)-1):
            current_wp = waypoints[path[i]]
            next_wp = waypoints[path[i+1]]
            distance = math.sqrt((current_wp['x'] - next_wp['x'])**2 + (current_wp['y'] - next_wp['y'])**2)
            print(f"   Step {i+1}: {current_wp['name']} → {next_wp['name']} (거리: {distance:.1f}m)")
            
    else:
        print("❌ 경로를 찾을 수 없습니다!")
        
    return path

# GPT와의 멀티턴 대화 함수
def multi_turn_conversation():
    print("\n" + "="*60)
    print("🗣️  MULTI-TURN CONVERSATION WITH GPT")
    print("="*60)
    print("맵 분석에 대한 추가 질문이나 요청을 입력하세요.")
    print("예시: '특정 영역에 waypoint 추가해줘', '경로 최적화 방법 알려줘'")
    print("종료하려면 'quit' 입력")
    
    while True:
        try:
            user_input = input("\n💬 질문: ").strip()
            
            if user_input.lower() in ['quit', '종료', 'exit']:
                print("👋 대화를 종료합니다.")
                break
            
            if not user_input:
                continue
            
            # 사용자 메시지 추가
            messages.append({
                "role": "user", 
                "content": user_input
            })
            
            # GPT 응답 받기
            response = client.chat.completions.create(
                model="gpt-4.1-2025-04-14",
                messages=messages,
                temperature=0.1
            )
            
            gpt_response = response.choices[0].message.content
            print(f"\n🤖 GPT: {gpt_response}")
            
            # GPT 응답을 메시지 리스트에 추가
            messages.append({
                "role": "assistant",
                "content": gpt_response
            })
            
        except KeyboardInterrupt:
            print("\n\n👋 대화를 종료합니다.")
            break
        except Exception as e:
            print(f"❌ 오류 발생: {e}")

# GPT 응답에서 waypoint 정보 파싱
print("\n=== PARSING WAYPOINT DATA ===")
waypoints, connections = parse_waypoints_from_gpt_response(initial_response)

print(f"✅ 파싱된 waypoint 개수: {len(waypoints)}")
print(f"✅ 연결 정보: {len(connections)}개 노드")

# Grid map 기반 연결 검증 및 업데이트
print(f"✅ Occupancy grid map 기반 연결 검증 시작...")
waypoints, connections = update_connections_with_validation(waypoints, connections, grid, resolution, origin)

# 파싱된 데이터 확인
print("\n📋 최종 검증된 Waypoint 정보:")
for node_id, waypoint in sorted(waypoints.items()):
    connected = connections.get(node_id, [])
    print(f"   {node_id}: {waypoint['name']} ({waypoint['x']:.1f}, {waypoint['y']:.1f}) → 연결: {connected}")

# Grid map 기반 경로 계획 함수 (추가 검증 포함)
def plan_high_level_path_with_grid_validation(robot_x, robot_y, goal_x, goal_y, waypoints, connections, grid, resolution, origin):
    print(f"\n=== HIGH-LEVEL PATH PLANNING WITH GRID VALIDATION ===")
    print(f"🤖 로봇 현재 위치: ({robot_x}, {robot_y})")
    print(f"🎯 목표 위치: ({goal_x}, goal_y})")
    
    # 현재 위치와 목표 위치에서 가장 가까운 waypoint 찾기
    start_waypoint = determine_region(robot_x, robot_y, waypoints)
    goal_waypoint = determine_region(goal_x, goal_y, waypoints)
    
    print(f"📍 시작 영역: {waypoints[start_waypoint]['name']} (Waypoint {start_waypoint})")
    print(f"📍 목표 영역: {waypoints[goal_waypoint]['name']} (Waypoint {goal_waypoint})")
    
    # 시작점과 첫 번째 waypoint 간 연결 검증
    start_grid = world_to_grid(robot_x, robot_y, resolution, origin)
    start_wp_grid = world_to_grid(waypoints[start_waypoint]['x'], waypoints[start_waypoint]['y'], resolution, origin)
    start_connection_path = astar_grid_search(grid, start_grid, start_wp_grid)
    
    if not start_connection_path:
        print(f"❌ 로봇 위치에서 시작 waypoint로 연결 불가능!")
        return []
    
    # 마지막 waypoint와 목표점 간 연결 검증
    goal_grid = world_to_grid(goal_x, goal_y, resolution, origin)
    goal_wp_grid = world_to_grid(waypoints[goal_waypoint]['x'], waypoints[goal_waypoint]['y'], resolution, origin)
    goal_connection_path = astar_grid_search(grid, goal_wp_grid, goal_grid)
    
    if not goal_connection_path:
        print(f"❌ 목표 waypoint에서 목표 지점으로 연결 불가능!")
        return []
    
    # High-level waypoint 경로 탐색
    path = find_path_a_star(start_waypoint, goal_waypoint, waypoints, connections)
    
    if path:
        print(f"\n✅ 계획된 경로 ({len(path)}개 waypoint):")
        total_distance = 0
        
        # 로봇 → 첫 번째 waypoint 거리
        start_distance = len(start_connection_path) * resolution
        total_distance += start_distance
        print(f"   0. 로봇 위치 → {waypoints[path[0]]['name']} (거리: {start_distance:.2f}m)")
        
        for i, node_id in enumerate(path):
            waypoint = waypoints[node_id]
            print(f"   {i+1}. {waypoint['name']} → ({waypoint['x']:.1f}, {waypoint['y']:.1f})")
            
        # Waypoint 간 거리 계산 및 검증
        print(f"\n📋 상세 이동 계획 (Grid 검증됨):")
        for i in range(len(path)-1):
            current_wp = waypoints[path[i]]
            next_wp = waypoints[path[i+1]]
            
            # Grid 기반 실제 경로 거리
            current_grid = world_to_grid(current_wp['x'], current_wp['y'], resolution, origin)
            next_grid = world_to_grid(next_wp['x'], next_wp['y'], resolution, origin)
            grid_path = astar_grid_search(grid, current_grid, next_grid)
            
            if grid_path:
                grid_distance = len(grid_path) * resolution
                total_distance += grid_distance
                print(f"   Step {i+1}: {current_wp['name']} → {next_wp['name']} (실제 거리: {grid_distance:.2f}m)")
            else:
                print(f"   Step {i+1}: {current_wp['name']} → {next_wp['name']} (⚠️ 연결 문제 있음)")
        
        # 마지막 waypoint → 목표점 거리
        goal_distance = len(goal_connection_path) * resolution  
        total_distance += goal_distance
        print(f"   {len(path)+1}. {waypoints[path[-1]]['name']} → 목표 지점 (거리: {goal_distance:.2f}m)")
        
        print(f"\n🎯 총 예상 거리: {total_distance:.2f}m")
        
    else:
        print("❌ 경로를 찾을 수 없습니다!")
        
    return path

# 사용자 입력을 받아 경로 계획 실행 (기본 버전)
def interactive_path_planning():
    print("\n" + "="*50)
    print("🚀 INTERACTIVE PATH PLANNING")
    print("="*50)
    
    while True:
        try:
            print("\n좌표 입력 (종료하려면 'quit' 입력):")
            
            # 현재 위치 입력
            robot_input = input("🤖 로봇 현재 위치 (x y): ").strip()
            if robot_input.lower() == 'quit':
                break
                
            robot_x, robot_y = map(float, robot_input.split())
            
            # 목표 위치 입력  
            goal_input = input("🎯 목표 위치 (x y): ").strip()
            if goal_input.lower() == 'quit':
                break
                
            goal_x, goal_y = map(float, goal_input.split())
            
            # 경로 계획 실행
            path = plan_high_level_path(robot_x, robot_y, goal_x, goal_y, waypoints, connections)
            
            # 계속할지 물어보기
            continue_input = input("\n다른 경로를 계획하시겠습니까? (y/n): ").strip().lower()
            if continue_input != 'y':
                break
                
        except ValueError:
            print("❌ 잘못된 입력 형식입니다. 숫자를 공백으로 구분해서 입력해주세요.")
        except KeyboardInterrupt:
            print("\n\n👋 프로그램을 종료합니다.")
            break

# 사용자 입력을 받아 경로 계획 실행 (리플래닝 기능 포함)
def interactive_path_planning_with_replanning():
    print("\n" + "="*50)
    print("🚀 INTERACTIVE PATH PLANNING WITH AUTO-REPLANNING")
    print("="*50)
    print("💡 경로 계획 실패 시 자동으로 GPT와 대화하여 리플래닝합니다")
    
    while True:
        try:
            print("\n좌표 입력 (종료하려면 'quit' 입력):")
            
            # 현재 위치 입력
            robot_input = input("🤖 로봇 현재 위치 (x y): ").strip()
            if robot_input.lower() == 'quit':
                break
                
            robot_x, robot_y = map(float, robot_input.split())
            
            # 목표 위치 입력  
            goal_input = input("🎯 목표 위치 (x y): ").strip()
            if goal_input.lower() == 'quit':
                break
                
            goal_x, goal_y = map(float, goal_input.split())
            
            # 리플래닝 포함 경로 계획 실행
            path, updated_waypoints, updated_connections = plan_high_level_path_with_replanning(
                robot_x, robot_y, goal_x, goal_y, waypoints, connections, max_attempts=3
            )
            
            # waypoint 데이터 업데이트
            waypoints.update(updated_waypoints) 
            connections.update(updated_connections)
            
            if path:
                print(f"\n🎉 최종 경로 계획 성공!")
            else:
                print(f"\n😞 경로 계획 최종 실패")
                
                # 수동 대화 모드 제안
                retry_input = input("\n💬 GPT와 직접 대화해서 문제를 해결해보시겠습니까? (y/n): ").strip().lower()
                if retry_input == 'y':
                    manual_conversation_mode(robot_x, robot_y, goal_x, goal_y)
            
            # 계속할지 물어보기
            continue_input = input("\n다른 경로를 계획하시겠습니까? (y/n): ").strip().lower()
            if continue_input != 'y':
                break
                
        except ValueError:
            print("❌ 잘못된 입력 형식입니다. 숫자를 공백으로 구분해서 입력해주세요.")
        except KeyboardInterrupt:
            print("\n\n👋 프로그램을 종료합니다.")
            break

# 수동 대화 모드 (특정 경로 계획 문제 해결용)
def manual_conversation_mode(robot_x=None, robot_y=None, goal_x=None, goal_y=None):
    print("\n" + "="*60)
    print("🗣️  MANUAL CONVERSATION MODE")
    print("="*60)
    
    if robot_x is not None:
        print(f"🎯 문제가 된 경로: ({robot_x}, {robot_y}) → ({goal_x}, {goal_y})")
        
    print("💬 GPT와 자유롭게 대화하여 문제를 해결하세요.")
    print("예시: '두 지점 사이에 중간 waypoint 추가해줘', 'waypoint 연결 관계 수정해줘'")
    print("종료하려면 'quit' 입력")
    
    while True:
        try:
            user_input = input("\n💬 질문: ").strip()
            
            if user_input.lower() in ['quit', '종료', 'exit']:
                print("👋 대화 모드를 종료합니다.")
                break
            
            if not user_input:
                continue
            
            # 사용자 메시지 추가
            messages.append({
                "role": "user", 
                "content": user_input
            })
            
            # GPT 응답 받기
            response = client.chat.completions.create(
                model="gpt-4.1-2025-04-14",
                messages=messages,
                temperature=0.1
            )
            
            gpt_response = response.choices[0].message.content
            print(f"\n🤖 GPT: {gpt_response}")
            
            # GPT 응답을 메시지 리스트에 추가
            messages.append({
                "role": "assistant",
                "content": gpt_response
            })
            
            # waypoint 업데이트 시도
            new_waypoints, new_connections = parse_waypoints_from_gpt_response(gpt_response)
            if new_waypoints and new_connections:
                apply_updates = input("\n🔄 새로운 waypoint가 감지되었습니다. 적용하시겠습니까? (y/n): ").strip().lower()
                if apply_updates == 'y':
                    waypoints.update(new_waypoints)
                    connections.update(new_connections)
                    print("✅ Waypoint 업데이트 완료!")
                    
                    # 다시 경로 계획 시도 제안
                    if robot_x is not None:
                        retry_planning = input("🔄 업데이트된 waypoint로 경로 계획을 다시 시도하시겠습니까? (y/n): ").strip().lower()
                        if retry_planning == 'y':
                            path = plan_high_level_path(robot_x, robot_y, goal_x, goal_y, waypoints, connections)
                            if path:
                                print("🎉 경로 계획 성공!")
                                break
                            else:
                                print("😞 여전히 경로 계획 실패")
            
        except KeyboardInterrupt:
            print("\n\n👋 대화 모드를 종료합니다.")
            break
        except Exception as e:
            print(f"❌ 오류 발생: {e}")

# 예시 실행
print("\n" + "="*50)
print("🧪 EXAMPLE PATH PLANNING")
print("="*50)

# 예시 1: 사용자가 제시한 케이스
plan_high_level_path(1, 5, 8, 4, waypoints, connections)

# 예시 2: 다른 케이스
plan_high_level_path(4.5, -4.5, -1.5, -0.4, waypoints, connections)

# 메인 메뉴 업데이트
def main_menu():
    print("\n" + "="*60)
    print("🎯 ROBOT PATH PLANNING SYSTEM WITH AUTO-REPLANNING")
    print("="*60)
    print("1. 대화형 경로 계획 + 자동 리플래닝 (Recommended)")
    print("2. 기본 대화형 경로 계획 (Basic)")
    print("3. GPT와 자유 대화 (Free Conversation)")
    print("4. 종료 (Exit)")
    
    while True:
        try:
            choice = input("\n선택 (1-4): ").strip()
            
            if choice == '1':
                interactive_path_planning_with_replanning()
            elif choice == '2':
                interactive_path_planning()
            elif choice == '3':
                multi_turn_conversation()
            elif choice == '4':
                print("👋 프로그램을 종료합니다.")
                break
            else:
                print("❌ 1, 2, 3, 4 중에서 선택해주세요.")
                
        except KeyboardInterrupt:
            print("\n\n👋 프로그램을 종료합니다.")
            break

# 프로그램 실행
if __name__ == "__main__":
    main_menu()