from openai import OpenAI
import os
import json
import math

# 모든 분리된 모듈들 import
from occupancygridmap import OccupancyGridMap
from pathfinding import find_path_a_star, grid_astar
from waypoint_parser import parse_waypoints_from_gpt_response, determine_region, print_waypoint_table
from utils import convert_pgm_to_base64_png, read_yaml_content, read_json_content
from visualization import visualize_path, visualize_waypoints_only

# matplotlib 폰트 설정 (한글 문제 해결)
# plt.rcParams['font.family'] = ['DejaVu Sans', 'Liberation Sans', 'sans-serif']
# plt.rcParams['axes.unicode_minus'] = False

api_key = os.getenv('OPENAI_API_KEY')

client = OpenAI(api_key=api_key)

# occupancy grid map 이미지를 PNG로 변환하여 인코딩
base64_image = convert_pgm_to_base64_png("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.pgm")

# YAML 파일 내용 읽기
yaml_content = read_yaml_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.yaml")

# Graph JSON 파일 내용 읽기
graph_data = read_json_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/obj_poses.json")

# YAML에서 resolution 값 안전하게 추출
try:
    import yaml
    yaml_data = yaml.safe_load(yaml_content)
    resolution_value = yaml_data.get('resolution', 0.05)
    origin_value = yaml_data.get('origin', [0, 0, 0])
except:
    resolution_value = 0.05
    origin_value = [0, 0, 0]

# 1단계: PGM 맵 이미지 구조 분석
print("🔍 1단계: 맵 이미지 구조 분석 중...")
map_analysis_response = client.chat.completions.create(
    model="gpt-4.1",  # 비전 기능을 지원하는 GPT-4.1 사용 (더 비용 효율적)
    messages=[
        {
            "role": "system",
            "content": """당신은 Occupancy Grid Map 이미지 분석 전문가입니다.

**분석 목표:**
- PGM 맵 이미지의 기하학적 구조 파악
- 방, 복도, 연결 통로 식별
- 각 영역의 위치와 크기를 월드 좌표계로 측정

**출력 형식:**
## 맵 구조 분석 결과

**전체 레이아웃:**
- 건물 형태: [설명]
- 주요 영역 개수: [개수]
- 맵 크기: 가로 X m, 세로 Y m

**방(Room) 분석:**
- 방1: 중심좌표(x,y), 크기(가로x세로)m, 형태 특징
- 방2: ...

**복도(Corridor) 분석:**  
- 복도1: 중심좌표(x,y), 크기, 연결 방향
- 복도2: ...

**연결성:**
- 영역 간 연결 통로 위치
- 접근 가능한 경로

**중요사항:**
- 흰색 = 자유공간, 검은색 = 장애물, 회색 = 미탐지 영역(플래닝할때 사용하지 말 것)
- 방 = 3면이 벽으로 둘러싸인 넓은 공간
- 복도 = 긴 통로형 공간
- 모든 좌표는 월드 좌표계로 표현 (해상도와 원점 고려)"""
        },
        {
            "role": "user", 
            "content": [
                {
                    "type": "text",
                    "text": f"""제공된 Occupancy Grid Map 이미지를 분석하여 건물 구조를 파악해주세요.

**맵 메타데이터:**
- 해상도: {resolution_value}m/픽셀
- 원점: {origin_value}
- 이미지: 흰색=자유공간, 검은색=벽, 회색=미탐지 영역(플래닝할때 사용하지 말 것)

**분석 요청:**
1. 전체 건물 레이아웃 파악
2. 방과 복도 구분 및 중심좌표 계산 (월드 좌표계)
3. 각 영역의 크기와 연결성 분석
4. 위의 출력 형식에 맞춰 상세 분석 결과 제공

월드 좌표계를 고려하여 정확한 분석을 제공해주세요."""
                },
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

map_structure_analysis = map_analysis_response.choices[0].message.content
print("✅ 맵 구조 분석 완료!")
print("map_structure_analysis: ", map_structure_analysis)
# 2단계: 종합 분석 및 Waypoint 생성
print("🚀 2단계: 종합 분석 및 Waypoint 네트워크 생성 중...")
response = client.chat.completions.create(
    model="gpt-4.1",  # 비전 기능 지원하는 GPT-4.1 사용
    messages=[
        {
            "role": "system",
            "content": 
                        """
                        당신은 로봇 경로계획 전문가입니다. 

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
                        - 0.6m 로봇 접근성(Inflation radius) 확보
                        - 데이터의 잠재적 중복/오차 처리
                        - 시각적 연결 공간의 올바른 구역 분류
                        """
        },
        {
            "role": "user",
            "content": 
                        """
                        간단한 맵 데이터를 분석하여 Strategic Waypoint 네트워크를 생성해주세요.

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

                        **요청:** 위 데이터와 맵 이미지를 분석하여 waypoint 네트워크를 설계해주세요.
                        """
        },
        {
            "role": "assistant",
            "content": 
                    """
                    ## 🔍 **단계별 분석 결과**

                    **STEP 1: 맵 구조 분석**
                    - 건물 레이아웃: 직사각형 형태의 단일층 구조
                    - 구분되는 영역: 메인 복도 1개, 방 1개, 엘리베이터 구역 1개
                    - 자유공간: 흰색 영역으로 로봇 이동 가능
                    - 장애물: 검은색 벽체로 물리적 차단
                    - 미탐지 영역: 회색 영역으로 로봇 이동 불가

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
                    - **A* 적용**: 각 waypoint 간 직선 거리 기반 휴리스틱 적용 가능
                    """
        },
        {
            "role": "user",
            "content": 
                    f"""다음 실제 맵 데이터를 분석하여 Strategic Waypoint 네트워크를 생성해주세요.

                    **맵 구조 분석 결과 (1단계):**
                    ```
                    {map_structure_analysis}
                    ```

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
                    1. **맵 구조 분석 활용**: 1단계에서 분석된 방/복도 정보를 기반으로 waypoint 배치
                    2. **월드 좌표 활용**: 1단계에서 이미 월드 좌표로 분석된 결과를 직접 활용
                    3. **JSON 데이터 매칭**: door/elevator centroid가 분석된 방/복도 중 어디에 위치하는지 확인
                    4. **접근성 고려**: door/elevator 주변의 접근 가능한 영역 확인 필요
                    5. **waypoint 위치**: occupancy grid map 자유공간에 배치하되 벽에서 최소 0.3m 이상 떨어진 위치

                    **네이밍 규칙:**
                    - `hub_*`: 중앙 허브/메인 복도로, 다수의 문 또는 엘리베이터와 연결되어 방(room) 또는 엘리베이터 내부(center)로 이동할 수 있는 중심 위치
                    - `door*_entry`: 문 접근점
                    - `elevator*_entry`: 엘리베이터 접근점
                    - `room*_center`: 각 방 내부 중심으로, door와 연결되어 있음
                    - `elevator*_center`: 엘리베이터 중심으로, elevator와 연결되어 있음

                    위의 예시와 동일한 형식으로 단계별 분석과 최종 waypoint 테이블을 제공해주세요.
                    맵 구조 분석 결과와 JSON 데이터를 종합하여 정확한 waypoint 네트워크를 설계해주세요.
                    """
        }
    ],
    temperature=0.0
)

# GPT 응답에서 waypoint 정보 파싱 (조용한 모드)
waypoints, connections = parse_waypoints_from_gpt_response(response.choices[0].message.content)

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