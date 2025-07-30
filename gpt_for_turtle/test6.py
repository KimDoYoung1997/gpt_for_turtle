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

# 통합된 공간 분석 및 Waypoint 생성
print("🔍 Multi-Agent Waypoint Generation 시작...")
print("   📝 Generator: Waypoint 네트워크 생성")
print("   🔍 Critic: 품질 평가 및 개선 제안")

def generator_gpt(iteration=0, critic_feedback=None):
    """Generator GPT: Waypoint 네트워크 생성"""
    
    feedback_context = ""
    if critic_feedback and iteration > 0:
        feedback_context = f"""
**🔍 CRITIC 피드백 (반복 {iteration}):**
```
{critic_feedback}
```

위 피드백을 반영하여 waypoint 네트워크를 개선해주세요.
"""

    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {
                "role": "system",
                "content": """당신은 **Waypoint Generator** 전문가입니다.

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

**핵심 원칙:**
1. **정확한 Room 중심**: grid map에서 3면이 벽으로 둘러싸인 공간의 기하학적 중심
2. **안전한 접근점**: Door/Elevator 근처의 충돌 없는 접근 위치
3. **효율적 연결성**: 최소 경로로 모든 공간 접근 가능

**고려사항:**
- 0.6m 로봇 접근성(Inflation radius) 확보
- 데이터의 잠재적 중복/오차 처리
- 시각적 연결 공간의 올바른 구역 분류

**출력 형식:**
## 🔍 **단계별 분석 결과**
[상세 분석 과정]

## ✅ **최종 Waypoint 리스트** 
| 번호 | 이름 | 좌표(x,y) | 역할/설명 | 연결 가능 노드 |
|-----|------|----------|-----------|---------------|
[waypoint 테이블]"""
            },
            {
                "role": "user",
                "content": f"""다음 데이터를 분석하여 Strategic Waypoint 네트워크를 생성해주세요.

{feedback_context}

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
0. ""맵 구조 분석**
    - 건물 레이아웃: 직사각형 형태의 단일층 구조
    - 구분되는 영역: 메인 복도 1개, 방 1개, 엘리베이터 구역 1개
    - 자유공간: 흰색 영역으로 로봇 이동 가능
    - 장애물: 검은색 벽체로 물리적 차단
    - 미탐지 영역: 회색 영역으로 로봇 이동 불가
1. **데이터 후처리**: 이미 어느 정도 정제되었지만 여전히 중복이나 오차 가능성 존재
2. **중심점 기반 분석**: centroid를 주요 위치로 활용하되, hull_points로 실제 크기/형태 확인
3. **방 구획 정확성**: 시각적으로 연결된 공간을 별개 방으로 오인하지 말 것
4. **접근성 고려**: door/elevator 주변의 접근 가능한 영역 확인 필요
5. **waypoint 위치**: occupancy grid map 폐곡선 내부(자유공간)에 배치
                    - 자유공간: 흰색 영역으로 로봇 이동 가능
                    - 장애물: 검은색 벽체로 물리적 차단
                    - 미탐지 영역: 회색 영역으로 로봇 이동 불가
**네이밍 규칙:**
- `hub_*`: 중앙 허브/메인 복도로, 다수의 문 또는 엘리베이터와 연결되어 방(room) 또는 엘리베이터 내부(center)로 이동할 수 있는 중심 위치
- `door*_entry`: 문 접근점
- `elevator*_entry`: 엘리베이터 접근점
- `room*_center`: 각 방 내부 중심으로, door와 연결되어 있음, grid map으로 3면이 벽이 있는 공간의 중심
- `elevator*_center`: 엘리베이터 중심으로, elevator와 연결되어 있음, grid map으로 3면이 벽이 있는 공간의 중심

위의 형식으로 단계별 분석과 최종 waypoint 테이블을 제공해주세요."""
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
        temperature=0.1
    )
    
    return response.choices[0].message.content

def critic_gpt(waypoint_content):
    """Critic GPT: Waypoint 품질 평가 및 개선 제안"""
    
    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {
                "role": "system", 
                "content": """당신은 **Waypoint Quality Critic** 전문가입니다.

**역할**: Generator가 생성한 waypoint 네트워크의 품질을 엄격하게 평가하고 개선 방안 제시

**평가 기준:**
1. **Room 중심점 정확성 (매우 중요!)**:
   - Room 중심이 정말로 3면이 벽으로 둘러싸인 공간의 기하학적 중심인가?
   - 벽에서 충분히 멀리 떨어져 있어 로봇이 안전하게 회전할 수 있는가?
   - 시각적으로 해당 Room의 실제 중심부에 위치하는가?
   
2. **Door/Elevator 연결 정확성**:
   - Door centroid 위치와 실제 접근점이 논리적으로 일치하는가?
   - Door entry waypoint가 복도 쪽에 적절히 배치되었는가?
   - Elevator entry waypoint가 대기 공간에 안전하게 위치하는가?
   
3. **공간 구획의 정확성**:
   - 시각적으로 연결된 공간을 별개 방으로 잘못 나눈 경우 경고
   - 복도와 방의 경계가 명확하게 구분되었는가?
   - Hub waypoint가 실제로 여러 공간을 연결하는 중심 역할을 하는가?
   
4. **연결성 및 접근성**:
   - 모든 waypoint가 occupancy grid에서 자유공간에 위치하는가?
   - waypoint 간 연결이 물리적으로 가능한 경로인가?
   - 0.6m 로봇이 실제로 접근 및 통과할 수 있는 위치인가?

**특별 주의사항:**
- **Room 중심점 오류**: 벽 근처나 장애물 주변에 배치된 경우 즉시 지적
- **공간 오분류**: 하나의 큰 공간을 여러 방으로 잘못 나눈 경우 경고
- **접근 불가**: Occupancy Grid에서 점유된 영역에 waypoint가 위치한 경우 문제 제기

**출력 형식:**
## 📊 **품질 평가 결과**

**✅ 잘된 점:**
- [좋은 점들 나열]

**⚠️ 문제점 및 개선사항:**
- [구체적인 문제점과 개선 방법]

**📋 종합 점수: X/10**

**🔄 추가 반복 필요성: [YES/NO]**

엄격하고 정확한 평가를 통해 waypoint 품질을 높여주세요."""
            },
            {
                "role": "user",
                "content": f"""다음 Generator가 생성한 waypoint 네트워크를 평가해주세요:

**🗺️ 참고 맵 정보:**
- 해상도: {resolution_value}m/픽셀
- 원점: {origin_value}

**🚪🛗 Door/Elevator 위치:**
```json
{json.dumps(graph_data, indent=2)}
```

**📍 Generator 결과:**
```
{waypoint_content}
```

**🔍 특별 검토 사항:**
1. **Room 중심점 검증**: 각 room*_center waypoint가 정말로 해당 방의 기하학적 중심에 위치하는지 확인
2. **공간 경계 검증**: Door를 기준으로 방과 복도가 올바르게 구분되었는지 확인
3. **접근성 검증**: 모든 waypoint가 자유공간에 위치하고 물리적으로 접근 가능한지 확인
4. **연결성 검증**: waypoint 간 연결이 합리적이고 효율적인지 확인

위 waypoint 네트워크를 엄격하게 평가하고, 문제점이 있다면 구체적인 개선 방안을 제시해주세요."""
            }
        ],
        temperature=0.0
    )
    
    return response.choices[0].message.content

# Multi-Agent Debate 실행
MAX_ITERATIONS = 5
current_iteration = 0
critic_feedback = None

print(f"\n{'='*80}")
print("🤖 MULTI-AGENT WAYPOINT GENERATION")
print(f"{'='*80}")

while current_iteration < MAX_ITERATIONS:
    print(f"\n🔄 반복 {current_iteration + 1}/{MAX_ITERATIONS}")
    print("-" * 50)
    
    # Generator: Waypoint 생성
    print("📝 Generator: Waypoint 네트워크 생성 중...")
    generator_result = generator_gpt(current_iteration, critic_feedback)
    
    # Critic: 품질 평가
    print("🔍 Critic: 품질 평가 중...")
    critic_result = critic_gpt(generator_result)
    
    print(f"\n📊 Critic 평가 결과:")
    print(critic_result)
    
    # 수렴 조건 확인
    if "추가 반복 필요성: NO" in critic_result or "추가 반복 필요성: [NO]" in critic_result:
        print(f"\n✅ 수렴 완료! {current_iteration + 1}번의 반복으로 만족스러운 품질 달성")
        final_result = generator_result
        break
    elif current_iteration == MAX_ITERATIONS - 1:
        print(f"\n⏰ 최대 반복 횟수 도달. 현재 결과를 최종으로 채택")
        final_result = generator_result
        break
    else:
        print(f"\n🔄 개선 필요. 다음 반복에서 피드백 반영...")
        critic_feedback = critic_result
        current_iteration += 1

print(f"\n{'='*80}")
print("🏆 FINAL WAYPOINT NETWORK")
print(f"{'='*80}")
print(final_result)

# 최종 결과 파싱
response_content = final_result

# GPT 응답에서 waypoint 정보 파싱 (조용한 모드)
waypoints, connections = parse_waypoints_from_gpt_response(response_content)

# Waypoint 파싱 검증
if len(waypoints) == 0:
    print("\n❌ ERROR: No waypoints were parsed from GPT response!")
    print("🔍 Debug Information:")
    print(f"   → Response length: {len(response_content)} characters")
    print(f"   → First 500 characters of response:")
    print(response_content[:500])
    print("\n❌ Cannot proceed with path planning without valid waypoints.")
    exit(1)

# Waypoint 테이블 출력
print("\n" + "="*80)
print("📋 GENERATED WAYPOINT NETWORK")
print("="*80)
print_waypoint_table(response_content)

print(f"\n✅ 통합 분석 완료! Parsed {len(waypoints)} waypoints successfully")
print(f"✅ Connection info: {len(connections)} nodes")

# Waypoint 구조 디버그 출력
print(f"\n🔍 Debug: Waypoint keys: {list(waypoints.keys())}")
for key, waypoint in list(waypoints.items())[:3]:  # 처음 3개만 출력
    print(f"   → {key}: {waypoint}")

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
    # Waypoint 가용성 확인
    if len(waypoints) > 0:
        print(f"\n🎯 Ready for interactive path planning with {len(waypoints)} waypoints")
        interactive_path_planning_with_visualization()
    else:
        print("\n❌ No waypoints available for interactive path planning!")
        print("   → Multi-Agent system failed to generate valid waypoints.")
        print("   → Please check GPT responses and parsing logic.")