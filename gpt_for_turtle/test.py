from openai import OpenAI
import base64
from PIL import Image
import io
import json

client = OpenAI()

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

# 맵 정보 파싱
def parse_map_info(yaml_content):
    lines = yaml_content.strip().split('\n')
    resolution = float([line.split(': ')[1] for line in lines if 'resolution:' in line][0])
    origin_str = [line.split(': ')[1] for line in lines if 'origin:' in line][0]
    origin = [float(x.strip('[]')) for x in origin_str.split(', ')]
    return resolution, origin

# occupancy grid map 이미지를 PNG로 변환하여 인코딩
base64_image = convert_pgm_to_base64_png("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.pgm")

# YAML 파일 내용 읽기
yaml_content = read_yaml_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.yaml")

# ICP poses JSON 파일 내용 읽기
graph_data = read_json_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/icp_poses.json")

# 맵 정보 파싱
resolution, origin = parse_map_info(yaml_content)

# 맵 크기 정보 (PGM 헤더에서 가져와야 하지만 여기서는 하드코딩)
map_width_pixels = 358
map_height_pixels = 224

# 맵의 실제 좌표 범위 계산
map_x_min = origin[0]
map_x_max = origin[0] + map_width_pixels * resolution
map_y_min = origin[1] 
map_y_max = origin[1] + map_height_pixels * resolution

print("=== 로봇 경로계획 시스템 ===")
print(f"맵 정보: {map_width_pixels}x{map_height_pixels} pixels, 해상도: {resolution}m/pixel")
print(f"맵 좌표 범위: X[{map_x_min:.2f} ~ {map_x_max:.2f}], Y[{map_y_min:.2f} ~ {map_y_max:.2f}]")
print()

# 사용자 입력 받기
start_x = float(input("시작 위치 X 좌표를 입력하세요: "))
start_y = float(input("시작 위치 Y 좌표를 입력하세요: "))
goal_x = float(input("목적지 X 좌표를 입력하세요: "))
goal_y = float(input("목적지 Y 좌표를 입력하세요: "))

print(f"\n입력된 경로: ({start_x}, {start_y}) → ({goal_x}, {goal_y})")

response = client.chat.completions.create(
    model="gpt-4.1",
    messages=[
        {
            "role": "user", 
            "content": f"""당신은 로봇 경로계획 전문가입니다. 다음 정보를 바탕으로 **실용적이고 효율적인 경로계획**을 수행해주세요.

**맵 정보 (YAML):**
```yaml
{yaml_content}
```

**ICP Door/Elevator 정보:**
```json
{json.dumps(graph_data, indent=2)}
```

**맵 경계 정보:**
- 맵 크기: {map_width_pixels} x {map_height_pixels} pixels
- 해상도: {resolution} m/pixel
- 원점: {origin}
- **실제 좌표 범위**: X[{map_x_min:.2f} ~ {map_x_max:.2f}], Y[{map_y_min:.2f} ~ {map_y_max:.2f}]

**로봇 사양:**
- **로봇 크기**: 0.6m (반지름 0.3m)
- **최소 통로 폭**: 0.8m 이상 필요
- **벽과의 최소 거리**: 0.4m 이상 유지

**경로계획 요청:**
- **시작 위치**: ({start_x}, {start_y})
- **목적지**: ({goal_x}, {goal_y})

**🔍 분석 단계:**

**STEP 1: 위치 유효성 검증**
- 시작 위치 ({start_x}, {start_y})가 맵 경계 X[{map_x_min:.2f}~{map_x_max:.2f}], Y[{map_y_min:.2f}~{map_y_max:.2f}] 내부에 있는지 확인
- 목적지 ({goal_x}, {goal_y})가 맵 경계 내부에 있는지 확인
- **⚠️ 중요**: occupancy grid map에서 색상별 영역 확인:
  * **흰색(자유공간)**: 로봇 이동 가능
  * **검은색(장애물)**: ❌ 로봇 이동 절대 불가능 - 벽, 가구 등 물리적 장애물
  * **회색(미탐색)**: 불확실한 영역, 가급적 피해야 함
- **로봇 크기(0.6m)를 고려하여** 검은색 장애물로부터 충분한 거리(0.4m 이상)가 확보되는지 확인
- 시작점이나 목적지가 검은색 영역에 있다면 **즉시 불가능 판정** 및 대안 위치 제시

**STEP 2: 직접 경로 분석**  
- 시작점에서 목적지까지 **직선 경로**가 가능한지 먼저 확인
- **중요**: 직선 경로상에 **검은색 장애물이 하나라도 있으면 직선 경로 불가능**
- 로봇 크기(0.6m)를 고려하여 경로 주변 0.3m 범위 내에도 검은색 영역이 없는지 확인
- 만약 직선 경로가 가능하다면 중간 waypoint 최소화

**STEP 3: 우회 경로 및 필수 경유 지점 식별**
- 직선 경로가 검은색 장애물로 인해 불가능한 경우에만 우회 경로 계획
- Door/elevator의 centroid 좌표 활용하되, **해당 지점이 흰색(자유공간)에 위치하는지** 반드시 확인
- **검은색 영역을 절대 통과하지 않는** 우회 경로 설계
- 불필요한 회전 포인트나 중간 경유점 제거

**STEP 4: 최적화된 Waypoint 생성**
- **최소한의 waypoint**로 효율적인 경로 구성
- 각 waypoint는 로봇 크기를 고려하여 안전하게 접근 가능한 위치
- 연속된 waypoint 간 직선 이동이 가능하도록 배치

**⚠️ 중요한 제약조건:**
1. **검은색 영역 절대 통과 금지**: occupancy grid map의 검은색(장애물) 영역은 물리적으로 통과 불가능
2. **로봇 크기 고려**: 0.6m 로봇이 검은색 장애물로부터 0.4m 이상 떨어져서 이동
3. **회전 포인트 금지**: 불필요한 중간 회전 지점 생성하지 말 것
4. **효율성 우선**: 검은색 장애물을 피하면서도 가장 짧은 경로 추구
5. **최소 waypoint**: 꼭 필요한 지점만 포함 (시작점 → 필수 경유점 → 목적지)

**최종 출력 형식:**
```yaml
status: "feasible/infeasible/needs_adjustment"
analysis:
  start_valid: true/false  # 시작점이 흰색(자유공간)에 있는지
  goal_valid: true/false   # 목적지가 흰색(자유공간)에 있는지
  start_in_obstacle: true/false  # 시작점이 검은색(장애물)에 있는지
  goal_in_obstacle: true/false   # 목적지가 검은색(장애물)에 있는지
  direct_path_possible: true/false  # 직선 경로상에 검은색 장애물 없는지
  robot_size_considered: true
waypoints:
  - [x1, y1]  # 시작점 (흰색 영역 확인됨)
  - [x2, y2]  # door 접근점 (필요시에만, 흰색 영역)
  - [x3, y3]  # 목적지 (흰색 영역 확인됨)
total_waypoints: N개
estimated_distance: "X.X미터"
obstacles_avoided: "검은색 장애물 회피 전략"
notes: "경로 특이사항"
```

**효율적이고 실용적인 경로**를 중심으로 분석해주세요. **검은색 장애물은 절대 통과할 수 없으므로** 반드시 회피하고, 불필요한 중간점은 제거하고 로봇이 실제로 안전하게 이동할 수 있는 경로만 제시해주세요."""
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
)

print("\n=== 경로계획 분석 결과 ===")
print(response.choices[0].message.content)