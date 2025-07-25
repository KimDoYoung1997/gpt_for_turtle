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

# occupancy grid map 이미지를 PNG로 변환하여 인코딩
base64_image = convert_pgm_to_base64_png("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.pgm")

# YAML 파일 내용 읽기
yaml_content = read_yaml_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.yaml")

# Graph JSON 파일 내용 읽기
graph_data = read_json_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/icp_poses.json")

response = client.chat.completions.create(
    model="gpt-4.1-2025-04-14",
    messages=[
        {
            "role": "user", 
            "content": f"""당신은 로봇 경로계획 전문가입니다. 다음 정보를 바탕으로 **단계별로 체계적으로 분석**해주세요.

**맵 정보 (YAML):**
```yaml
{yaml_content}
```

**ICP 정제된 Door/Elevator 정보 (JSON):**
```json
{json.dumps(graph_data, indent=2)}
```

**📊 데이터 구조 설명:**
- `door`: 감지된 문들의 배열 (각각 centroid 중심점과 hull_points 외곽점들 포함)
- `elevator`: 감지된 엘리베이터들의 배열
- `centroid`: 각 객체의 중심점 좌표 [x, y, z]
- `hull_points`: 객체의 실제 형태를 나타내는 외곽 점들

**⚠️ 중요한 고려사항:**
1. **ICP 후처리 데이터**: 이미 어느 정도 정제되었지만 여전히 중복이나 오차 가능성 존재
2. **중심점 기반 분석**: centroid를 주요 위치로 활용하되, hull_points로 실제 크기/형태 확인
3. **방 구획 정확성**: 시각적으로 연결된 공간을 별개 방으로 오인하지 말 것
4. **접근성 고려**: door/elevator 주변의 접근 가능한 영역 확인 필요

---

**🔍 STEP 1: 맵 구조 분석**
먼저 occupancy grid map을 자세히 관찰하여:
- 전체적인 건물 레이아웃 파악 (L자형, 직사각형 등)
- 명확하게 구분되는 방/구역/엘리베이터 실내영역들 식별  
- 복도와 방의 연결 구조 분석
- 장애물(검은색)과 자유공간(흰색) 분포 확인
- 미탐색 영역(회색) 위치 파악
- 엘리베이터문과 연결된 밀폐된 영역은 엘리베이터 내부로 판단
- 엘리베이터 내부 영역까지 occupancy grid map에 포함되어 있음
**🔍 STEP 2: Door/Elevator 위치 분석**
ICP 정제된 데이터를 바탕으로:
- 각 door/elevator의 centroid 위치를 맵상에서 확인
- hull_points를 통해 실제 크기와 방향 파악
- 지나치게 가까운 객체들이 있다면 동일 객체로 판단하여 통합
- 맵의 벽이나 장애물과의 관계 분석
- 실제로 통행 가능한 door/elevator만 선별

**🔍 STEP 3: 구역 정의 및 연결성 분석**  
정제된 door/elevator를 바탕으로:
- 각 방/구역을 자유공간의 연결성으로 정의
- Door를 통해 연결되는 구역들 간의 관계 파악
- Elevator가 있는 구역의 특성 분석 (층간 이동 가능성) 
- 로봇이 물리적으로 이동 가능한 경로 식별
- 막다른 길이나 접근 불가능한 영역 확인

**🔍 STEP 4: Strategic Waypoint 생성**
최종적으로:
- 각 구역의 중심점 또는 전략적 위치 선정
- Door/Elevator 접근을 위한 대기점 설정 (centroid 근처의 자유공간)
- 구역 간 이동 시 중간 경유점 배치
- A* 알고리즘에 입력할 주요 노드들의 맵 좌표(x,y) 리스트 생성
- **각 waypoint에 명확한 이름(네이밍) 부여**
- **waypoint 간 연결성(connections) 명시**

**맵 좌표계:**
- 해상도: {yaml_content.split('resolution: ')[1].split()[0]}m/pixel  
- 원점: {yaml_content.split('origin: ')[1].split()[0]}

**⚠️ 최종 출력 형식:**
다음과 같은 표 형식으로 체계적인 waypoint 네트워크를 제공해주세요:

```
## ✅ **최종 Waypoint 리스트**

| 번호 | 이름                     | 좌표(x,y)            | 역할/설명                         | 연결 가능 노드 |
|-----|-------------------------|---------------------|-----------------------------------|---------------|
| 1   | hub_center              | (x1, y1)            | 중앙 허브/복도 중심              | 2,3,4,5       |
| 2   | door1_entry             | (x2, y2)            | 문1 접근점 (구역A 입구)           | 1, 6          |
| 3   | door2_entry             | (x3, y3)            | 문2 접근점 (구역B 입구)           | 1, 7          |
| 4   | elevator1_entry         | (x4, y4)            | 엘리베이터1 접근점                | 1, 8          |
| 5   | zoneA_center            | (x5, y5)            | 구역A 내부 중심                   | 2             |
| 6   | zoneB_center            | (x6, y6)            | 구역B 내부 중심                   | 3             |
| 7   | elevator1_interior      | (x7, y7)            | 엘리베이터1 내부                  | 4             |
```

**추가 정보:**
- **총 waypoint 개수**: N개
- **주요 연결 경로**: hub_center를 중심으로 한 스타 네트워크 구조
- **로봇 접근성**: 모든 waypoint는 0.6m 로봇이 안전하게 접근 가능
- **A* 적용**: 각 waypoint 간 직선 거리 기반 휴리스틱 적용 가능

**네이밍 규칙:**
- `hub_*`: 중앙 허브/메인 복도
- `door*_entry`: 문 접근점
- `elevator*_entry/interior`: 엘리베이터 관련
- `zone*_center`: 각 구역 내부 중심
```

각 단계별로 상세히 분석하고, 최종적으로 **명확한 네이밍과 연결성을 포함한** 실용적인 waypoint 네트워크를 제공해주세요.
"""
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

print("=== SYSTEMATIC HIGH-LEVEL PATH PLANNING ANALYSIS ===")
print(response.choices[0].message.content)