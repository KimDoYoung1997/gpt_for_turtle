from openai import OpenAI
import base64
from PIL import Image
import io
import json
import os
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

print("=== SYSTEMATIC HIGH-LEVEL PATH PLANNING ANALYSIS ===")
print(response.choices[0].message.content)