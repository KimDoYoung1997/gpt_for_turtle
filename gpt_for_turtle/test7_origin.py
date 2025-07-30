import os
import json
import math

# 유틸리티 모듈들 import
from utils import convert_pgm_to_base64_png, read_yaml_content, read_json_content

api_key = os.getenv('OPENAI_API_KEY')
client = OpenAI(api_key=api_key)

class HighLevelPathPlanner:
    """Door/Elevator 기반 High-Level Path Planning 시스템"""
    
    def __init__(self, pgm_path, yaml_path, json_path):
        """시스템 초기화"""
        print("🔍 High-Level Path Planner 초기화 중...")
        
        # 맵 데이터 로드
        self.base64_image = convert_pgm_to_base64_png(pgm_path)
        self.yaml_content = read_yaml_content(yaml_path)
        self.obj_poses = read_json_content(json_path)
        
        # YAML에서 resolution과 origin 추출
        try:
            import yaml
            yaml_data = yaml.safe_load(self.yaml_content)
            self.resolution = yaml_data.get('resolution', 0.05)
            self.origin = yaml_data.get('origin', [0, 0, 0])
        except:
            self.resolution = 0.05
            self.origin = [0, 0, 0]
        
        # Door/Elevator 정보 정리
        self.doors = {door['class_name']: door['centroid'][:2] for door in self.obj_poses['door']}
        self.elevators = {elevator['class_name']: elevator['centroid'][:2] for elevator in self.obj_poses['elevator']}
        
        print(f"✅ 로드 완료:")
        print(f"   📊 Resolution: {self.resolution}m/pixel")
        print(f"   📍 Origin: {self.origin}")
        print(f"   🚪 Doors: {list(self.doors.keys())}")
        print(f"   🛗 Elevators: {list(self.elevators.keys())}")
    
    def plan_high_level_path(self, start_x, start_y, goal_x, goal_y):
        """GPT를 활용한 High-Level Path Planning"""
        
        print(f"\n🚀 High-Level Path Planning 시작")
        print(f"   📍 시작점: ({start_x}, {start_y})")
        print(f"   🎯 목적지: ({goal_x}, {goal_y})")
        
        # GPT에게 경로 계획 요청
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "system",
                    "content": """당신은 **Mobile Robot High-Level Path Planning** 전문가입니다.

**역할**: Occupancy Grid Map과 Door/Elevator 위치를 분석하여 시작점에서 목적지까지의 **최단 경로**를 계획

**🚨 중요한 원칙:**
1. **최소 경유점**: 절대적으로 필요한 Door/Elevator만 선택
2. **불필요한 우회 금지**: 굳이 거치지 않아도 되는 Door/Elevator는 제외
3. **직선 경로 우선**: 장애물이 없다면 직선 경로 선택
4. **Include only the necessary doors that are part of the path being used**

**올바른 예시:**
- 시작점 (-3,0) → 목적지 (8,4)
- 올바른 경로: start → elevator2 → door2 → goal (최소 경유점)
- 잘못된 경로: start → elevator2 → door1 → door2 → goal (불필요한 door1)

**분석 과정:**
1. 시작점에서 가장 가까운 Door/Elevator 찾기
2. 목적지에서 가장 가까운 Door/Elevator 찾기  
3. 둘 사이의 연결성 확인
4. **절대 필요한 것만** 경로에 포함

**출력 형식:**
## 🔍 **경로 분석 결과**

**시작점 분석**: (x, y) - 가장 가까운 Door/Elevator 식별
**목적지 분석**: (x, y) - 가장 가까운 Door/Elevator 식별
**연결성 분석**: 두 점 사이의 최단 연결 방법

## ✅ **최종 경로 계획**

**경로**: start → [최소 필수 경유점] → goal
**이유**: 각 선택이 꼭 필요한 이유만 설명

**❌ 절대 금지사항:**
- 불필요한 Door/Elevator 추가
- 의미없는 우회 경로
- 3개 이상의 경유점 (특별한 경우 제외)"""
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": f"""다음 조건에서 **최단 경로**를 계획해주세요.

**🗺️ 맵 정보:**
- 해상도: {self.resolution}m/픽셀
- 원점: {self.origin}
- 이미지: 흰색=자유공간, 검은색=벽/장애물

**📍 경로 계획 요청:**
- 시작점: ({start_x}, {start_y})
- 목적지: ({goal_x}, {goal_y})

**🚪🛗 사용 가능한 Door/Elevator:**
```json
{json.dumps(self.obj_poses, indent=2)}
```

**⚡ 핵심 요구사항:**
1. **최소 경유점**: 꼭 필요한 Door/Elevator만 선택
2. **거리 최적화**: 시작점에서 가장 가까운 것, 목적지에서 가장 가까운 것 우선
3. **불필요한 우회 절대 금지**: 3개 이상의 경유점은 특별한 이유가 있을 때만
4. **Include only the necessary doors/elevators that are part of the path**

**예시 (참고용):**
- (-3,0) → (8,4): start → elevator2 → door2 → goal ✅
- (-3,0) → (8,4): start → elevator2 → door1 → door2 → goal ❌ (door1 불필요)

위 정보를 바탕으로 **최단 경로**를 계획해주세요."""
                        },
                        {
                            "type": "image_url", 
                            "image_url": {
                                "url": f"data:image/png;base64,{self.base64_image}"
                            }
                        }
                    ]
                }
            ],
            temperature=0.0  # 더 일관된 결과를 위해 0.0으로 설정
        )
        
        return response.choices[0].message.content
    
    def parse_planned_path(self, gpt_response):
        """GPT 응답에서 경로 정보 추출"""
        print("\n🔧 경로 정보 파싱 중...")
        
        # 간단한 키워드 기반 파싱
        waypoints = []
        
        # Door/Elevator 이름들을 응답에서 찾기
        for door_name in self.doors.keys():
            if door_name in gpt_response:
                waypoints.append((door_name, self.doors[door_name]))
        
        for elevator_name in self.elevators.keys():
            if elevator_name in gpt_response:
                waypoints.append((elevator_name, self.elevators[elevator_name]))
        
        return waypoints
    
    def interactive_planning(self):
        """대화형 경로 계획 인터페이스"""
        print(f"\n{'='*80}")
        print("🤖 HIGH-LEVEL PATH PLANNING SYSTEM")
        print("📋 Door/Elevator 기반 최단 경로 계획")
        print(f"{'='*80}")
        print("시작점과 목적지를 입력하면 GPT가 최단 Door/Elevator 경로를 계획합니다.")
        print("Type 'quit' to exit.")
        
        plan_count = 0
        
        while True:
            try:
                print(f"\n{'-'*50}")
                print(f"Path Planning #{plan_count + 1}")
                print(f"{'-'*50}")
                
                # 시작점 입력
                start_input = input("🤖 시작점 입력 (x y): ").strip()
                if start_input.lower() == 'quit':
                    break
                    
                start_coords = list(map(float, start_input.split()))
                if len(start_coords) != 2:
                    print("❌ 정확히 두 개의 숫자를 입력하세요 (x y)")
                    continue
                start_x, start_y = start_coords
                
                # 목적지 입력  
                goal_input = input("🎯 목적지 입력 (x y): ").strip()
                if goal_input.lower() == 'quit':
                    break
                    
                goal_coords = list(map(float, goal_input.split()))
                if len(goal_coords) != 2:
                    print("❌ 정확히 두 개의 숫자를 입력하세요 (x y)")
                    continue
                goal_x, goal_y = goal_coords
                
                # GPT 기반 경로 계획
                print(f"\n🧠 GPT가 최단 경로를 분석 중...")
                gpt_response = self.plan_high_level_path(start_x, start_y, goal_x, goal_y)
                
                print(f"\n📋 GPT 경로 계획 결과:")
                print("="*60)
                print(gpt_response)
                print("="*60)
                
                # 경로 파싱
                waypoints = self.parse_planned_path(gpt_response)
                if waypoints:
                    print(f"\n📍 추출된 경유점 ({len(waypoints)}개):")
                    for i, (name, coords) in enumerate(waypoints, 1):
                        print(f"   {i}. {name}: ({coords[0]:.2f}, {coords[1]:.2f})")
                        
                    # 간단한 검증
                    if len(waypoints) > 3:
                        print(f"⚠️  경고: 경유점이 {len(waypoints)}개로 많습니다. 최적화가 필요할 수 있습니다.")
                
                plan_count += 1
                
                # 계속할지 물어보기
                print(f"\n{'-'*40}")
                continue_input = input("다른 경로를 계획하시겠습니까? (y/n): ").strip().lower()
                if continue_input not in ['y', 'yes', 'ㅇ']:
                    break
                    
            except ValueError:
                print("❌ 잘못된 입력 형식입니다. 숫자를 공백으로 구분해서 입력하세요.")
            except KeyboardInterrupt:
                print("\n\n👋 프로그램을 종료합니다.")
                break
        
        print(f"\n🎉 총 {plan_count}개의 경로를 계획했습니다.")

def main():
    """메인 함수"""
    print("🚀 Door/Elevator 기반 High-Level Path Planning 시스템")
    
    # 맵 파일 경로 설정
    pgm_path = "/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.pgm"
    yaml_path = "/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.yaml"
    json_path = "/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/obj_poses.json"
    
    try:
        # High-Level Path Planner 초기화
        planner = HighLevelPathPlanner(pgm_path, yaml_path, json_path)
        
        # 대화형 경로 계획 시작
        planner.interactive_planning()
        
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        print("파일 경로와 OpenAI API 키를 확인해주세요.")

if __name__ == "__main__":
    main()
