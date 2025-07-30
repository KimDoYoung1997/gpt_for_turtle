#!/usr/bin/env python3
from openai import OpenAI
import os
import json
import math
import re

# ROS2 관련 import 추가
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
import tf2_ros
from tf2_ros import TransformException

# 유틸리티 모듈들 import
from utils import convert_pgm_to_base64_png, read_yaml_content, read_json_content

api_key = os.getenv('OPENAI_API_KEY')
client = OpenAI(api_key=api_key)

class HighLevelPathPlanner(Node):
    """Door/Elevator 기반 High-Level Path Planning 시스템 with Nav2 Integration"""
    
    def __init__(self, pgm_path, yaml_path, json_path):
        """시스템 초기화"""
        super().__init__('high_level_path_planner')
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
        
        # Nav2 Action Client 설정
        self.nav_action_client = ActionClient(self, NavigateToPose, '/keti_amm/navigate_to_pose')
        self.current_pose = None
        
        # TF2 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 타이머 설정 (현재 위치 업데이트)
        self.create_timer(0.1, self.update_current_pose)
        
        print(f"✅ 로드 완료:")
        print(f"   📊 Resolution: {self.resolution}m/pixel")
        print(f"   📍 Origin: {self.origin}")
        print(f"   🚪 Doors: {list(self.doors.keys())}")
        print(f"   🛗 Elevators: {list(self.elevators.keys())}")
        
        # Nav2 action server 연결 확인
        self.get_logger().info('Nav2 action server를 기다리는 중...')
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Nav2 action server를 찾을 수 없습니다. 시뮬레이션 모드로 동작합니다.')
            self.nav2_available = False
        else:
            self.get_logger().info('Nav2 action server가 준비되었습니다!')
            self.nav2_available = True
    
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
        """GPT 응답에서 경로 정보 추출 (개선된 버전)"""
        print("\n🔧 경로 정보 파싱 중...")
        
        # 경로 패턴 찾기: "경로**: start → elevator2 → door2 → goal" 형태
        path_patterns = [
            r'\*\*경로\*\*[:\s]*start\s*→\s*(.*?)\s*→\s*goal',
            r'경로[:\s]*start\s*→\s*(.*?)\s*→\s*goal',
            r'start\s*→\s*(.*?)\s*→\s*goal'
        ]
        
        waypoint_sequence = []
        
        for pattern in path_patterns:
            match = re.search(pattern, gpt_response, re.IGNORECASE)
            if match:
                waypoints_str = match.group(1)
                # waypoints를 → 기준으로 분리
                waypoints = [w.strip() for w in waypoints_str.split('→')]
                
                for waypoint in waypoints:
                    # Door/Elevator 이름과 좌표 매칭
                    if waypoint in self.doors:
                        waypoint_sequence.append((waypoint, self.doors[waypoint], 'door'))
                    elif waypoint in self.elevators:
                        waypoint_sequence.append((waypoint, self.elevators[waypoint], 'elevator'))
                
                print(f"✅ 경로 파싱 성공: {[w[0] for w in waypoint_sequence]}")
                return waypoint_sequence
        
        # 패턴 매칭 실패 시 기존 키워드 기반 방식 사용
        print("⚠️ 패턴 매칭 실패, 키워드 기반 파싱 사용")
        waypoints = []
        
        for door_name in self.doors.keys():
            if door_name in gpt_response:
                waypoints.append((door_name, self.doors[door_name], 'door'))
        
        for elevator_name in self.elevators.keys():
            if elevator_name in gpt_response:
                waypoints.append((elevator_name, self.elevators[elevator_name], 'elevator'))
        
        return waypoints
    
    def execute_navigation_sequence(self, waypoint_sequence, goal_x, goal_y):
        """경로 순서대로 Nav2 내비게이션 실행"""
        if not self.nav2_available:
            print("❌ Nav2가 사용할 수 없습니다. 시뮬레이션 모드입니다.")
            return False
        
        print(f"\n🚀 순차 내비게이션 시작 ({len(waypoint_sequence)}개 경유점 + 최종 목적지)")
        
        # 각 경유점으로 순차 이동
        for i, (name, coords, obj_type) in enumerate(waypoint_sequence, 1):
            print(f"\n📍 Step {i}/{len(waypoint_sequence)}: {name} ({obj_type}) -> ({coords[0]:.2f}, {coords[1]:.2f})")
            
            success = self.nav2_navigate(coords[0], coords[1])
            if not success:
                print(f"❌ {name} 이동 실패!")
                return False
            
            print(f"✅ {name} 도착 완료!")
            
            # 잠시 대기 (안정화)
            import time
            time.sleep(1.0)
        
        # 최종 목적지로 이동
        print(f"\n🎯 최종 목적지로 이동: ({goal_x}, {goal_y})")
        success = self.nav2_navigate(goal_x, goal_y)
        
        if success:
            print("🎉 전체 경로 내비게이션 완료!")
            return True
        else:
            print("❌ 최종 목적지 이동 실패!")
            return False
    
    def nav2_navigate(self, target_x, target_y, target_yaw=0.0):
        """Nav2를 사용한 단일 지점 내비게이션"""
        if not self.nav_action_client.server_is_ready():
            self.get_logger().error('Nav2 action server가 준비되지 않았습니다.')
            return False
        
        try:
            send_goal_future = self.send_nav2_goal(target_x, target_y, target_yaw)
            success = self.wait_for_nav2_completion(send_goal_future)
            return success
        except Exception as e:
            self.get_logger().error(f'Nav2 실행 중 오류: {str(e)}')
            return False
    
    def send_nav2_goal(self, target_x, target_y, target_yaw=0.0):
        """Nav2 NavigateToPose action을 사용하여 목적지로 이동합니다."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 목적지 위치 설정
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.position.z = 0.0
        
        # 목적지 방향 설정 (yaw를 quaternion으로 변환)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(target_yaw / 2.0)
        
        goal_msg.behavior_tree = ""  # 기본 behavior tree 사용
        
        self.get_logger().info(f'Nav2 목적지 전송: x={target_x}, y={target_y}, yaw={target_yaw}')
        
        # Action 전송
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.nav2_feedback_callback
        )
        
        return send_goal_future
    
    def nav2_feedback_callback(self, feedback_msg):
        """Nav2 피드백을 처리합니다."""
        feedback = feedback_msg.feedback
        current_pos = feedback.current_pose.pose.position
        distance = feedback.distance_remaining
        
        self.get_logger().debug(
            f'Nav2 진행상황: 현재위치=({current_pos.x:.2f}, {current_pos.y:.2f}), '
            f'남은거리={distance:.2f}m'
        )
    
    def wait_for_nav2_completion(self, send_goal_future, timeout_sec=30.0):
        """Nav2 action이 완료될 때까지 대기합니다."""
        # Goal 수락 대기 (timeout 추가)
        start_time = self.get_clock().now()
        while not send_goal_future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > 5.0:  # 5초 timeout
                self.get_logger().error('Nav2 goal 전송 timeout!')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 목적지가 거부되었습니다!')
            return False
        
        self.get_logger().info('Nav2 목적지가 수락되었습니다. 이동을 시작합니다...')
        
        # 결과 대기 (timeout 추가)
        result_future = goal_handle.get_result_async()
        start_time = self.get_clock().now()
        
        while not result_future.done():
            current_time = self.get_clock().now()
            elapsed = (current_time - start_time).nanoseconds / 1e9
            
            if elapsed > timeout_sec:
                self.get_logger().warn(f'Nav2 내비게이션이 {timeout_sec}초 timeout되었습니다!')
                # Goal 취소 시도
                try:
                    goal_handle.cancel_goal_async()
                    self.get_logger().info('Nav2 goal을 취소했습니다.')
                except:
                    pass
                return False
            
            # 주기적으로 상태 체크
            if int(elapsed) % 5 == 0 and elapsed > 0:  # 5초마다 로그
                self.get_logger().debug(f'Nav2 내비게이션 진행 중... ({elapsed:.0f}초 경과)')
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Nav2 내비게이션이 성공적으로 완료되었습니다!')
            return True
        elif result.status == 2:  # CANCELED
            self.get_logger().warn('Nav2 내비게이션이 취소되었습니다.')
            return False
        elif result.status == 3:  # ABORTED
            self.get_logger().error('Nav2 내비게이션이 중단되었습니다.')
            return False
        else:
            self.get_logger().error(f'Nav2 내비게이션이 실패했습니다. 상태: {result.status}')
            return False
    
    def update_current_pose(self):
        """현재 위치 업데이트"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
            
            self.current_pose = transform
            return True
                
        except TransformException as ex:
            # 조용히 실패 (너무 많은 로그 방지)
            return False
    
    def interactive_planning(self):
        """대화형 경로 계획 인터페이스 (Nav2 통합)"""
        print(f"\n{'='*80}")
        print("🤖 HIGH-LEVEL PATH PLANNING SYSTEM with Nav2")
        print("📋 Door/Elevator 기반 최단 경로 계획 + 실제 내비게이션")
        print(f"{'='*80}")
        print("시작점과 목적지를 입력하면 GPT가 경로를 계획하고 Nav2로 실행합니다.")
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
                    for i, (name, coords, obj_type) in enumerate(waypoints, 1):
                        print(f"   {i}. {name} ({obj_type}): ({coords[0]:.2f}, {coords[1]:.2f})")
                    
                    # 실행 여부 확인
                    execute_input = input(f"\n🚀 Nav2로 실제 내비게이션을 실행하시겠습니까? (y/n): ").strip().lower()
                    if execute_input in ['y', 'yes', 'ㅇ']:
                        print(f"\n🤖 Nav2 내비게이션 시작...")
                        
                        # ROS2 spin을 위한 별도 스레드에서 실행하거나 직접 실행
                        success = self.execute_navigation_sequence(waypoints, goal_x, goal_y)
                        
                        if success:
                            print("🎉 내비게이션이 성공적으로 완료되었습니다!")
                        else:
                            print("❌ 내비게이션 중 오류가 발생했습니다.")
                    else:
                        print("📋 경로 계획만 완료했습니다. 실행하지 않습니다.")
                        
                    # 간단한 검증
                    if len(waypoints) > 3:
                        print(f"⚠️  경고: 경유점이 {len(waypoints)}개로 많습니다. 최적화가 필요할 수 있습니다.")
                else:
                    print("⚠️ 경유점을 추출할 수 없었습니다. GPT 응답을 확인하세요.")
                
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
    """메인 함수 (ROS2 통합)"""
    # ROS2 초기화
    rclpy.init()
    
    print("🚀 Door/Elevator 기반 High-Level Path Planning 시스템 with Nav2")
    
    # 맵 파일 경로 설정
    pgm_path = "/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.pgm"
    yaml_path = "/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.yaml"
    json_path = "/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/obj_poses.json"
    
    try:
        # High-Level Path Planner 초기화 (ROS2 Node)
        planner = HighLevelPathPlanner(pgm_path, yaml_path, json_path)
        
        # 대화형 경로 계획 시작
        planner.interactive_planning()
        
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        print("파일 경로와 OpenAI API 키를 확인해주세요.")
    finally:
        # ROS2 정리
        if 'planner' in locals():
            planner.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
