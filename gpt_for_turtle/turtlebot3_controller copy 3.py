#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros
from tf2_ros import TransformException
import openai
import json
import math

class TurtleBot3GPTController(Node):
    def __init__(self):
        super().__init__('turtlebot3_gpt_controller')
        self.current_pose = None
        self.landmarks = {}  # 랜드마크 정보 저장
        
        # 환경 변수에서 API 키 읽어오기, OPENAI_API_KEY 키에 저장되어 있는 값을 읽어들인다.
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY 환경 변수가 설정되지 않았습니다.')
            return
            
        openai.api_key = api_key
        
        # 랜드마크 데이터 로드
        self.load_landmarks()
        
        # Publisher 설정
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Nav2 Action Client 설정
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF2 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 타이머 설정 (10Hz로 tf 업데이트)
        self.create_timer(0.1, self.update_current_pose)
        
        self.get_logger().info('TurtleBot3 GPT 컨트롤러가 시작되었습니다.')
        self.get_logger().info('Nav2 action server를 기다리는 중...')
        
        # Nav2 action server가 준비될 때까지 대기
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Nav2 action server를 찾을 수 없습니다. nav2 명령은 사용할 수 없습니다.')
        else:
            self.get_logger().info('Nav2 action server가 준비되었습니다!')
    
    def update_current_pose(self):
        wait_count = 0
        self.update_bool = False
        while wait_count < 50:  # 최대 5초 대기
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time())
                
                self.current_pose = transform
                self.get_logger().debug(f'현재 위치 업데이트: x={transform.transform.translation.x:.2f}, y={transform.transform.translation.y:.2f}')
                self.update_bool = True
                return True
                
            except TransformException as ex:
                self.get_logger().debug(f'TF 업데이트를 기다리는 중... ({wait_count}/50)')
                rclpy.spin_once(self, timeout_sec=0.1)
                wait_count += 1
        
        # self.get_logger().error('TF를 가져올 수 없습니다.')
        return False
    
    def generate_movement_command(self, prompt):
        system_message = """
        당신은 ROS2 TurtleBot3 로봇을 제어하는 시스템입니다.
        다음과 같은 명령을 처리할 수 있습니다:
        
        1. 기본 이동 명령: JSON 형식으로 반환
        {
            "command_type": "basic_move",
            "type": "linear" 또는 "angular",
            "distance": float(미터 또는 라디안),
            "direction": "forward"/"backward" 또는 "left"/"right"
        }
        
        각도 회전 시:
        - 90도는 1.5708 라디안 
        - 45도는 0.7854 라디안
        - 180도는 3.1416 라디안
        
        모든 각도는 라디안으로 변환하여 distance에 입력해야 합니다.

        2. Nav2 내비게이션 명령 (권장): JSON 형식으로 반환 - 경로 계획과 장애물 회피 포함
        {
            "command_type": "nav2_navigate",
            "target_x": float,
            "target_y": float,
            "target_yaw": float (선택사항, 라디안 단위, 기본값 0.0)
        }
        
        3. 랜드마크 기반 내비게이션 명령 (권장): JSON 형식으로 반환 - 미리 정의된 위치로 이동
        {
            "command_type": "landmark_navigate",
            "landmark_name": string (예: "door_0", "room_1" 등)
        }
        
        4. 직접 위치 이동 명령 (백업용): JSON 형식으로 반환 - Nav2 없을 때 사용
        {
            "command_type": "move_to_position",
            "target_x": float,
            "target_y": float
        }
        
        5. 위치 확인 명령:
        {
            "command_type": "get_pose"
        }
        
        6. 랜드마크 목록 확인 명령:
        {
            "command_type": "list_landmarks"
        }
        
        7. 복합 이동 명령:
        {
            "command_type": "sequence",
            "moves": [
                {
                    "command_type": "basic_move",
                    "type": "linear" 또는 "angular",
                    "distance": float,
                    "direction": "forward"/"backward" 또는 "left"/"right"
                },
                {
                    "command_type": "landmark_navigate",
                    "landmark_name": string
                },
                {
                    "command_type": "nav2_navigate",
                    "target_x": float,
                    "target_y": float,
                    "target_yaw": float
                }
            ]
        }
        
        명령 선택 우선순위:
        1. "door_0으로 가", "room_1로 이동" 등 특정 랜드마크 이름이 있으면 "landmark_navigate" 사용
        2. 위치 이동 요청시 기본적으로 "nav2_navigate" 사용 (더 안전하고 정확)
        3. "직접", "단순히", "빠르게" 등의 키워드가 있으면 "move_to_position" 사용
        4. "랜드마크 목록", "어떤 장소" 등이 있으면 "list_landmarks" 사용
        5. 복잡한 환경이나 장애물 회피가 필요하면 반드시 "nav2_navigate" 사용      
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": prompt}
                ]
            )
            return json.loads(response.choices[0].message.content)
        except Exception as e:
            self.get_logger().error(f'GPT 오류: {str(e)}')
            return None

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

    def move_to_position(self, target_x, target_y):
        while rclpy.ok():
            if not self.wait_for_pose_update():
                continue
            
            current_pos = self.get_current_position()
            if not current_pos:
                self.get_logger().info('위치 업데이트를 기다리는 중...')
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            
            target_pos = Point(x=float(target_x), y=float(target_y))
            distance, target_theta = self.calculate_movement_values('linear', current_pos, target_pos=target_pos)
            
            if distance < 0.1:
                self.stop_movement()
                break
            
            current_theta = self.get_current_orientation()
            if current_theta is None:
                continue
            
            angle_diff = target_theta - current_theta
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle_diff) > 0.1:
                self.move_robot(angular_z=0.5 if angle_diff > 0 else -0.5)
            else:
                self.move_robot(linear_x=min(0.2, distance))
            
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def stop_movement(self):
        twist = Twist()
        self.velocity_publisher.publish(twist)
    
    def execute_command(self, command_data):
        if command_data['command_type'] == 'basic_move':
            self.execute_basic_movement(command_data)
        elif command_data['command_type'] == 'move_to_position':
            self.move_to_position(command_data['target_x'], command_data['target_y'])
        elif command_data['command_type'] == 'nav2_navigate':
            target_x = command_data['target_x']
            target_y = command_data['target_y']
            target_yaw = command_data.get('target_yaw', 0.0)  # 기본값 0.0
            
            # Nav2 action server가 사용 가능한지 확인
            if not self.nav_action_client.server_is_ready():
                self.get_logger().warn('Nav2 action server가 준비되지 않았습니다. 직접 이동으로 대체합니다.')
                self.move_to_position(target_x, target_y)
                return
            
            # Nav2 goal 전송
            try:
                send_goal_future = self.send_nav2_goal(target_x, target_y, target_yaw)
                success = self.wait_for_nav2_completion(send_goal_future)
                
                # Nav2가 실패한 경우 직접 이동으로 폴백
                if not success:
                    self.get_logger().warn('Nav2 내비게이션이 실패했습니다. 직접 이동으로 재시도합니다.')
                    self.move_to_position(target_x, target_y)
                    
            except Exception as e:
                self.get_logger().error(f'Nav2 실행 중 오류: {str(e)}. 직접 이동으로 대체합니다.')
                self.move_to_position(target_x, target_y)
        
        elif command_data['command_type'] == 'landmark_navigate':
            landmark_name = command_data['landmark_name']
            position = self.get_landmark_position(landmark_name)
            
            if position is not None:
                landmark_x, landmark_y, landmark_yaw = position
                self.get_logger().info(f'랜드마크 "{landmark_name}"으로 이동: x={landmark_x:.2f}, y={landmark_y:.2f}, yaw={landmark_yaw:.2f}')
                
                # Nav2 우선 시도
                if self.nav_action_client.server_is_ready():
                    try:
                        send_goal_future = self.send_nav2_goal(landmark_x, landmark_y, landmark_yaw)
                        success = self.wait_for_nav2_completion(send_goal_future)
                        
                        if not success:
                            self.get_logger().warn('Nav2 내비게이션이 실패했습니다. 직접 이동으로 재시도합니다.')
                            self.move_to_position(landmark_x, landmark_y)
                    except Exception as e:
                        self.get_logger().error(f'Nav2 실행 중 오류: {str(e)}. 직접 이동으로 대체합니다.')
                        self.move_to_position(landmark_x, landmark_y)
                else:
                    self.get_logger().warn('Nav2가 준비되지 않았습니다. 직접 이동으로 대체합니다.')
                    self.move_to_position(landmark_x, landmark_y)
            else:
                self.get_logger().error(f'랜드마크 "{landmark_name}"을(를) 찾을 수 없습니다.')
            
        elif command_data['command_type'] == 'get_pose':
            if self.update_current_pose():  # TF 업데이트 시도
                self.get_logger().info(
                    f'현재 위치: x={self.current_pose.transform.translation.x:.2f}, '
                    f'y={self.current_pose.transform.translation.y:.2f}, '
                    f'theta={self.get_yaw_from_quaternion(self.current_pose.transform.rotation):.2f}'
                )
                return  # 성공적으로 위치를 출력한 경우 여기서 종료
            self.get_logger().error('현재 위치를 가져올 수 없습니다.')
        elif command_data['command_type'] == 'list_landmarks':
            available_landmarks = self.list_available_landmarks()
            if available_landmarks:
                landmarks_info = []
                for landmark_name in available_landmarks:
                    landmark = self.landmarks[landmark_name]
                    landmarks_info.append(f"{landmark_name} ({landmark['category']}): x={landmark['x']:.2f}, y={landmark['y']:.2f}")
                self.get_logger().info(f'사용 가능한 랜드마크 ({len(available_landmarks)}개):\n' + '\n'.join(landmarks_info))
            else:
                self.get_logger().info('등록된 랜드마크가 없습니다.')
            return
        elif command_data['command_type'] == 'sequence':
            for move in command_data['moves']:
                if move['command_type'] == 'basic_move':
                    self.execute_basic_movement(move)
                    self.stop_movement()
                elif move['command_type'] == 'move_to_position':
                    self.move_to_position(move['target_x'], move['target_y'])
                    self.stop_movement()
                elif move['command_type'] == 'landmark_navigate':
                    landmark_name = move['landmark_name']
                    position = self.get_landmark_position(landmark_name)
                    
                    if position is not None:
                        landmark_x, landmark_y, landmark_yaw = position
                        # Nav2 action server가 사용 가능한지 확인
                        if self.nav_action_client.server_is_ready():
                            try:
                                send_goal_future = self.send_nav2_goal(landmark_x, landmark_y, landmark_yaw)
                                success = self.wait_for_nav2_completion(send_goal_future)
                                if not success:
                                    self.get_logger().warn('Nav2 실패, 직접 이동으로 대체합니다.')
                                    self.move_to_position(landmark_x, landmark_y)
                            except Exception as e:
                                self.get_logger().error(f'Nav2 오류: {str(e)}, 직접 이동으로 대체합니다.')
                                self.move_to_position(landmark_x, landmark_y)
                        else:
                            self.get_logger().warn('Nav2 서버가 준비되지 않아 직접 이동으로 대체합니다.')
                            self.move_to_position(landmark_x, landmark_y)
                    else:
                        self.get_logger().error(f'랜드마크 "{landmark_name}"을(를) 찾을 수 없어 해당 이동을 건너뜁니다.')
                elif move['command_type'] == 'nav2_navigate':
                    target_x = move['target_x']
                    target_y = move['target_y']
                    target_yaw = move.get('target_yaw', 0.0)
                    
                    if self.nav_action_client.server_is_ready():
                        try:
                            send_goal_future = self.send_nav2_goal(target_x, target_y, target_yaw)
                            success = self.wait_for_nav2_completion(send_goal_future)
                            if not success:
                                self.get_logger().warn('Nav2 실패, 직접 이동으로 대체합니다.')
                                self.move_to_position(target_x, target_y)
                        except Exception as e:
                            self.get_logger().error(f'Nav2 오류: {str(e)}, 직접 이동으로 대체합니다.')
                            self.move_to_position(target_x, target_y)
                    else:
                        self.get_logger().warn('Nav2 서버가 준비되지 않아 직접 이동으로 대체합니다.')
                        self.move_to_position(target_x, target_y)
                rclpy.spin_once(self, timeout_sec=0.1)
    
    def execute_basic_movement(self, movement_data):
        if not self.wait_for_pose_update():
            return
        
        # sequence 명령의 경우
        if 'linear_x' in movement_data and 'angular_z' in movement_data:
            linear_x = float(movement_data.get('linear_x', 0.0))
            angular_z = float(movement_data.get('angular_z', 0.0))
            duration = float(movement_data.get('duration', 1.0))  # 기본값 1초
            
            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
                self.move_robot(linear_x, angular_z)
                rclpy.spin_once(self, timeout_sec=0.1)
        
        # 기존 basic_move 명령의 경우
        else:
            movement_type = movement_data.get('type', 'linear')
            target_distance = abs(float(movement_data.get('distance', 0.0)))
            direction = movement_data.get('direction', 'forward')
            
            linear_x = 0.2 if direction == 'forward' else -0.2
            angular_z = 0.5 if direction == 'left' else -0.5
            
            start_pos = self.get_current_position()
            start_theta = self.get_current_orientation()
            
            while rclpy.ok():
                if not self.wait_for_pose_update():
                    break
                
                current_progress = self.calculate_movement_values(
                    movement_type, 
                    self.get_current_position(), 
                    start_pos=start_pos, 
                    start_theta=start_theta
                )
                
                if current_progress >= target_distance:
                    break
                
                self.move_robot(
                    linear_x if movement_type == 'linear' else 0.0,
                    angular_z if movement_type == 'angular' else 0.0
                )
                rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_movement()
    
    def run(self):
        while rclpy.ok():
            try:
                prompt = input("\nTurtleBot3에게 명령을 내려주세요 (종료: 'quit'): ")
                if prompt.lower() == 'quit':
                    break
                
                command_data = self.generate_movement_command(prompt)   #  prompt : 'x 0, y 1 만큼 이동해'  # command_data : {'command_type': 'move_to_position', 'target_x': 0.0, 'target_y': 1.0}
                if command_data:
                    self.get_logger().info(f'실행할 명령: {command_data}')
                    self.execute_command(command_data)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'오류 발생: {str(e)}')

    def calculate_distance(self, point1, point2):
        """두 점 사이의 거리를 계산합니다."""
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        return math.sqrt(dx**2 + dy**2)

    def get_current_position(self):
        """현재 위치를 Point 형태로 반환합니다."""
        if not self.current_pose or not hasattr(self.current_pose, 'transform'):
            return None
        point = Point()
        point.x = self.current_pose.transform.translation.x
        point.y = self.current_pose.transform.translation.y
        point.z = 0.0
        return point

    def get_current_orientation(self):
        """현재 방향(theta)을 반환합니다."""
        if not self.current_pose or not hasattr(self.current_pose, 'transform'):
            return None
        return self.get_yaw_from_quaternion(self.current_pose.transform.rotation)

    def wait_for_pose_update(self):
        """TF 업데이트를 기다립니다."""
        if not self.update_current_pose():
            self.get_logger().error('현재 위치를 가져올 수 없습니다.')
            return False
        return True

    def move_robot(self, linear_x=0.0, angular_z=0.0):
        """로봇 이동을 위한 공통 함수"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.velocity_publisher.publish(twist)

    def calculate_movement_values(self, movement_type, current_pos, target_pos=None, start_pos=None, start_theta=None):
        """이동 관련 계산을 처리하는 통합 함수"""
        if movement_type == 'linear':
            if target_pos:  # move_to_position용
                dx = target_pos.x - current_pos.x
                dy = target_pos.y - current_pos.y
                distance = math.sqrt(dx**2 + dy**2)
                target_theta = math.atan2(dy, dx)
                return distance, target_theta
            else:  # basic_movement용
                return self.calculate_distance(start_pos, current_pos)
        elif movement_type == 'angular':  # angular movement
            current_theta = self.get_current_orientation()
            if current_theta is None or start_theta is None:
                return 0.0
            angle_diff = abs(current_theta - start_theta)
            return 2 * math.pi - angle_diff if angle_diff > math.pi else angle_diff
        else:
            return self.get_logger().info('잘못된 명령입니다. 코드 위치를 확인하세요')

    def load_landmarks(self):
        """graph.json에서 랜드마크 데이터를 로드합니다."""
        try:
            # 현재 패키지 디렉토리에서 graph.json 찾기
            import rclpy
            from ament_index_python.packages import get_package_share_directory
            
            # 여러 가능한 경로 시도
            possible_paths = [
                'src/gpt_for_turtle/gpt_for_turtle/graph.json',  # 현재 ws 기준
                '/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/graph.json',  # 절대 경로
                'graph.json'  # 현재 디렉토리
            ]
            
            graph_data = None
            for path in possible_paths:
                try:
                    with open(path, 'r') as f:
                        graph_data = json.load(f)
                        self.get_logger().info(f'graph.json을 로드했습니다: {path}')
                        break
                except FileNotFoundError:
                    continue
            
            if graph_data is None:
                self.get_logger().warn('graph.json 파일을 찾을 수 없습니다. 랜드마크 기능이 비활성화됩니다.')
                return
            
            # nodes 배열에서 랜드마크 정보 추출
            if 'nodes' in graph_data:
                for node in graph_data['nodes']:
                    if 'id' in node and 'pose' in node:
                        landmark_id = node['id']
                        pose = node['pose']  # [x, y, z]
                        
                        # yaw 계산 (orientation에서)
                        yaw = 0.0
                        if 'orientation' in node and len(node['orientation']) >= 4:
                            # quaternion [x, y, z, w]에서 yaw 계산
                            qx, qy, qz, qw = node['orientation']
                            yaw = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))
                        
                        self.landmarks[landmark_id] = {
                            'x': pose[0],
                            'y': pose[1],
                            'z': pose[2] if len(pose) > 2 else 0.0,
                            'yaw': yaw,
                            'category': node.get('category', 'unknown')
                        }
                        
                self.get_logger().info(f'랜드마크 데이터를 로드했습니다. 총 {len(self.landmarks)}개: {list(self.landmarks.keys())}')
            else:
                self.get_logger().warn('graph.json에 nodes 정보가 없습니다.')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'graph.json 파싱 오류: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'랜드마크 로드 중 오류: {str(e)}')

    def get_landmark_position(self, landmark_name):
        """랜드마크 이름으로 해당 랜드마크의 위치를 반환합니다."""
        if landmark_name in self.landmarks:
            landmark = self.landmarks[landmark_name]
            return landmark['x'], landmark['y'], landmark.get('yaw', 0.0)
        else:
            # 사용 가능한 랜드마크 목록 표시
            available = list(self.landmarks.keys())
            self.get_logger().error(f'랜드마크 "{landmark_name}"을(를) 찾을 수 없습니다. 사용 가능한 랜드마크: {available}')
            return None

    def list_available_landmarks(self):
        """사용 가능한 랜드마크 목록을 반환합니다."""
        return list(self.landmarks.keys())


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBot3GPTController()
    
    try:
        controller.run()
    except Exception as e:
        controller.get_logger().error(f'실행 중 오류 발생: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

