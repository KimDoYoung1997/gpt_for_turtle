#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, TransformStamped
import tf2_ros
from tf2_ros import TransformException
from openai import OpenAI
import json
import math

class TurtleBot3GPTController(Node):
    def __init__(self):
        super().__init__('turtlebot3_gpt_controller')
        self.current_pose = None
        # 환경 변수에서 API 키 읽어오기, OPENAI_API_KEY 키에 저장되어 있는 값을 읽어들인다.
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY 환경 변수가 설정되지 않았습니다.')
            return
            
        self.client = OpenAI(api_key=api_key)
        
        # Publisher 설정
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # TF2 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 타이머 설정 (10Hz로 tf 업데이트)
        self.create_timer(0.1, self.update_current_pose)
        
        self.get_logger().info('TurtleBot3 GPT 컨트롤러가 시작되었습니다.')
    
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

        2. 위치 이동 명령: JSON 형식으로 반환
        {
            "command_type": "move_to_position",
            "target_x": float,
            "target_y": float
        }
        3. 위치 확인 명령:
        {
            "command_type": "get_pose"
        }
        4. 복합 이동 명령:
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
                    "command_type": "move_to_position",
                    "target_x": float,
                    "target_y": float
                }
            ]
        }        
        """
        
        try:
            response = self.client.chat.completions.create(
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
        elif command_data['command_type'] == 'get_pose':
            if self.update_current_pose():  # TF 업데이트 시도
                self.get_logger().info(
                    f'현재 위치: x={self.current_pose.transform.translation.x:.2f}, '
                    f'y={self.current_pose.transform.translation.y:.2f}, '
                    f'theta={self.get_yaw_from_quaternion(self.current_pose.transform.rotation):.2f}'
                )
                return  # 성공적으로 위치를 출력한 경우 여기서 종료
            self.get_logger().error('현재 위치를 가져올 수 없습니다.')
        elif command_data['command_type'] == 'sequence':
            for move in command_data['moves']:
                if move['command_type'] == 'basic_move':
                    self.execute_basic_movement(move)
                    self.stop_movement()
                elif move['command_type'] == 'move_to_position':
                    self.move_to_position(move['target_x'], move['target_y'])
                    self.stop_movement()
                rclpy.spin_once(self, timeout_sec=0.1)
    
    def execute_basic_movement(self, movement_data):
        if not self.wait_for_pose_update():
            return
        
        # sequence 명령의 경우
        if 'linear_x' in movement_data and 'angular_z' in movement_data:
            linear_x = float(movement_data.get('linear_x', 0.0))
            angular_z = float(movement_data.get('angular_z', 0.0))
            
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

