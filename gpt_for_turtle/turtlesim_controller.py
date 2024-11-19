#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from openai import OpenAI
import json
import math

class TurtleGPTController(Node):
    def __init__(self):
        super().__init__('turtle_gpt_controller')
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY 환경 변수가 설정되지 않았습니다.')
            return
            
        self.client = OpenAI(api_key=api_key)

        # OpenAI API 키 직접 설정
        
        # Publisher 설정
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        
        # Pose Subscriber 설정
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        self.current_pose = Pose()
        self.initial_pose = None
        self.get_logger().info('TurtleGPT 컨트롤러가 시작되었습니다.')
    
    def pose_callback(self, msg):
        self.current_pose = msg
        if self.initial_pose is None:
            self.initial_pose = Pose()
            self.initial_pose.x = msg.x
            self.initial_pose.y = msg.y
            self.initial_pose.theta = msg.theta
    
    def generate_movement_command(self, prompt):
        system_message = """
        당신은 ROS2 turtlesim 로봇을 제어하는 시스템입니다.
        다음과 같은 명령을 처리할 수 있습니다:
        1. 기본 이동 명령: JSON 형식으로 반환
        {
            "command_type": "basic_move",
            "linear_x": float (-2.0~2.0),
            "angular_z": float (-2.0~2.0),
            "duration": float (1.0~5.0)
        }
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
                    "type": "move",
                    "linear_x": float,
                    "angular_z": float,
                    "duration": float
                },
                {
                    "type": "position",
                    "x": float,
                    "y": float
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
            # 현재 위치와 목표 위치 간의 차이 계산
            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 0.1:  # 목표 지점에 도달
                self.stop_movement()
                break
                
            # 목표 방향 계산
            target_theta = math.atan2(dy, dx)
            angle_diff = target_theta - self.current_pose.theta
            
            # 각도 보정
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
                
            # 이동 명령 생성
            twist = Twist()
            if abs(angle_diff) > 0.1:
                twist.angular.z = 0.8 if angle_diff > 0 else -0.8
            else:
                twist.linear.x = min(0.5, distance)
            
            self.velocity_publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def stop_movement(self):
        twist = Twist()
        self.velocity_publisher.publish(twist)
    
    def execute_command(self, command_data):
        if command_data['command_type'] == 'basic_move':
            self.execute_basic_movement(command_data)
        elif command_data['command_type'] == 'move_to_position':
            self.move_to_position(command_data['target_x'], command_data['target_y'])
        elif command_data['command_type'] == 'get_pose':
            self.get_logger().info(f'현재 위치: x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}, theta={self.current_pose.theta:.2f}')
        elif command_data['command_type'] == 'sequence':
            for move in command_data['moves']:
                if move['type'] == 'move':
                    self.execute_basic_movement(move)
                elif move['type'] == 'position':
                    self.move_to_position(move['x'], move['y'])
    
    def execute_basic_movement(self, movement_data):
        twist = Twist()
        twist.linear.x = float(movement_data.get('linear_x', 0.0))
        twist.angular.z = float(movement_data.get('angular_z', 0.0))
        
        start_time = self.get_clock().now()
        duration = float(movement_data.get('duration', 1.0))
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            # 벽 충돌 감지
            if self.check_wall_collision():
                self.stop_movement()
                self.get_logger().warn('벽 충돌 감지! 이동을 중지합니다.')
                break
            
            self.velocity_publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_movement()
    
    def check_wall_collision(self):
        # 벽 경계 확인 (turtlesim은 일반적으로 11x11 크기)
        return (self.current_pose.x <= 0.5 or self.current_pose.x >= 10.5 or
                self.current_pose.y <= 0.5 or self.current_pose.y >= 10.5)
    
    def run(self):
        while rclpy.ok():
            try:
                prompt = input("\n거북이에게 명령을 내려주세요 (종료: 'quit'): ")
                if prompt.lower() == 'quit':
                    break
                
                command_data = self.generate_movement_command(prompt)
                if command_data:
                    self.get_logger().info(f'실행할 명령: {command_data}')
                    self.execute_command(command_data)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'오류 발생: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleGPTController()
    
    try:
        controller.run()
    except Exception as e:
        controller.get_logger().error(f'실행 중 오류 발생: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
