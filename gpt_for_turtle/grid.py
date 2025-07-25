#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import yaml
import os
from PIL import Image


class MapVisualizer:
    def __init__(self, yaml_file_path, pgm_file_path):
        """
        맵 시각화 클래스 초기화
        
        Args:
            yaml_file_path: YAML 메타데이터 파일 경로
            pgm_file_path: PGM 이미지 파일 경로
        """
        self.yaml_file_path = yaml_file_path
        self.pgm_file_path = pgm_file_path
        self.map_data = None
        self.map_metadata = None
        
    def load_map_metadata(self):
        """YAML 파일에서 맵 메타데이터를 읽어옵니다."""
        try:
            with open(self.yaml_file_path, 'r') as file:
                self.map_metadata = yaml.safe_load(file)
            print("맵 메타데이터 로드 완료:")
            for key, value in self.map_metadata.items():
                print(f"  {key}: {value}")
        except Exception as e:
            print(f"YAML 파일 로드 실패: {e}")
            return False
        return True
    
    def load_map_image(self):
        """PGM 파일에서 맵 이미지를 읽어옵니다."""
        try:
            # PIL을 사용해서 PGM 파일 읽기
            with Image.open(self.pgm_file_path) as img:
                self.map_data = np.array(img)
            
            # 이미지를 y축으로 뒤집어서 올바른 월드 좌표계로 변환
            self.map_data = np.flipud(self.map_data)
            
            print(f"맵 이미지 로드 완료:")
            print(f"  크기: {self.map_data.shape}")
            print(f"  데이터 타입: {self.map_data.dtype}")
            print(f"  값 범위: {self.map_data.min()} ~ {self.map_data.max()}")
            
        except Exception as e:
            print(f"PGM 파일 로드 실패: {e}")
            return False
        return True
    
    def convert_to_occupancy_grid(self):
        """
        PGM 이미지 데이터를 점유 격자로 변환합니다.
        
        Returns:
            numpy array: 점유 확률 (0: 자유공간, 1: 점유공간, 0.5: 미지영역)
        """
        if self.map_data is None or self.map_metadata is None:
            return None
        
        # PGM 값을 점유 확률로 변환
        occupied_thresh = self.map_metadata.get('occupied_thresh', 0.65)
        free_thresh = self.map_metadata.get('free_thresh', 0.25)
        
        # 255 스케일을 확률로 변환
        probability = (255 - self.map_data) / 255.0
        
        # 점유 격자 생성
        occupancy_grid = np.full_like(probability, 0.5)  # 기본값: 미지영역
        
        # 자유공간
        occupancy_grid[probability < free_thresh] = 0.0
        
        # 점유공간
        occupancy_grid[probability > occupied_thresh] = 1.0
        
        return occupancy_grid
    
    def get_world_coordinates(self):
        """맵 좌표를 실제 월드 좌표로 변환하는 정보를 반환합니다."""
        if self.map_metadata is None:
            return None
        
        resolution = self.map_metadata.get('resolution', 0.05)
        origin = self.map_metadata.get('origin', [0, 0, 0])
        
        height, width = self.map_data.shape
        
        # 월드 좌표 범위 계산 (y축 뒤집기 고려)
        x_min = origin[0]
        y_min = origin[1]
        x_max = x_min + width * resolution
        y_max = y_min + height * resolution
        
        return {
            'x_range': (x_min, x_max),
            'y_range': (y_min, y_max),
            'resolution': resolution,
            'origin': origin
        }
    
    def visualize_map(self, show_grid=True, show_coordinates=True):
        """맵을 시각화합니다."""
        if not self.load_map_metadata() or not self.load_map_image():
            return
        
        # 점유 격자 변환
        occupancy_grid = self.convert_to_occupancy_grid()
        world_coords = self.get_world_coordinates()
        
        # 플롯 설정
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
        
        # 원본 PGM 이미지 표시 (origin='lower'로 올바른 방향)
        ax1.imshow(self.map_data, cmap='gray', origin='lower')
        ax1.set_title('원본 PGM 이미지 (수정된 방향)')
        ax1.set_xlabel('픽셀 X')
        ax1.set_ylabel('픽셀 Y')
        
        if show_grid:
            ax1.grid(True, alpha=0.3)
        
        # 점유 격자 맵 표시
        if world_coords and show_coordinates:
            # 실제 월드 좌표로 표시
            extent = [
                world_coords['x_range'][0], world_coords['x_range'][1],
                world_coords['y_range'][0], world_coords['y_range'][1]
            ]
            ax2.imshow(occupancy_grid, cmap='RdYlBu_r', origin='lower', extent=extent, vmin=0, vmax=1)
            ax2.set_xlabel('X (미터)')
            ax2.set_ylabel('Y (미터)')
            ax2.set_title(f'점유 격자 맵 (해상도: {world_coords["resolution"]}m/픽셀)')
        else:
            ax2.imshow(occupancy_grid, cmap='RdYlBu_r', origin='lower', vmin=0, vmax=1)
            ax2.set_xlabel('픽셀 X')
            ax2.set_ylabel('픽셀 Y')
            ax2.set_title('점유 격자 맵')
        
        if show_grid:
            ax2.grid(True, alpha=0.3)
        
        # 컬러바 추가
        cbar = plt.colorbar(ax2.images[0], ax=ax2, fraction=0.046, pad=0.04)
        cbar.set_label('점유 확률 (0: 자유, 1: 점유, 0.5: 미지)')
        
        # 범례 정보 표시
        info_text = f"""맵 정보:
크기: {self.map_data.shape[1]} x {self.map_data.shape[0]} 픽셀
해상도: {self.map_metadata.get('resolution', 'N/A')} m/픽셀
원점: {self.map_metadata.get('origin', 'N/A')}
점유 임계값: {self.map_metadata.get('occupied_thresh', 'N/A')}
자유 임계값: {self.map_metadata.get('free_thresh', 'N/A')}"""
        
        plt.figtext(0.02, 0.02, info_text, fontsize=9, 
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8))
        
        plt.tight_layout()
        plt.show()
    
    def save_visualization(self, output_path="map_visualization.png"):
        """시각화 결과를 파일로 저장합니다."""
        if not self.load_map_metadata() or not self.load_map_image():
            return
        
        occupancy_grid = self.convert_to_occupancy_grid()
        world_coords = self.get_world_coordinates()
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
        
        ax1.imshow(self.map_data, cmap='gray', origin='lower')
        ax1.set_title('원본 PGM 이미지 (수정된 방향)')
        ax1.set_xlabel('픽셀 X')
        ax1.set_ylabel('픽셀 Y')
        ax1.grid(True, alpha=0.3)
        
        if world_coords:
            extent = [
                world_coords['x_range'][0], world_coords['x_range'][1],
                world_coords['y_range'][0], world_coords['y_range'][1]
            ]
            ax2.imshow(occupancy_grid, cmap='RdYlBu_r', origin='lower', extent=extent, vmin=0, vmax=1)
            ax2.set_xlabel('X (미터)')
            ax2.set_ylabel('Y (미터)')
            ax2.set_title(f'점유 격자 맵 (해상도: {world_coords["resolution"]}m/픽셀)')
        else:
            ax2.imshow(occupancy_grid, cmap='RdYlBu_r', origin='lower', vmin=0, vmax=1)
            ax2.set_xlabel('픽셀 X')
            ax2.set_ylabel('픽셀 Y')
            ax2.set_title('점유 격자 맵')
        
        ax2.grid(True, alpha=0.3)
        
        cbar = plt.colorbar(ax2.images[0], ax=ax2, fraction=0.046, pad=0.04)
        cbar.set_label('점유 확률 (0: 자유, 1: 점유, 0.5: 미지)')
        
        plt.tight_layout()
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"시각화 결과가 {output_path}에 저장되었습니다.")

    def visualize_comparison(self):
        """원본과 수정된 방향을 비교해서 보여줍니다."""
        if not self.load_map_metadata():
            return
            
        # 원본 이미지 로드 (뒤집지 않은 상태)
        try:
            with Image.open(self.pgm_file_path) as img:
                original_data = np.array(img)
        except Exception as e:
            print(f"원본 이미지 로드 실패: {e}")
            return
        
        # 뒤집힌 이미지 (현재 self.map_data)
        flipped_data = np.flipud(original_data)
        
        # 비교 시각화
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
        
        # 원본 (뒤집지 않은 상태)
        ax1.imshow(original_data, cmap='gray', origin='upper')
        ax1.set_title('원본 PGM (origin=upper)')
        ax1.set_xlabel('픽셀 X')
        ax1.set_ylabel('픽셀 Y')
        ax1.grid(True, alpha=0.3)
        
        ax2.imshow(original_data, cmap='gray', origin='lower')
        ax2.set_title('원본 PGM (origin=lower)')
        ax2.set_xlabel('픽셀 X')
        ax2.set_ylabel('픽셀 Y')
        ax2.grid(True, alpha=0.3)
        
        # 뒤집힌 상태
        ax3.imshow(flipped_data, cmap='gray', origin='upper')
        ax3.set_title('Y축 뒤집힌 PGM (origin=upper)')
        ax3.set_xlabel('픽셀 X')
        ax3.set_ylabel('픽셀 Y')
        ax3.grid(True, alpha=0.3)
        
        ax4.imshow(flipped_data, cmap='gray', origin='lower')
        ax4.set_title('Y축 뒤집힌 PGM (origin=lower) - 수정된 버전')
        ax4.set_xlabel('픽셀 X')
        ax4.set_ylabel('픽셀 Y')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()


def main():
    """메인 함수 - 맵 시각화 실행"""
    # 파일 경로 설정
    current_dir = os.path.dirname(os.path.abspath(__file__))
    yaml_file = os.path.join(current_dir, "map", "250722.yaml")
    pgm_file = os.path.join(current_dir, "..", "..", "..", "build", "gpt_for_turtle", "gpt_for_turtle", "map", "250722.pgm")
    
    # 파일 존재 확인
    if not os.path.exists(yaml_file):
        print(f"YAML 파일을 찾을 수 없습니다: {yaml_file}")
        return
    
    if not os.path.exists(pgm_file):
        print(f"PGM 파일을 찾을 수 없습니다: {pgm_file}")
        return
    
    print(f"YAML 파일: {yaml_file}")
    print(f"PGM 파일: {pgm_file}")
    
    # 맵 시각화
    visualizer = MapVisualizer(yaml_file, pgm_file)
    
    # 방향 비교 시각화 (선택적)
    print("\n방향 비교를 보시겠습니까? (y/n):")
    # response = input().lower()
    # if response == 'y' or response == 'yes':
    #     visualizer.visualize_comparison()
    
    print("\n수정된 방향으로 맵을 시각화하고 있습니다...")
    visualizer.visualize_map(show_grid=True, show_coordinates=True)
    
    # 시각화 결과 저장
    output_file = os.path.join(current_dir, "map_visualization_corrected.png")
    visualizer.save_visualization(output_file)


if __name__ == "__main__":
    main()
