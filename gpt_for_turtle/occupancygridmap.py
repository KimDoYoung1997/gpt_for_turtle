import numpy as np
import yaml
import math
from PIL import Image

class OccupancyGridMap:
    def __init__(self, pgm_path, yaml_path):
        self.load_map_data(pgm_path, yaml_path)
    
    def load_map_data(self, pgm_path, yaml_path):
        """PGM과 YAML 파일에서 맵 데이터 로드"""
        # YAML 파일에서 메타데이터 읽기
        with open(yaml_path, 'r') as f:
            self.map_metadata = yaml.safe_load(f)
        
        self.resolution = self.map_metadata['resolution']
        self.origin = self.map_metadata['origin']
        self.occupied_thresh = self.map_metadata.get('occupied_thresh', 0.65)
        self.free_thresh = self.map_metadata.get('free_thresh', 0.25)
        self.negate = self.map_metadata.get('negate', 0)
        
        # PGM 파일에서 grid 데이터 읽기
        image = Image.open(pgm_path)
        self.grid_data = np.array(image)
        
        # grid.py 방식: Y축을 뒤집어서 올바른 월드 좌표계로 변환
        self.grid_data = np.flipud(self.grid_data)
        
        self.height, self.width = self.grid_data.shape
    
    def world_to_grid(self, world_x, world_y):
        """월드 좌표를 그리드 좌표로 변환 (grid.py 방식 적용)"""
        # 월드 좌표에서 맵 원점을 기준으로 한 상대 좌표로 변환
        relative_x = world_x - self.origin[0]
        relative_y = world_y - self.origin[1]
        
        # 픽셀 좌표로 변환
        grid_x = int(relative_x / self.resolution)
        grid_y = int(relative_y / self.resolution)
        
        # grid.py 방식: 이미 flipud()로 Y축을 뒤집었으므로 추가 변환 불필요
        # 단지 범위 체크만 수행
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """그리드 좌표를 월드 좌표로 변환 (grid.py 방식 적용)"""
        # 픽셀 좌표를 월드 좌표로 변환
        world_x = self.origin[0] + grid_x * self.resolution
        world_y = self.origin[1] + grid_y * self.resolution
        
        return world_x, world_y
    
    def is_valid_cell(self, grid_x, grid_y):
        """유효한 그리드 셀인지 확인"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def is_free_cell(self, grid_x, grid_y):
        """자유 공간인지 확인"""
        if not self.is_valid_cell(grid_x, grid_y):
            return False
        
        pixel_value = self.grid_data[grid_y, grid_x]
        
        # negate 처리
        if self.negate:
            pixel_value = 255 - pixel_value
        
        # 0은 점유, 255는 자유공간, 중간값은 미지
        normalized_value = pixel_value / 255.0
        
        return normalized_value > self.free_thresh
    
    def is_occupied_cell(self, grid_x, grid_y):
        """점유 공간인지 확인"""
        if not self.is_valid_cell(grid_x, grid_y):
            return True  # 맵 밖은 점유로 간주
        
        pixel_value = self.grid_data[grid_y, grid_x]
        
        # negate 처리
        if self.negate:
            pixel_value = 255 - pixel_value
        
        normalized_value = pixel_value / 255.0
        
        return normalized_value < (1.0 - self.occupied_thresh)
    
    def get_neighbors(self, grid_x, grid_y):
        """8방향 이웃 셀 반환"""
        neighbors = []
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        for dx, dy in directions:
            nx, ny = grid_x + dx, grid_y + dy
            if self.is_valid_cell(nx, ny) and self.is_free_cell(nx, ny):
                # 대각선 이동의 경우 cost가 더 큼
                cost = math.sqrt(2) if abs(dx) + abs(dy) == 2 else 1.0
                neighbors.append((nx, ny, cost))
        
        return neighbors 