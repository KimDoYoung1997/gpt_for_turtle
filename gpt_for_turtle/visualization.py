import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def visualize_path(grid_map, waypoints, connections, full_path, start_pos, goal_pos, waypoint_path=None):
    """A* 경로 계획 결과를 시각화"""
    plt.figure(figsize=(12, 10))
    
    # 1. Occupancy Grid Map 표시
    extent = [
        grid_map.origin[0], 
        grid_map.origin[0] + grid_map.width * grid_map.resolution,
        grid_map.origin[1],
        grid_map.origin[1] + grid_map.height * grid_map.resolution
    ]
    
    plt.imshow(grid_map.grid_data, cmap='gray', 
               extent=extent, origin='lower', alpha=0.8)
    
    # 2. Waypoint들 표시
    waypoint_xs = [wp['x'] for wp in waypoints.values()]
    waypoint_ys = [wp['y'] for wp in waypoints.values()]
    plt.scatter(waypoint_xs, waypoint_ys, c='blue', s=100, marker='s', 
                alpha=0.7, label='Waypoints', zorder=5)
    
    # Waypoint 라벨 표시
    for node_id, wp in waypoints.items():
        plt.annotate(f"{node_id}", (wp['x'], wp['y']), 
                    xytext=(5, 5), textcoords='offset points',
                    fontsize=8, color='blue', weight='bold')
    
    # 3. Waypoint 간 연결선 표시 (점선)
    for node_id, connected_nodes in connections.items():
        if node_id not in waypoints:
            continue
        for connected_id in connected_nodes:
            if connected_id not in waypoints:
                continue
            start_wp = waypoints[node_id]
            end_wp = waypoints[connected_id]
            plt.plot([start_wp['x'], end_wp['x']], 
                    [start_wp['y'], end_wp['y']], 
                    'b--', alpha=0.3, linewidth=1, zorder=2)
    
    # 4. A* 경로 표시
    if full_path and len(full_path) > 1:
        path_xs = [pos[0] for pos in full_path]
        path_ys = [pos[1] for pos in full_path]
        
        # Waypoint 구간별로 다른 색상 사용
        if waypoint_path and len(waypoint_path) > 1:
            colors = ['red', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan']
            current_idx = 0
            
            for i, waypoint_id in enumerate(waypoint_path):
                target_wp = waypoints[waypoint_id]
                # 현재 waypoint까지의 경로 찾기
                next_idx = current_idx
                for j in range(current_idx, len(full_path)):
                    if abs(full_path[j][0] - target_wp['x']) < 0.1 and abs(full_path[j][1] - target_wp['y']) < 0.1:
                        next_idx = j + 1
                        break
                
                if next_idx > current_idx:
                    segment_xs = path_xs[current_idx:next_idx]
                    segment_ys = path_ys[current_idx:next_idx]
                    color = colors[i % len(colors)]
                    plt.plot(segment_xs, segment_ys, color=color, linewidth=2.5, 
                            alpha=0.8, label=f'Segment {i+1}', zorder=4)
                    current_idx = next_idx - 1
            
            # 마지막 구간 (목표점까지)
            if current_idx < len(full_path) - 1:
                segment_xs = path_xs[current_idx:]
                segment_ys = path_ys[current_idx:]
                color = colors[len(waypoint_path) % len(colors)]
                plt.plot(segment_xs, segment_ys, color=color, linewidth=2.5, 
                        alpha=0.8, label=f'Segment {len(waypoint_path)+1}', zorder=4)
        else:
            plt.plot(path_xs, path_ys, 'red', linewidth=2.5, alpha=0.8, 
                    label='A* Path', zorder=4)
    
    # 5. 시작점과 목표점 표시
    plt.scatter(start_pos[0], start_pos[1], c='lime', s=200, marker='o', 
                edgecolors='black', linewidth=2, label='Start', zorder=6)
    plt.scatter(goal_pos[0], goal_pos[1], c='red', s=200, marker='*', 
                edgecolors='black', linewidth=2, label='Goal', zorder=6)
    
    # 6. 범례와 제목
    plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
    plt.title('A* Path Planning Visualization\n(Gray: Occupied, White: Free Space)', fontsize=14, pad=20)
    plt.xlabel('X (m)', fontsize=12)
    plt.ylabel('Y (m)', fontsize=12)
    plt.grid(True, alpha=0.3)
    
    # 축 비율 맞추기
    plt.axis('equal')
    plt.tight_layout()
    
    # 통계 정보 텍스트 박스
    if full_path:
        total_distance = sum(math.sqrt((full_path[i+1][0] - full_path[i][0])**2 + 
                                      (full_path[i+1][1] - full_path[i][1])**2) 
                            for i in range(len(full_path)-1))
        stats_text = f'Path Length: {total_distance:.1f}m\nPath Points: {len(full_path)}'
        if waypoint_path:
            stats_text += f'\nWaypoints: {len(waypoint_path)}'
        
        plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
                fontsize=10, verticalalignment='top', 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.show()

def visualize_waypoints_only(grid_map, waypoints, connections):
    """Waypoint 네트워크만 시각화"""
    plt.figure(figsize=(12, 10))
    
    # 1. Occupancy Grid Map 표시
    extent = [
        grid_map.origin[0], 
        grid_map.origin[0] + grid_map.width * grid_map.resolution,
        grid_map.origin[1],
        grid_map.origin[1] + grid_map.height * grid_map.resolution
    ]
    
    plt.imshow(grid_map.grid_data, cmap='gray', 
               extent=extent, origin='lower', alpha=0.8)
    
    # 2. Waypoint들 표시
    waypoint_xs = [wp['x'] for wp in waypoints.values()]
    waypoint_ys = [wp['y'] for wp in waypoints.values()]
    plt.scatter(waypoint_xs, waypoint_ys, c='blue', s=100, marker='s', 
                alpha=0.7, label='Waypoints', zorder=5)
    
    # Waypoint 라벨 표시
    for node_id, wp in waypoints.items():
        plt.annotate(f"{node_id}: {wp['name']}", (wp['x'], wp['y']), 
                    xytext=(5, 5), textcoords='offset points',
                    fontsize=8, color='blue', weight='bold')
    
    # 3. Waypoint 간 연결선 표시
    for node_id, connected_nodes in connections.items():
        if node_id not in waypoints:
            continue
        for connected_id in connected_nodes:
            if connected_id not in waypoints:
                continue
            start_wp = waypoints[node_id]
            end_wp = waypoints[connected_id]
            plt.plot([start_wp['x'], end_wp['x']], 
                    [start_wp['y'], end_wp['y']], 
                    'b-', alpha=0.6, linewidth=2, zorder=3)
    
    plt.legend(loc='upper right')
    plt.title('Waypoint Network Visualization', fontsize=14, pad=20)
    plt.xlabel('X (m)', fontsize=12)
    plt.ylabel('Y (m)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.show() 