import re
import math

def parse_waypoints_from_gpt_response(gpt_response):
    """GPT 응답에서 waypoint 정보를 파싱하는 함수"""
    waypoints = {}
    connections = {}
    
    lines = gpt_response.split('\n')
    parsing_table = False
    
    for line in lines:
        # 테이블 시작 찾기
        if '번호' in line and '이름' in line and '좌표' in line:
            parsing_table = True
            continue
        
        # 테이블 데이터 파싱
        if parsing_table and '|' in line:
            parts = [p.strip() for p in line.split('|')]
            if len(parts) >= 6 and parts[1].isdigit():
                try:
                    node_id = int(parts[1])
                    name = parts[2]
                    
                    # 좌표 파싱 (x, y) 형태
                    coord_match = re.search(r'\(([^,]+),\s*([^)]+)\)', parts[3])
                    if coord_match:
                        x = float(coord_match.group(1))
                        y = float(coord_match.group(2))
                        
                        waypoints[node_id] = {
                            'name': name,
                            'x': x, 
                            'y': y,
                            'description': parts[4]
                        }
                        
                        # 연결 노드 파싱
                        connection_str = parts[5].replace(' ', '')
                        if connection_str and connection_str != '-':
                            connected_nodes = [int(n) for n in connection_str.split(',') if n.isdigit()]
                            connections[node_id] = connected_nodes
                
                except (ValueError, IndexError):
                    continue
    
    return waypoints, connections

def determine_region(x, y, waypoints):
    """주어진 좌표가 어떤 영역에 속하는지 판단하는 함수"""
    min_distance = float('inf')
    closest_waypoint = None
    
    for node_id, waypoint in waypoints.items():
        distance = math.sqrt((x - waypoint['x'])**2 + (y - waypoint['y'])**2)
        if distance < min_distance:
            min_distance = distance
            closest_waypoint = node_id
    
    return closest_waypoint

def print_waypoint_table(gpt_response):
    """GPT 응답에서 waypoint 테이블 부분만 추출해서 출력"""
    lines = gpt_response.split('\n')
    printing = False
    
    for line in lines:
        # 테이블 시작 찾기
        if '최종 Waypoint' in line or ('번호' in line and '이름' in line and '좌표' in line):
            printing = True
            print(line)
            continue
        
        # 테이블 끝 찾기 (빈 줄이나 다른 섹션 시작)
        if printing and (line.strip() == '' or line.startswith('**')):
            if '추가 정보' in line or '총 waypoint' in line:
                print(line)
                continue
            else:
                break
        
        # 테이블 내용 출력
        if printing:
            print(line)

def print_parsed_waypoints_table(waypoints, connections):
    """파싱된 waypoint 정보를 깔끔한 표 형태로 출력"""
    print("\n📍 PARSED WAYPOINT DETAILS")
    print("=" * 110)
    
    # 테이블 헤더
    print(f"{'번호':<4} | {'이름':<25} | {'좌표(x, y)':<15} | {'역할/설명':<35} | {'연결 노드':<15}")
    print("-" * 110)
    
    # waypoint를 번호 순으로 정렬
    sorted_waypoints = sorted(waypoints.items())
    
    for node_id, waypoint in sorted_waypoints:
        name = waypoint['name']
        coordinates = f"({waypoint['x']:.1f}, {waypoint['y']:.1f})"
        description = waypoint.get('description', '')
        
        # 연결 노드 정보 가져오기
        connected_nodes = connections.get(node_id, [])
        connections_str = ', '.join(map(str, connected_nodes)) if connected_nodes else '-'
        
        # 문자열 길이 제한
        if len(name) > 24:
            name = name[:21] + "..."
        if len(description) > 34:
            description = description[:31] + "..."
        
        print(f"{node_id:<4} | {name:<25} | {coordinates:<15} | {description:<35} | {connections_str:<15}")
    
    print("=" * 110)
    print(f"📊 총 waypoint 개수: {len(waypoints)}개")
    print(f"🔗 연결 정보가 있는 노드: {len(connections)}개") 