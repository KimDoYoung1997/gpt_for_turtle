import os
from openai import OpenAI
from utils import convert_pgm_to_base64_png, read_yaml_content
import yaml
print('🖼️  GPT-4o 이미지 분석 테스트...')

yaml_content = read_yaml_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250722.yaml")

# YAML에서 resolution 값 안전하게 추출
try:
    
    yaml_data = yaml.safe_load(yaml_content)
    resolution_value = yaml_data.get('resolution', 0.05)
    origin_value = yaml_data.get('origin', [0, 0, 0])
except:
    resolution_value = 0.05
    origin_value = [0, 0, 0]


text_prompt = '''제공된 Occupancy Grid Map 이미지를 분석하여 건물 구조를 파악해주세요.

**맵 메타데이터:**
- 해상도: {resolution_value}m/픽셀
- 원점: {origin_value}
- 이미지: 흰색=자유공간, 검은색=벽

**분석 요청:**
1. 전체 건물 레이아웃(방, 엘리베이터, 복도) 파악
2. 방,엘리베이터, 복도 구분 및 중심좌표 계산 (월드 좌표계)
3. 각 영역의 크기와 연결성 분석
4. 위의 출력 형식에 맞춰 상세 분석 결과 제공

월드 좌표계를 고려하여 정확한 분석을 제공해주세요.
'''

try:
    client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
    base64_image = convert_pgm_to_base64_png('map/test.pgm')
    
    print(f'📊 Base64 데이터 준비 완료 ({len(base64_image)} 문자)')
    
    # 간단한 이미지 분석 테스트
    response = client.chat.completions.create(
        model='gpt-4.1',
        messages=[
            {
                'role': 'system',
                'content': '''
                You are a helpful navigation assistant.
                '''
            },
            {
                'role': 'user',
                'content': [
                    {
                        'type': 'text',
                        'text': text_prompt
                    },
                    {
                        'type': 'image_url',
                        'image_url': {
                            'url': f'data:image/png;base64,{base64_image}'
                        }
                    }
                ]
            }
        ],
        max_tokens=200
    )
    
    print(f'✅ 이미지 분석 성공!')
    print(f'응답: {response.choices[0].message.content}')
    
except Exception as e:
    print(f'❌ 이미지 분석 실패: {e}')
    import traceback
    traceback.print_exc()
