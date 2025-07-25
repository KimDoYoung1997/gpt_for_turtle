import os
from openai import OpenAI
from utils import convert_pgm_to_base64_png

print('🖼️  GPT-4o 이미지 분석 테스트...')

try:
    client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
    base64_image = convert_pgm_to_base64_png('map/test.pgm')
    
    print(f'📊 Base64 데이터 준비 완료 ({len(base64_image)} 문자)')
    
    # 간단한 이미지 분석 테스트
    response = client.chat.completions.create(
        model='gpt-4.1',
        messages=[
            {
                'role': 'user',
                'content': [
                    {
                        'type': 'text',
                        'text': '이 이미지에 대해 설명해봐.'
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
