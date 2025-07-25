from openai import OpenAI
import base64
from PIL import Image
import io

client = OpenAI()

# PGM 파일을 PNG로 변환 후 base64로 인코딩
def convert_pgm_to_base64_png(pgm_path):
    # PGM 파일을 PIL Image로 열기
    image = Image.open(pgm_path)
    
    # PNG 형식으로 변환하여 메모리 버퍼에 저장
    buffer = io.BytesIO()
    image.save(buffer, format='PNG')
    buffer.seek(0)
    
    # base64로 인코딩
    return base64.b64encode(buffer.read()).decode('utf-8')

# YAML 파일 내용 읽기
def read_yaml_content(yaml_path):
    with open(yaml_path, "r") as yaml_file:
        return yaml_file.read()

# occupancy grid map 이미지를 PNG로 변환하여 인코딩
base64_image = convert_pgm_to_base64_png("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250404.pgm")

# YAML 파일 내용 읽기
yaml_content = read_yaml_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/map/250404.yaml")

response = client.chat.completions.create(
    model="gpt-4.1",
    messages=[
        {
            "role": "user", 
            "content": f"""이것은 cartographer로 생성된 occupancy grid map입니다. 
            
다음은 이 맵의 YAML 설정 파일 내용입니다:
{yaml_content}

이 occupancy grid map을 분석하고 다음 내용을 설명해주세요:
1. 맵의 전체적인 구조와 환경
2. 장애물과 자유 공간의 분포
3. 맵의 해상도와 크기 정보
4. 로봇이 탐색한 영역의 특징
5. 이 환경에서의 네비게이션 가능성"""
        },
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/png;base64,{base64_image}"
                    }
                }
            ]
        }
    ]
)

print(response.choices[0].message.content)