# 이미지 분석 기능이 포함된 챗봇 코드
from openai import OpenAI
import os
from utils import encode_image_to_base64
api_key = os.getenv('OPENAI_API_KEY')

client = OpenAI()

# def encode_image_to_base64(image_path):
#     """이미지 파일을 base64로 인코딩하는 함수"""
#     try:
#         with Image.open(image_path) as img:
#             # PNG로 변환하여 호환성 확보
#             if img.mode == 'RGBA':
#                 img = img.convert('RGB')
            
#             # 메모리에 PNG로 저장
#             buffer = io.BytesIO()
#             img.save(buffer, format='PNG')
            
#             # base64 인코딩
#             image_data = buffer.getvalue()
#             base64_image = base64.b64encode(image_data).decode('utf-8')
            
#             return base64_image
#     except Exception as e:
#         print(f"이미지 인코딩 오류: {e}")
#         return None

def analyze_image_with_gpt4o(image_path, user_question="이 이미지를 분석해주세요."):
    """GPT-4o를 사용하여 이미지를 분석하는 함수"""
    base64_image = encode_image_to_base64(image_path)
    
    if base64_image is None:
        return "이미지를 읽을 수 없습니다. 파일 경로를 확인해주세요."
    
    try:
        response = client.chat.completions.create(
            model="gpt-4o",  # 최신 GPT-4o 모델 사용
            messages=[
                {
                    "role": "system",
                    "content": "당신은 이미지 분석 전문가입니다. 이미지를 자세히 분석하여 한국어로 상세하게 설명해주세요."
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": user_question
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/png;base64,{base64_image}"
                            }
                        },
                        {
                            "type": "text",
                            "text": "이곳에 놀러가려하는데 관련해서 어떤 장비를 사야하는지 알려줘?"
                        },
                        {
                            "type": "text",
                            "text": "장비가 얼마야?"
                        },
                    ]
                }
            ],
            temperature=0.0,
        )
        
        return response.choices[0].message.content
        
    except Exception as e:
        return f"이미지 분석 중 오류가 발생했습니다: {e}"

messages = [
    {
        "role": "system",
        "content": "당신은 도움이 되는 AI 어시스턴트입니다. 텍스트 질문에는 일반적인 대화로 응답하고, 이미지가 포함된 질문에는 이미지를 분석하여 상세하게 설명해주세요."
    }
]

print("💬 이미지 분석 챗봇이 시작되었습니다!")
print("📝 사용법:")
print("   - 일반 질문: 그냥 텍스트로 입력하세요")
print("   - 이미지 분석: @이미지파일명 (예: @images.jpg)")
print("   - 종료: '종료', 'exit', 'quit' 입력")
print("-" * 50)

while True:
    user_input = input("사용자 입력: ")

    if user_input.lower() in ["종료", "exit", "quit"]:
        print("챗봇: 안녕히 가세요!")
        break

    # 이미지 파일 입력 체크 (@로 시작하는 경우)
    if user_input.startswith("@"):
        image_path = user_input[1:]  # @ 제거
        
        # 파일 존재 확인
        if not os.path.exists(image_path):
            print(f"챗봇: 죄송합니다. '{image_path}' 파일을 찾을 수 없습니다.")
            continue
        
        print("🔍 이미지를 분석 중입니다...")
        
        # 이미지 분석 수행
        analysis_result = analyze_image_with_gpt4o(image_path)
        print("챗봇 (이미지 분석):", analysis_result)
        
        # 대화 기록에 추가 (이미지는 텍스트로 요약하여 저장)
        messages.append({
            "role": "user",
            "content": f"이미지 파일 '{image_path}' 분석 요청"
        })
        messages.append({
            "role": "assistant", 
            "content": analysis_result
        })
        
    else:
        # 일반 텍스트 대화
        messages.append({
            "role": "user",
            "content": user_input
        })

        try:
            completion = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=messages,
                temperature=0.7
            )

            response = completion.choices[0].message.content
            print("챗봇:", response)
            
            messages.append({
                "role": "assistant",
                "content": response
            })
            
        except Exception as e:
            print(f"챗봇: 죄송합니다. 오류가 발생했습니다: {e}")
