from openai import OpenAI
import os
import base64
from PIL import Image
import io

client = OpenAI()

def encode_image_to_base64(image_path):
    """이미지 파일을 base64로 인코딩하는 함수"""
    try:
        with Image.open(image_path) as img:
            if img.mode == 'RGBA':
                img = img.convert('RGB')
            
            buffer = io.BytesIO()
            img.save(buffer, format='PNG')
            
            image_data = buffer.getvalue()
            base64_image = base64.b64encode(image_data).decode('utf-8')
            
            return base64_image
    except Exception as e:
        print(f"이미지 인코딩 오류: {e}")
        return None

def test_method1_single_message_multiple_content():
    """방법 1: 하나의 메시지에 여러 content (멀티모달)"""
    print("🔵 방법 1: 하나의 메시지에 여러 content")
    print("-" * 50)
    
    # 가상의 이미지 데이터 (실제로는 star.jpg를 사용)
    base64_image = "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg=="
    
    response = client.chat.completions.create(
        model="gpt-4o-mini",  # 테스트용으로 mini 사용
        messages=[
            {
                "role": "system",
                "content": "당신은 AI 어시스턴트입니다."
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "첫 번째 질문: 안녕하세요!"
                    },
                    {
                        "type": "text", 
                        "text": "두 번째 질문: 오늘 날씨가 어때요?"
                    },
                    {
                        "type": "text",
                        "text": "세 번째 질문: 파이썬을 배우고 싶어요."
                    }
                ]
            }
        ],
        temperature=0.1
    )
    
    print("GPT 응답:")
    print(response.choices[0].message.content)
    return response.choices[0].message.content

def test_method2_multiple_messages():
    """방법 2: 여러 개의 메시지 (대화 히스토리)"""
    print("\n🟠 방법 2: 여러 개의 메시지 (대화 히스토리)")
    print("-" * 50)
    
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {
                "role": "system",
                "content": "당신은 AI 어시스턴트입니다."
            },
            {
                "role": "user",
                "content": "첫 번째 질문: 안녕하세요!"
            },
            {
                "role": "assistant", 
                "content": "안녕하세요! 반갑습니다."
            },
            {
                "role": "user",
                "content": "두 번째 질문: 오늘 날씨가 어때요?"
            },
            {
                "role": "assistant",
                "content": "죄송하지만 실시간 날씨 정보는 확인할 수 없습니다."
            },
            {
                "role": "user", 
                "content": "세 번째 질문: 파이썬을 배우고 싶어요."
            }
        ],
        temperature=0.1
    )
    
    print("GPT 응답:")
    print(response.choices[0].message.content)
    return response.choices[0].message.content

def test_method3_combined_in_one_text():
    """방법 3: 하나의 텍스트에 모든 질문 합치기"""
    print("\n🟢 방법 3: 하나의 텍스트에 모든 질문 합치기")
    print("-" * 50)
    
    combined_text = """다음 질문들에 각각 답변해주세요:

1. 첫 번째 질문: 안녕하세요!
2. 두 번째 질문: 오늘 날씨가 어때요?
3. 세 번째 질문: 파이썬을 배우고 싶어요.

각 질문에 대해 개별적으로 답변해주세요."""
    
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {
                "role": "system", 
                "content": "당신은 AI 어시스턴트입니다. 여러 질문이 있을 때 각각에 대해 명확하게 답변해주세요."
            },
            {
                "role": "user",
                "content": combined_text
            }
        ],
        temperature=0.1
    )
    
    print("GPT 응답:")
    print(response.choices[0].message.content)
    return response.choices[0].message.content

if __name__ == "__main__":
    print("🧪 OpenAI API 메시지 구조 비교 테스트")
    print("=" * 60)
    
    try:
        result1 = test_method1_single_message_multiple_content()
        result2 = test_method2_multiple_messages()
        result3 = test_method3_combined_in_one_text()
        
        print("\n" + "=" * 60)
        print("📊 결론 분석:")
        print("=" * 60)
        print("방법 1 (멀티 content): 모든 질문을 동시에 고려하되, 선택적 답변 가능")
        print("방법 2 (대화 히스토리): 이전 대화를 기억하며 마지막 질문에만 답변")  
        print("방법 3 (텍스트 결합): 가장 확실하게 모든 질문에 답변")
        
    except Exception as e:
        print(f"❌ 테스트 중 오류: {e}") 