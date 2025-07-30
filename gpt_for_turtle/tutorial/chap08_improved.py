from openai import OpenAI
import os 
from utils import encode_image_to_base64

api_key = os.getenv('OPENAI_API_KEY')
if not api_key:
    raise ValueError("OPENAI_API_KEY 환경변수를 설정해주세요!")

client = OpenAI(api_key=api_key)

def analyze_image_with_gpt4o(user_question="이 이미지를 분석해주세요.", 
                           image_path=None, 
                           message_history=None):
    """
    GPT-4o를 사용해 이미지 분석 및 대화
    
    Args:
        user_question: 사용자 질문
        image_path: 이미지 파일 경로 (선택사항)
        message_history: 대화 히스토리 (multi-turn을 위해 반드시 전달해야 함!)
        
    Returns:
        업데이트된 message_history
        
    주의: Multi-turn 대화를 원한다면 반환된 message_history를 
         다음 호출 시 다시 전달해야 합니다!
    """
    
    # 안전한 기본값 처리
    if message_history is None:
        message_history = []
        print("💡 새로운 대화를 시작합니다.")
    
    if len(message_history) == 0:
        message_history.append({
            "role": "system", 
            "content": "You are a helpful assistant."
        })
    
    # 이미지와 텍스트가 모두 있는 경우 하나의 메시지로 처리
    if image_path is not None:
        # 파일 존재 확인 추가
        if not os.path.exists(image_path):
            print(f"❌ 파일을 찾을 수 없습니다: {image_path}")
            return message_history
            
        base64_image = encode_image_to_base64(image_path)
        if base64_image is None:
            print("❌ 이미지 인코딩에 실패했습니다.")
            return message_history
        
        # 하나의 메시지에 이미지와 텍스트 모두 포함
        user_content = [
            {
                "type": "image_url", 
                "image_url": {"url": f"data:image/png;base64,{base64_image}"}
            },
            {
                "type": "text",
                "text": user_question
            }
        ]
        
        message_history.append({
            "role": "user", 
            "content": user_content
        })
    else:
        # 텍스트만 있는 경우
        message_history.append({
            "role": "user", 
            "content": user_question
        })
    
    try:
        # GPT에 질문을 던져 답변을 받음
        completion = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=message_history 
        )
        
        response_content = completion.choices[0].message.content
        message_history.append({
            "role": "assistant", 
            "content": response_content
        })
        
        print("💬 메시지 히스토리 길이:", len(message_history))
        print("🤖 챗봇:", response_content)
        return message_history
        
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        return message_history


# Multi-turn 사용 예시 함수
def simple_chat_example():
    """간단한 multi-turn 대화 예시"""
    print("\n🎯 Multi-turn 대화 예시:")
    
    # 히스토리 초기화
    history = None
    
    # 첫 번째 질문
    history = analyze_image_with_gpt4o("안녕하세요! 저는 김철수입니다.", message_history=history)
    
    # 두 번째 질문 (이름을 기억하는지 확인)
    history = analyze_image_with_gpt4o("제 이름이 뭐라고 했죠?", message_history=history)
    
    return history


if __name__ == "__main__":
    print("💬 이미지 분석 챗봇이 시작되었습니다!")
    print("📝 사용법:")
    print("   - 일반 질문: 그냥 텍스트로 입력하세요")
    print("   - 이미지 분석: @이미지파일명 (예: @images.jpg)")
    print("   - 이미지 + 텍스트: @이미지파일명 / 텍스트 (예: @images.jpg / 이 동물을 좋아해)")
    print("   - 예시 보기: 'test' 입력")
    print("   - 종료: '종료', 'exit', 'quit' 입력")
    print("-" * 50)

    message_history = []  # 여기서 초기화!

    while True:
        user_input = input("\n💬 입력: ")

        if user_input.lower() in ["종료", "exit", "quit"]:
            print("👋 챗봇: 안녕히 가세요!")
            break
        
        # 테스트 예시 실행
        if user_input.lower() == "test":
            simple_chat_example()
            continue

        # 이미지 파일 입력 체크 (@로 시작하는 경우)
        if user_input.startswith("@"):
            # '/' 기준으로 이미지와 텍스트 분리
            if " / " in user_input:
                # @ 제거 후 / 기준으로 분리
                parts = user_input[1:].split(" / ", 1)  # 최대 2개 부분으로 분리
                image_path = parts[0].strip()
                text_prompt = parts[1].strip()
                print(f"🔍 이미지({image_path})와 텍스트를 함께 분석 중입니다...")
                message_history = analyze_image_with_gpt4o(
                    user_question=text_prompt,
                    image_path=image_path,
                    message_history=message_history
                )
            else:
                # 기존 방식: 이미지만
                image_path = user_input[1:]  # @ 제거
                print("🔍 이미지를 분석 중입니다...")
                message_history = analyze_image_with_gpt4o(
                    user_question="이 이미지를 분석해주세요.",
                    image_path=image_path,
                    message_history=message_history
                )
        else:
            message_history = analyze_image_with_gpt4o(
                user_input, 
                message_history=message_history
            ) 