from openai import OpenAI
import os 
from utils import encode_image_to_base64

api_key = os.getenv('OPENAI_API_KEY')
client = OpenAI(api_key=api_key)
def analyze_image_with_gpt4o(user_question="이 이미지를 분석해주세요.",image_path=None,message_history=[]):
    if len(message_history)==0:
        message_history.append(
            {"role": "system", 
             "content": "You are a helpful assistant."
             }
        )
    if image_path is not None:
        base64_image = encode_image_to_base64(image_path)
        message_history.append(
            {"role": "user", 
             "content": [{"type": "image_url", 
                          "image_url": {"url": f"data:image/png;base64,{base64_image}"}
                          }
                         ]
             }
             )
        
    # 사용자 질문 추가
    message_history.append(
        {"role": "user", 
         "content": user_question})
    # GPT에 질문을 던져 답변을 받음
    completion=client.chat.completions.create(
        model="gpt-4.1",
        messages=message_history 
    )
    message_history.append(
        {"role": "assistant", 
         "content": completion.choices[0].message.content}
    )
    print("message_history의 length",len(message_history))
    print(completion.choices[0].message.content)
    return message_history


if __name__ == "__main__":
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
            analysis_result = analyze_image_with_gpt4o(user_question="이 이미지를 분석해주세요.",image_path=image_path)
        else:
            analysis_result = analyze_image_with_gpt4o(user_input)
