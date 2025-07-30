from openai import OpenAI
import os 
from utils import encode_image_to_base64, convert_pgm_to_base64_png, read_yaml_content, read_json_content


api_key = os.getenv('OPENAI_API_KEY')
client = OpenAI(api_key=api_key)

# Graph JSON 파일 내용 읽기
graph_data = read_json_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/obj_poses.json")



def analyze_image_with_gpt4o(user_question="이 이미지를 분석해주세요.",image_path=None,graph_data=None,message_history=[]):
    if len(message_history)==0:
        message_history.append(
            {"role": "system", 
             "content": 
                 """
                    당신은 모바일 로봇 High-level 경로 계획 전문가입니다.  실제 실내 맵 데이터({image_path}, {yaml_path})는 복도, 방, 엘리베이터 등의 구조로 구성되어 있고, 
                        - {image_path} 이미지 파일은 실제 맵 데이터를 표현한 이미지 파일입니다.
                        - {yaml_path} YAML 파일은 실제 맵 데이터의 정보(origin, resolution 등)를 담고 있습니다.
                        - {graph_data} Graph JSON 파일은 실제 맵 데이터의 정보(door, elevator, room 등)를 담고 있습니다.
                    이를 분석하여 모바일 로봇의 high-level 경로 계획을 생성해주세요.
                        - 
                 """
             }
        )
    if image_path is not None:
        
        base64_image = convert_pgm_to_base64_png(image_path)
        yaml_path = image_path.replace(".pgm", ".yaml")
        yaml_content = read_yaml_content(yaml_path)
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
    print("   - 이미지 분석: @이미지파일명 (예: @./map/250722.pgm)")
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
            analysis_result = analyze_image_with_gpt4o(user_question="모바일 로봇이 사용할 Occupancy Grid Map 데이터를 분석해주세요.",image_path=image_path)
        else:
            analysis_result = analyze_image_with_gpt4o(user_input)
