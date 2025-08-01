from openai import OpenAI
import os 
from utils import encode_image_to_base64, convert_pgm_to_base64_png, read_yaml_content, read_json_content


api_key = os.getenv('OPENAI_API_KEY')
client = OpenAI(api_key=api_key)

# Graph JSON 파일 내용 읽기
graph_data = read_json_content("/home/keti/turtlesim_gpt_ws/src/gpt_for_turtle/gpt_for_turtle/obj_poses.json")



def analyze_image_with_gpt4o(user_question="이 이미지를 분석해주세요.",image_path=None,graph_data=None,message_history=None):
    if message_history is None:
        message_history = []
    
    if len(message_history)==0:
        message_history.append(
            {"role": "system", 
             "content": "You are a helpful navigation assistant for mobile robots in indoor environments."
             }
        )
        
        # 예시 대화 추가
        message_history.append(
            {"role": "user", 
             "content": """당신은 모바일 로봇 High-level 경로 계획 전문가입니다. 실제 실내 맵 데이터는 복도, 방, 엘리베이터 등의 구조로 구성되어 있고, 
                        - 이미지 파일은 실제 맵 데이터를 표현한 Occupancy Grid Map입니다.
                        - YAML 파일은 실제 맵 데이터의 메타정보(origin, resolution 등)를 담고 있습니다.
                        - Graph JSON 파일은 실제 맵에서 추출된 객체 정보(door, elevator, room 등)와 그 좌표를 담고 있습니다.
                    
                    경로 계획 시 다음 원칙을 따르세요:
                    1. **노드 분석 우선**: 먼저 맵의 주요 노드들(방 중심, 복도 중심, 문, 엘리베이터 등)과 그들 간의 연결 관계를 분석하세요.
                    2. **필요한 문만 경유**: Include only the necessary doors that are part of the path being used, and do not mention doors that won't be traversed even if they are in the path.
                    3. **직접 연결 우선**: 중앙 복도와 직접 연결된 공간은 문을 거치지 않고 바로 접근 가능합니다.
                    4. **최단 경로**: 가장 효율적이고 직접적인 경로를 제안하세요.
                    
                    좌표 질문에 대해서는 반드시 pixel 좌표를 실제 미터(m) 단위로 변환해서 답변하세요.
                    변환 공식: world_x = origin_x + (pixel_x * resolution), world_y = origin_y + (pixel_y * resolution)
                    
                    맵 파일들(@250722.pgm, @250722.yaml, @obj_poses.json)을 분석해서 방 1의 중심 좌표를 실제 미터 단위로 알려주세요."""
             }
        )
        
        message_history.append(
            {"role": "assistant", 
             "content": """네, 맵 파일들을 종합적으로 분석해서 정확한 좌표를 제공해드리겠습니다.

## 📋 노드 연결 관계 분석

**주요 노드들:**
- **중앙 복도 (Central Hub)**: (0.0, 0.0) - 모든 공간의 연결 중심
- **방1 (상좌 공간)**: (-1.72, -2.37) - 중앙 복도와 직접 연결
- **엘리베이터1 공간**: (4.763, 2.449) - 중앙 복도와 직접 연결
- **엘리베이터2 공간**: (-1.625, -0.355) - 중앙 복도와 직접 연결

**연결 관계:**
```
중앙 복도 ↔ 방1 (직접 연결)
중앙 복도 ↔ 엘리베이터1 공간 (직접 연결)  
중앙 복도 ↔ 엘리베이터2 공간 (직접 연결)
```

## 맵 정보 분석

**YAML 파일 정보:**
- resolution: 0.05 (1픽셀 = 0.05미터)  
- origin: [-5.97, -5.12, 0] (맵 원점 좌표)

**픽셀 → 실제 좌표 변환:**
- 변환 공식: world_x = origin_x + (pixel_x × resolution)
- 변환 공식: world_y = origin_y + (pixel_y × resolution)

**방 1의 좌표 계산:**
- 픽셀 좌표 추정: (85, 55)
- 실제 좌표 계산:
  - world_x = -5.97 + (85 × 0.05) = -5.97 + 4.25 = **-1.72m**
  - world_y = -5.12 + (55 × 0.05) = -5.12 + 2.75 = **-2.37m**

따라서 **방 1의 중심 좌표는 (-1.72m, -2.37m)**입니다.

## ✅ **최종 Waypoint 리스트**

| 번호 | 이름                     | 좌표(x,y)            | 역할/설명                         | 연결 가능 노드 |
|-----|-------------------------|---------------------|-----------------------------------|---------------|
| 1   | 중앙 복도 중심            | (0.0, 0.0)           | 중앙 허브                          | 2,3,4         |
| 2   | 방1 중심                 | (-1.72, -2.37)       | 상좌 방 중심                       | 1             |
| 3   | 엘리베이터1              | (4.763, 2.449)       | 상단 엘리베이터                     | 1             |
| 4   | 엘리베이터2              | (-1.625, -0.355)     | 하단 엘리베이터                     | 1             |

obj_poses.json의 door 정보는 필요시에만 경유하며, 대부분의 경로는 중앙 복도를 통한 직접 연결이 가능합니다."""
             }
        )
    
    if image_path is not None:
        base64_image = convert_pgm_to_base64_png(image_path)
        yaml_path = image_path.replace(".pgm", ".yaml")
        yaml_content = read_yaml_content(yaml_path)
        
        # 이미지와 함께 yaml 내용과 graph_data도 함께 전달
        content_parts = [
            {"type": "image_url", 
             "image_url": {"url": f"data:image/png;base64,{base64_image}"}
            }
        ]
        
        # YAML 내용 추가
        if yaml_content:
            content_parts.append({
                "type": "text",
                "text": f"맵 YAML 파일 내용:\n{yaml_content}"
            })
        
        # Graph JSON 데이터 추가
        if graph_data:
            import json
            content_parts.append({
                "type": "text", 
                "text": f"객체 위치 정보 (obj_poses.json):\n{json.dumps(graph_data, indent=2, ensure_ascii=False)}"
            })
        
        message_history.append(
            {"role": "user", 
             "content": content_parts
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

    # 대화 이력을 유지하기 위한 변수
    message_history = []

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
            message_history = analyze_image_with_gpt4o(
                user_question="모바일 로봇이 사용할 Occupancy Grid Map 데이터를 분석해주세요.",
                image_path=image_path,
                graph_data=graph_data,
                message_history=message_history
            )
        else:
            message_history = analyze_image_with_gpt4o(
                user_input,
                graph_data=graph_data,
                message_history=message_history
            )
