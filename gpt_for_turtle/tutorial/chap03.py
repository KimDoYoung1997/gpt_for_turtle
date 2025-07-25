from openai import OpenAI
import os

api_key = os.getenv('OPENAI_API_KEY')

client = OpenAI(api_key=api_key)

# role : 역할 
    # system : chatgpt를 어떤 사람이라고 취급. 예를 들어 상담사, 음악을 좋아하는 사람, 코딩 전문가 등 이라고 지정
    # assistant : 챗봇의 역할을 지정
    # user : 사용자의 역할을 지정

# completition = client.chat.completions.create(
#     model="gpt-4o-mini",
#     messages=[
#         # {
#         #     "role": "system",
#         #     "content": "당신은 프로그래밍 전문가입니다. 사용자가 입력한 분야에 필요한 기술과 학습 방법을 3문장 이내로 알려주세요."
#         # },
#         {
#             "role": "user",
#             "content": "보안 전문가"
#         }
#     ],
#  )

completition = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {
            "role": "system",
            "content": "당신은 프로그래밍 학습 조언가입니다."
        },
        {
            "role": "user",
            "content": "초보자가 배우기 좋은 언어를 하나 추천해줘. 이름만 알려줘."
        },
        {
            "role": "assistant",
            # "content": "파이썬(Python)"
            "content": "자바(Java)"

        },
        {
            "role": "user",
            "content": "어떤 분야에 활용할 수 있어? 간단히 설명해줘."
        },
        
    ],
)

print(completition.choices[0].message.content)