# 뉴스 제목을 보고 카테고리를 분류하는 코드
from openai import OpenAI
import os

api_key = os.getenv('OPENAI_API_KEY')

client = OpenAI()
messages = [
    {
        "role": "system",
        "content": "You are a helpful assistant."   # 기본
    }
]

while True:
    user_input = input("사용자 입력: ")

    if user_input.lower() in ["종료", "exit", "quit"]:
        print("챗봇 : 안녕히 가세요!")
        break

    messages.append({
        "role": "user",
        "content": user_input
    })

    completion = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages
    )

    response = completion.choices[0].message.content

    print("챗봇:", response)
    messages.append({
        "role": "assistant",
        "content": response
    })
