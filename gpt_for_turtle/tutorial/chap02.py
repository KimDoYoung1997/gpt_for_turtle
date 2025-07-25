from openai import OpenAI
import os

api_key = os.getenv('OPENAI_API_KEY')

client = OpenAI(api_key=api_key)

completition = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {
            "role": "user",
            "content": "누구냐, 넌?"
        }
    ],
    temperature=0.0, # 0.0은 예측 결과가 항상 동일하게 나옴, 1.0은 예측 결과가 랜덤하게 나옴
    max_tokens=5
 )

print(completition.choices[0].message.content)