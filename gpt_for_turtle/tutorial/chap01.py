from openai import OpenAI
import os


api_key = os.getenv('OPENAI_API_KEY')
if not api_key:
    print('OPENAI_API_KEY 환경 변수가 설정되지 않았습니다.')
    
client = OpenAI(api_key=api_key)

completion = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {
            "role": "user",
            "content": "누구냐, 넌?"
        }
        
    ]
)

print(completion.choices[0].message.content)
print(type(completion))
