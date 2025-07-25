# 뉴스 제목을 보고 카테고리를 분류하는 코드
from openai import OpenAI
import os

api_key = os.getenv('OPENAI_API_KEY')

client = OpenAI()
## 제로샷 예시
# completion = client.chat.completions.create(
#     model="gpt-4o-mini",
#     messages=[
#         {
#             "role": "system",
#             "content": "뉴스 제목을 보고 어떤 카테고리에 속하는지 알려줘."
#         },
#         {
#             "role": "user",
#             "content": "코스피 지수 상승"
#         },
#     ]
# )

# ## 원샷 예시
# completion = client.chat.completions.create(
#     model="gpt-4o-mini",
#     messages=[
#         {
#             "role": "system",
#             "content": "뉴스 제목을 보고 어떤 카테고리에 속하는지 알려줘."
#         },
#         {
#             "role": "user",
#             "content": "챔피언스 리그, 치열한 순위 다툼"
#         },
#         {
#             "role": "assistant",
#             "content": "스포츠"
#         },
#         {
#             "role": "user",
#             "content": "코스피 지수 상승"
#         },
#     ]
# )

## 퓨샷 예시
completion = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {
            "role": "system",
            "content": "뉴스 제목을 보고 어떤 카테고리에 속하는지 알려줘."
        },
        {
            "role": "user",
            "content": "챔피언스 리그, 치열한 순위 다툼"
        },
        {
            "role": "assistant",
            "content": "스포츠"
        },
        {
            "role": "user",
            "content": "자율주행 자동차, 내년부터 상용화 추진"
        },
        {
            "role": "assistant",
            "content": "기술"
        },
        {
            "role": "user",
            "content": "코스피 지수 상승"
        },
    ]
)

print(completion.choices[0].message.content)
print(completion)