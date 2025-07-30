from openai import OpenAI
import os 

api_key = os.getenv('OPENAI_API_KEY')
client = OpenAI(api_key=api_key)
def ask(question,message_history=[]):
    if len(message_history)==0:
        message_history.append(
            {"role": "system", 
             "content": "You are a helpful assistant."
             }
        )

        
    # 사용자 질문 추가
    message_history.append(
        {"role": "user", 
         "content": question})
    # GPT에 질문을 던져 답변을 받음
    completion=client.chat.completions.create(
        model="gpt-4o-mini",
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
    while True:
        user_input = input("사용자 입력: ")

        if user_input.lower() in ["종료", "exit", "quit"]:
            print("종료합니다.!")
            break
        ask(user_input)