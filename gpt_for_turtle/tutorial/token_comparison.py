from openai import OpenAI
import tiktoken

client = OpenAI()

def count_tokens(text, model="gpt-4o-mini"):
    """토큰 개수를 계산하는 함수"""
    encoding = tiktoken.encoding_for_model(model)
    return len(encoding.encode(text))

def test_token_consumption():
    """두 방식의 토큰 소비량 비교"""
    print("🔢 토큰 소비량 비교 테스트")
    print("=" * 50)
    
    # Method 1: 멀티 content (짧은 메시지)
    method1_messages = [
        {
            "role": "system",
            "content": "당신은 AI 어시스턴트입니다."
        },
        {
            "role": "user", 
            "content": "질문1: 안녕하세요! 질문2: 날씨가 어때요? 질문3: 파이썬을 배우고 싶어요."
        }
    ]
    
    # Method 2: 대화 히스토리 (긴 메시지)
    method2_messages = [
        {
            "role": "system",
            "content": "당신은 AI 어시스턴트입니다."
        },
        {
            "role": "user",
            "content": "질문1: 안녕하세요!"
        },
        {
            "role": "assistant", 
            "content": "안녕하세요! 반갑습니다! 어떻게 도와드릴까요?"
        },
        {
            "role": "user",
            "content": "질문2: 오늘 날씨가 어때요?"
        },
        {
            "role": "assistant",
            "content": "죄송하지만 실시간 날씨 정보는 확인할 수 없습니다. 날씨 앱을 확인해보세요."
        },
        {
            "role": "user", 
            "content": "질문3: 파이썬을 배우고 싶어요."
        }
    ]
    
    # 토큰 계산
    def calculate_total_tokens(messages):
        total = 0
        for msg in messages:
            total += count_tokens(f"{msg['role']}: {msg['content']}")
        return total
    
    tokens1 = calculate_total_tokens(method1_messages)
    tokens2 = calculate_total_tokens(method2_messages)
    
    print(f"📊 Method 1 (멀티 content): {tokens1} 토큰")
    print(f"📊 Method 2 (대화 히스토리): {tokens2} 토큰")
    print(f"💰 토큰 차이: {tokens2 - tokens1} ({((tokens2/tokens1-1)*100):.1f}% 더 비쌈)")
    
    # 실제 API 호출해서 usage 확인
    print("\n🚀 실제 API 호출 테스트...")
    
    # Method 1 호출
    response1 = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=method1_messages,
        temperature=0.1
    )
    
    # Method 2 호출  
    response2 = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=method2_messages,
        temperature=0.1
    )
    
    print(f"📈 실제 사용량:")
    print(f"  Method 1 - Input: {response1.usage.prompt_tokens}, Output: {response1.usage.completion_tokens}")
    print(f"  Method 2 - Input: {response2.usage.prompt_tokens}, Output: {response2.usage.completion_tokens}")
    print(f"  💸 Input 토큰 차이: {response2.usage.prompt_tokens - response1.usage.prompt_tokens}")

if __name__ == "__main__":
    test_token_consumption() 