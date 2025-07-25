from openai import OpenAI
import time

client = OpenAI()

def real_multiturn_test():
    """진짜 Multi-turn: 매번 API 호출"""
    print("🔴 진짜 Multi-turn (chap05.py 방식)")
    print("=" * 50)
    
    messages = [
        {
            "role": "system",
            "content": "당신은 AI 어시스턴트입니다. 짧게 답변해주세요."
        }
    ]
    
    questions = [
        "안녕하세요! 제 이름은 김철수입니다.",
        "제 이름이 뭐라고 했죠?", 
        "저는 몇 살일까요? 추측해보세요."
    ]
    
    api_calls = 0
    total_tokens = 0
    
    for i, question in enumerate(questions, 1):
        print(f"\n--- {i}번째 대화 ---")
        print(f"사용자: {question}")
        
        # 사용자 메시지 추가
        messages.append({
            "role": "user",
            "content": question
        })
        
        # API 호출
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=messages,
            temperature=0.1
        )
        
        api_calls += 1
        total_tokens += response.usage.total_tokens
        
        assistant_response = response.choices[0].message.content
        print(f"어시스턴트: {assistant_response}")
        
        # 어시스턴트 응답 추가
        messages.append({
            "role": "assistant",
            "content": assistant_response
        })
        
        print(f"토큰 사용: {response.usage.total_tokens}")
        time.sleep(1)  # API 제한 방지
    
    print(f"\n📊 진짜 Multi-turn 결과:")
    print(f"   API 호출 횟수: {api_calls}번")
    print(f"   총 토큰 사용: {total_tokens}")
    return messages

def fake_multiturn_test():
    """가짜 Multi-turn: 한 번의 API 호출로 시뮬레이션"""
    print("\n🔵 가짜 Multi-turn (test_method2 방식)")
    print("=" * 50)
    
    # 미리 작성된 대화 히스토리 (가짜)
    messages = [
        {
            "role": "system",
            "content": "당신은 AI 어시스턴트입니다. 짧게 답변해주세요."
        },
        {
            "role": "user",
            "content": "안녕하세요! 제 이름은 김철수입니다."
        },
        {
            "role": "assistant",
            "content": "안녕하세요 김철수님! 반갑습니다."  # 가짜 응답
        },
        {
            "role": "user", 
            "content": "제 이름이 뭐라고 했죠?"
        },
        {
            "role": "assistant",
            "content": "김철수라고 하셨습니다."  # 가짜 응답
        },
        {
            "role": "user",
            "content": "저는 몇 살일까요? 추측해보세요."
        }
    ]
    
    # 한 번의 API 호출
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
        temperature=0.1
    )
    
    print("대화 히스토리 (미리 작성됨):")
    for msg in messages[1:]:  # system 제외
        role = "사용자" if msg["role"] == "user" else "어시스턴트"
        print(f"{role}: {msg['content']}")
    
    print(f"\n최종 GPT 응답: {response.choices[0].message.content}")
    
    print(f"\n📊 가짜 Multi-turn 결과:")
    print(f"   API 호출 횟수: 1번")
    print(f"   총 토큰 사용: {response.usage.total_tokens}")

def compare_results(real_messages):
    """두 방식의 결과 비교"""
    print("\n" + "="*60)
    print("🔍 결과 비교 분석")
    print("="*60)
    
    print("🔴 진짜 Multi-turn 특징:")
    print("   ✅ 실제 대화 맥락 유지")
    print("   ✅ GPT가 이전 대화를 정확히 기억")
    print("   ❌ 여러 번 API 호출 (비용 증가)")
    print("   ❌ 대화가 길어질수록 토큰 급증")
    
    print("\n🔵 가짜 Multi-turn 특징:")
    print("   ✅ 한 번의 API 호출 (비용 절약)")
    print("   ✅ 빠른 응답")
    print("   ❌ 중간 응답들이 실제 GPT 응답이 아님")
    print("   ❌ 대화 흐름이 부자연스러울 수 있음")
    
    print(f"\n💡 결론: chap05.py는 진짜 multi-turn이라 매번 API를 호출하며,")
    print(f"   점점 더 많은 토큰을 소비합니다!")

if __name__ == "__main__":
    print("🧪 진짜 vs 가짜 Multi-turn 비교 테스트")
    print("="*60)
    
    try:
        real_messages = real_multiturn_test()
        fake_multiturn_test()
        compare_results(real_messages)
        
    except Exception as e:
        print(f"❌ 테스트 중 오류: {e}") 