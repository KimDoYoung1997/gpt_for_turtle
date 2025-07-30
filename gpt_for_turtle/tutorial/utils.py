from PIL import Image
import io
import base64
def encode_image_to_base64(image_path):
    """이미지 파일을 base64로 인코딩하는 함수"""
    try:
        with Image.open(image_path) as img:
            # PNG로 변환하여 호환성 확보
            if img.mode == 'RGBA':
                img = img.convert('RGB')
            
            # 메모리에 PNG로 저장
            buffer = io.BytesIO()
            img.save(buffer, format='PNG')
            
            # base64 인코딩
            image_data = buffer.getvalue()
            base64_image = base64.b64encode(image_data).decode('utf-8')
            
            return base64_image
    except Exception as e:
        print(f"이미지 인코딩 오류: {e}")
        return None

