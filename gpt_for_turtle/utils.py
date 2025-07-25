import base64
import io
import json
import yaml
from PIL import Image

def convert_pgm_to_base64_png(pgm_path):
    """PGM 파일을 PNG로 변환 후 base64로 인코딩"""
    # PGM 파일을 PIL Image로 열기
    image = Image.open(pgm_path)
    
    # PNG 형식으로 변환하여 메모리 버퍼에 저장
    buffer = io.BytesIO()
    image.save(buffer, format='PNG')
    buffer.seek(0)
    
    # base64로 인코딩
    return base64.b64encode(buffer.read()).decode('utf-8')

def read_yaml_content(yaml_path):
    """YAML 파일 내용 읽기"""
    with open(yaml_path, "r") as yaml_file:
        return yaml_file.read()

def read_json_content(json_path):
    """JSON 파일 내용 읽기"""
    with open(json_path, "r") as json_file:
        return json.load(json_file) 