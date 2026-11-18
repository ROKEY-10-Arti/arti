"""읽을 문장 준비: 안내 문구(guide_lines.yaml)와 작품 설명(PC3 /arti/description 서비스).

작품 설명의 원본은 PC3(키오스크/관제)의 SQLite DB다 (docs/common/ros2_interfaces.md).
config/artworks_sample.yaml은 PC3 없이 시험할 때 fake_description_server가 읽는 샘플이다.
"""

import re

import yaml


def load_yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


class GuideLines:
    """guide_lines.yaml: TTS 설정(목소리·빠르기), 안내 키 → 문장."""

    def __init__(self, path):
        data = load_yaml(path)
        tts = data.get('tts') or {}
        self.voice = tts.get('voice', 'ko-KR-SunHiNeural')
        self.rate = tts.get('rate', '+0%')
        self.guides = {str(k): str(v).strip() for k, v in (data.get('guides') or {}).items()}


def artwork_number(artwork_id):
    """'art_02' → 2. ROS 통신(Description.srv, StartTour)의 작품 번호는 int32로 통일. 숫자가 없으면 None."""
    m = re.search(r'(\d+)$', str(artwork_id).strip())
    return int(m.group(1)) if m else None


def artwork_ids(path):
    """arti_common artworks.yaml의 작품 ID 목록 ('art_01' ...) — 시작할 때 미리 합성할 대상."""
    return [str(r['artwork_id']) for r in (load_yaml(path).get('artworks') or []) if r.get('artwork_id')]


def sample_descriptions(path):
    """artworks_sample.yaml → {작품 번호: description} (fake_description_server용)."""
    out = {}
    for r in load_yaml(path).get('artworks') or []:
        num = artwork_number(r.get('artwork_id', ''))
        text = str(r.get('description') or '').strip()
        if num is not None and text:
            out[num] = text
    return out
