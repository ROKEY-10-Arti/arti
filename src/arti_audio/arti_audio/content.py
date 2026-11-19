"""읽을 문장 준비: 안내 문구(guide_lines.yaml, 로봇이 가짐)와 작품 목록(arti_common artworks.yaml).

작품 설명 문장은 로봇 PC에 두지 않고 PC3(키오스크/관제)의 SQLite DB에 요청해서 받는다
(/arti/description, docs/common/ros2_interfaces.md).
"""

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


def artwork_ids(path):
    """arti_common artworks.yaml의 작품 번호 목록 [1, 2, ...] — 시작할 때 미리 합성할 대상.
    작품 ID는 ROS 통신에서 int32로 통일 (ExplainArtwork, Description, StartTour)."""
    return [int(r['artwork_id']) for r in (load_yaml(path).get('artworks') or []) if r.get('artwork_id')]
