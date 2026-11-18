"""작품별 TTS 해설 문장 (하드코딩).

DB 연결 전 사용할 짧은 임시 작품 해설. 나중에 DB 조회로 교체할 수 있습니다.
키는 a_course.yaml의 artwork_id 와 같아야 합니다.
"""

# artwork_id -> 읽어줄 문장
EXPLANATIONS = {
    '1': '1번 작품을 감상하세요.',
    '2': '2번 작품을 감상하세요.',
    '3': '3번 작품을 감상하세요.',
    '4': '4번 작품을 감상하세요.',
}


def explanation_for(artwork_id: str) -> str:
    """작품 ID에 해당하는 해설 문장 반환. 등록 안 된 ID면 ValueError."""
    try:
        return EXPLANATIONS[str(artwork_id)]
    except KeyError as exc:
        raise ValueError(f'해설이 등록되지 않은 작품 ID: {artwork_id}') from exc
