"""ArtI 데모용 한 문장 TTS 노드. 음성 재생이 '끝난 뒤에만' 완료 신호를 발행합니다.

a_course_node 는 이 완료 신호를 받아야 다음 작품으로 이동하므로,
'설명이 끝난 다음에 이동'이 보장됩니다.

입력:  /robot4/arti/explanation_request  std_msgs/String JSON
       {"request_id":"...", "artwork_id":"2"}
       /robot4/guide_message             std_msgs/String  (예: '안녕히가세요')
출력:  /robot4/arti/explanation_result   std_msgs/String JSON
       {"request_id":"...", "artwork_id":"2", "success":true}

gtts 엔진은 인터넷과 mpg123을 사용하고, espeak 엔진은 로컬 espeak-ng를 사용합니다.
실제 오디오는 이 노드가 실행된 PC의 기본 오디오 출력 장치에서 나옵니다.
"""
from __future__ import annotations

import json
from pathlib import Path
from queue import SimpleQueue, Empty
import subprocess
from tempfile import TemporaryDirectory
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .hardcoded_explanations import explanation_for


def play_text(text: str, engine: str) -> None:
    """text를 음성으로 재생하고, 재생이 끝날 때까지 블로킹."""
    if engine == 'gtts':
        # Google TTS로 mp3 생성 -> mpg123으로 재생 (인터넷 필요)
        from gtts import gTTS  # 선택 의존성; ROS 노드가 쓰는 Python 환경(venv)에 설치 필요
        with TemporaryDirectory(prefix='arti_tts_') as directory:
            filename = Path(directory) / 'speech.mp3'
            gTTS(text=text, lang='ko').save(str(filename))
            subprocess.run(['mpg123', '-q', str(filename)], check=True, timeout=90)
    elif engine == 'espeak':
        # 오프라인 엔진
        subprocess.run(['espeak-ng', '-v', 'ko', '-s', '125', text],
                       check=True, timeout=90)
    else:
        raise ValueError(f'지원하지 않는 TTS 엔진: {engine} (gtts 또는 espeak)')


class HardcodedTTSNode(Node):
    def __init__(self):
        super().__init__('hardcoded_tts_node')
        self.declare_parameter('engine', 'gtts')   # 'gtts' 또는 'espeak'
        self._result_pub = self.create_publisher(String, 'arti/explanation_result', 10)
        # 작품 해설 요청 (완료 응답 필요)
        self.create_subscription(String, 'arti/explanation_request', self._on_request, 10)
        # 출구 인사 등 안내 문구: guide_message 텍스트를 읽어 줌 (내부 키 'start','tour_end'는 무시).
        self.create_subscription(String, 'guide_message', self._on_guide, 10)
        # 재생 스레드 -> ROS 스레드로 결과 전달하는 큐
        self._completed = SimpleQueue()
        self._busy = False                         # 해설 재생 중 여부 (중복 요청 방지)
        self.create_timer(0.1, self._flush_completed)
        self.get_logger().info('작품 1~4 한 문장 TTS 준비 완료; 오디오는 이 PC에서 재생')

    def _reply(self, request_id: str, artwork_id: str, success: bool):
        """arti/explanation_result 로 완료/실패 응답 발행."""
        result = {'request_id': request_id, 'artwork_id': artwork_id,
                  'success': bool(success)}
        self._result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))

    def _on_request(self, msg: String):
        """해설 요청 수신 -> 문장 찾기 -> 별도 스레드에서 재생 시작."""
        try:
            body = json.loads(msg.data)
            if not isinstance(body, dict):
                raise ValueError('요청은 JSON 객체여야 합니다')
            request_id = body.get('request_id')
            artwork_id = str(body.get('artwork_id', ''))
            if not isinstance(request_id, str) or not request_id:
                raise ValueError('request_id가 필요합니다')
            text = explanation_for(artwork_id)   # hardcoded_explanations.py 에서 문장 조회
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'잘못된 TTS 요청: {exc}')
            return

        if self._busy:
            self.get_logger().error('이전 해설 재생 중; 중복 TTS 요청 거절')
            self._reply(request_id, artwork_id, False)
            return

        self._busy = True
        engine = str(self.get_parameter('engine').value)
        self.get_logger().info(f'작품 {artwork_id} 음성 해설 시작: {text}')
        # 재생은 블로킹이므로 스레드로 분리 (ROS 콜백이 멈추지 않도록)
        Thread(target=self._worker,
               args=(request_id, artwork_id, text, engine), daemon=True).start()

    def _on_guide(self, msg: String):
        """guide_message 수신 -> 응답 없이 읽기만 함 (출구 '안녕히가세요' 등)."""
        text = msg.data.strip()
        if not text or text in ('start', 'tour_end'):
            return   # 내부 신호용 키워드는 읽지 않음
        engine = str(self.get_parameter('engine').value)
        self.get_logger().info(f'안내 음성: {text}')
        Thread(target=self._speak_only, args=(text, engine), daemon=True).start()

    def _speak_only(self, text: str, engine: str):
        """안내 문구 재생 스레드 (결과 응답 없음)."""
        try:
            play_text(text, engine)
        except Exception as exc:
            self.get_logger().error(f'안내 음성 실패: {exc}')

    def _worker(self, request_id: str, artwork_id: str, text: str, engine: str):
        """해설 재생 스레드: 재생 끝나면 결과를 큐에 넣음."""
        success = False
        failure = ''
        try:
            play_text(text, engine)
            success = True
        except Exception as exc:
            failure = str(exc)
        self._completed.put((request_id, artwork_id, success, failure))

    def _flush_completed(self):
        """0.1초마다 큐 확인 -> 재생 끝난 해설의 결과를 ROS로 발행."""
        while True:
            try:
                request_id, artwork_id, success, failure = self._completed.get_nowait()
            except Empty:
                break
            self._busy = False
            if failure:
                self.get_logger().error(f'작품 {artwork_id} TTS 실패: {failure}')
            else:
                self.get_logger().info(f'작품 {artwork_id} 음성 해설 완료')
            self._reply(request_id, artwork_id, success)


def main(args=None):
    rclpy.init(args=args)
    node = HardcodedTTSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
