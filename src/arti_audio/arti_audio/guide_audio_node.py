"""
작품 설명·안내 음성 재생 노드 (T35).

받는 것
  explain_artwork (arti_interfaces/action/ExplainArtwork) : 작품 설명 재생 (artwork_zone → 여기)
  guide_message   (std_msgs/String)                       : 안내 키 (guide_lines.yaml의 guides)
부르는 것
  /arti/description (arti_interfaces/srv/Description, 서비스 클라이언트 → PC3) : 작품 번호 → 설명 문장
    네임스페이스 없는 전역 이름 (PC3 서버 하나를 두 로봇이 같이 씀). 명세: docs/common/ros2_interfaces.md

흐름
  artwork_id "art_02" → 번호 2 → /arti/description 요청 → PC3가 DB에서 description 응답  (content.py)
             → edge-tts로 mp3 합성(캐시)                                                (tts.py)
             → 재생 명령(pw-play) → 스피커                                               (player.py)
  PC3가 없거나 description_timeout_sec 안에 응답이 없으면 → 설명하지 않고 실패 (reason no_description)
  PC3가 success=false(없는 작품)·빈 설명을 주면 → 실패 (reason no_artwork)

규칙
  - 안내 음성이 오면 진행 중인 설명은 끊는다 (result.reason = preempted).
  - 안내 음성이 나오는 동안 새 설명 goal은 안내가 끝나거나 멈출 때까지 기다렸다가 재생한다.
  - 새 설명 goal이 오면 이전 설명은 끊는다 (preempted). 한 번에 설명 하나만.
  - 취소하면 0.3초 안에 멈춘다 (합성·재생 중 모두 50 ms마다 확인).
  - 시작할 때 안내 문구와 (PC3가 응답하면) 작품 설명을 미리 합성해 둔다 → 첫 재생 지연을 줄인다.
    작품 목록은 arti_common artworks.yaml. 설명 문장은 매번 PC3에 묻는다 (DB가 바뀌면 새로 합성).
"""

import os
import shlex
import threading
import time

from ament_index_python.packages import get_package_share_directory
from arti_interfaces.action import ExplainArtwork
from arti_interfaces.srv import Description
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from arti_audio.content import artwork_ids, artwork_number, GuideLines
from arti_audio.player import Player
from arti_audio.tts import TtsCache

POLL_SEC = 0.05          # 취소·끊김 확인 주기
FEEDBACK_SEC = 0.5       # 진행률 발행 주기
MP3_BITRATE_BPS = 48000  # edge-tts 출력 비트레이트 (진행률 추정용)


class GuideAudioNode(Node):

    def __init__(self):
        super().__init__('guide_audio_node')
        config = os.path.join(get_package_share_directory('arti_audio'), 'config')
        # 파라미터 (바꿀 때는 -p 이름:=값). 파일 경로를 비우면 설치된 share/arti_audio/config/의 파일
        self.declare_parameter('guide_lines_file', '')      # TTS 목소리·빠르기, 안내 문구 (guide_lines.yaml)
        self.declare_parameter('artworks_file', '')         # 미리 합성할 작품 목록 (비우면 arti_common artworks.yaml)
        self.declare_parameter('description_service', '/arti/description')   # PC3 작품 설명 서비스 (전역 이름)
        self.declare_parameter('description_timeout_sec', 3.0)   # PC3 응답 제한 [s] — 넘으면 실패
        self.declare_parameter('tts_cache_dir', '~/.cache/arti_audio/tts')   # 합성한 mp3 (저장소 밖)
        self.declare_parameter('tts_timeout_sec', 10)       # 합성 서버 접속 제한 [s] (받기는 3배)
        self.declare_parameter('prefetch', True)            # 시작할 때 안내·작품 설명을 미리 합성
        # 재생 명령 (뒤에 파일 경로가 붙는다). 기본은 PipeWire 기본 출력(노트북 스피커)
        #   음량 "pw-play --volume 0.8", 출력 장치(에어팟 등) "pw-play --target <sink 이름>" (wpctl status)
        self.declare_parameter('player_cmd', 'pw-play')

        p = self.get_parameter
        self.lines = GuideLines(p('guide_lines_file').value
                                or os.path.join(config, 'guide_lines.yaml'))
        self.artworks_file = p('artworks_file').value or os.path.join(
            get_package_share_directory('arti_common'), 'config', 'artworks.yaml')
        self.desc_timeout = float(p('description_timeout_sec').value)
        self.tts = TtsCache(p('tts_cache_dir').value, self.lines.voice, self.lines.rate,
                            int(p('tts_timeout_sec').value))
        self.player_cmd = shlex.split(p('player_cmd').value)

        self._lock = threading.Lock()
        self._guide = None          # 지금 나오는 안내 음성 Player
        self._explain_stop = None   # 지금 설명 중인 goal을 끊는 Event

        cb = ReentrantCallbackGroup()
        # 응답은 executor의 다른 스레드가 받는다 → 액션 실행 스레드는 future만 기다림 (같은 그룹이라도 Reentrant라 OK)
        self.desc_client = self.create_client(Description, p('description_service').value,
                                              callback_group=cb)
        self.create_subscription(String, 'guide_message', self._on_guide, 10,
                                 callback_group=cb)
        self._server = ActionServer(
            self, ExplainArtwork, 'explain_artwork',
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=cb)

        self.get_logger().info(
            f'준비됨: 목소리 {self.lines.voice}, 작품 설명 {self.desc_client.srv_name}, '
            f'재생 명령 "{" ".join(self.player_cmd)}", 캐시 {self.tts.cache_dir}')
        if p('prefetch').value:
            threading.Thread(target=self._prefetch, daemon=True).start()

    # ---------- 미리 합성 ----------

    def _prefetch(self):
        texts = [('guide', k, t) for k, t in self.lines.guides.items()]
        try:
            ids = artwork_ids(self.artworks_file)
        except Exception as e:  # 작품 목록이 이상해도 노드는 계속
            self.get_logger().warn(f'작품 목록 읽기 실패 (작품 설명 미리 합성 건너뜀): {e}')
            ids = []
        if ids and not self.desc_client.wait_for_service(timeout_sec=self.desc_timeout):
            self.get_logger().warn(f'{self.desc_client.srv_name} 없음 (PC3 확인) → 작품 설명 미리 합성 건너뜀')
            ids = []
        for aid in ids:
            text, why = self._ask_description(aid)
            if text:
                texts.append(('artwork', aid, text))
            else:
                self.get_logger().warn(f'작품 설명 못 받음 ({aid}: {why}) → 미리 합성 건너뜀')
        made, failed = 0, []
        for kind, key, text in texts:
            if self.tts.cached(text):
                continue
            try:
                self.tts.synthesize(text)
                made += 1
            except Exception as e:
                failed.append(f'{kind}.{key} ({type(e).__name__})')
        if failed:
            self.get_logger().warn(f'미리 합성 실패 (인터넷 확인): {", ".join(failed)}')
        self.get_logger().info(f'미리 합성 끝: 새로 {made}개, 전체 {len(texts)}개')

    def _start_player(self, path):
        try:
            return Player(self.player_cmd, path)
        except OSError as e:  # 재생 명령이 없을 때 등
            self.get_logger().error(f'재생 명령 실행 실패 ({self.player_cmd[0]}): {e}')
            return None

    # ---------- 안내 음성 (guide_message) ----------

    def _on_guide(self, msg):
        key = msg.data.strip()
        text = self.lines.guides.get(key)
        if text is None:
            self.get_logger().warn(f'모르는 안내 키: "{key}" (guide_lines.yaml의 guides)')
            return
        try:
            path = self.tts.cached(text) or self.tts.synthesize(text)
        except Exception as e:
            self.get_logger().error(f'안내 음성 합성 실패 ({key}): {e}')
            return
        with self._lock:
            if self._explain_stop is not None:   # 안내가 설명보다 우선
                self._explain_stop.set()
            if self._guide is not None:          # 이전 안내는 새 안내로 바꾼다
                self._guide.stop()
            self._guide = self._start_player(path)
        self.get_logger().info(f'안내 음성: {key}')

    def _guide_playing(self):
        with self._lock:
            return self._guide is not None and self._guide.is_playing()

    # ---------- 작품 설명 (explain_artwork 액션) ----------

    def _on_accepted(self, goal_handle):
        with self._lock:
            if self._explain_stop is not None:   # 이전 설명 끊기
                self._explain_stop.set()
        goal_handle.execute()

    def _finish(self, goal_handle, how, completed, reason):
        getattr(goal_handle, how)()   # succeed / abort / canceled
        self.get_logger().info(
            f'설명 끝: {goal_handle.request.artwork_id} ({reason})')
        return ExplainArtwork.Result(completed=completed, reason=reason)

    def _wait(self, goal_handle, stop, busy, tick=None):
        """busy()가 참인 동안 POLL_SEC마다 기다린다 (tick이 있으면 매번 부름).
        취소·끊김이면 바로 (how, reason), 끝까지 기다렸으면 None."""
        while busy():
            if goal_handle.is_cancel_requested:
                return 'canceled', 'canceled'
            if stop.is_set():
                return 'abort', 'preempted'
            if tick is not None:
                tick()
            time.sleep(POLL_SEC)
        return None

    def _ask_description(self, artwork_id, goal_handle=None, stop=None):
        """PC3에 작품 설명을 묻는다. 돌려줌: (description, '') 또는 (None, 실패 이유).
        실패 이유: no_artwork (번호 없음·success=false·빈 설명) / no_description (서버 없음·시간 초과)
                   / canceled·preempted (goal_handle이 있을 때, 기다리는 중 취소·끊김)"""
        num = artwork_number(artwork_id)
        if num is None:
            return None, 'no_artwork'
        deadline = time.monotonic() + self.desc_timeout

        def waiting(done):
            return lambda: not done() and time.monotonic() < deadline

        def wait(done):
            if goal_handle is None:
                while waiting(done)():
                    time.sleep(POLL_SEC)
                return None
            return self._wait(goal_handle, stop, waiting(done))

        hit = wait(self.desc_client.service_is_ready)
        if hit:
            return None, hit[1]
        if not self.desc_client.service_is_ready():
            return None, 'no_description'
        future = self.desc_client.call_async(Description.Request(artwork_id=num))
        hit = wait(future.done)
        if hit:
            future.cancel()
            return None, hit[1]
        if not future.done():
            future.cancel()
            return None, 'no_description'
        res = future.result()
        text = (res.description or '').strip() if res is not None else ''
        if res is None or not res.success or not text:
            return None, 'no_artwork'
        return text, ''

    def _execute(self, goal_handle):
        stop = threading.Event()
        with self._lock:
            self._explain_stop = stop
        try:
            return self._play_explain(goal_handle, goal_handle.request.artwork_id, stop)
        finally:
            with self._lock:
                if self._explain_stop is stop:
                    self._explain_stop = None

    def _play_explain(self, goal_handle, artwork_id, stop):
        # 1) PC3에 작품 설명 요청 (기다리는 중에도 50 ms마다 취소·끊김 확인)
        text, why = self._ask_description(artwork_id, goal_handle, stop)
        if why in ('canceled', 'preempted'):
            return self._finish(goal_handle, 'canceled' if why == 'canceled' else 'abort', False, why)
        if not text:
            if why == 'no_description':
                self.get_logger().error(f'작품 설명 응답 없음: {self.desc_client.srv_name} '
                                        f'({self.desc_timeout:.0f} s, PC3 확인) → 설명 안 함')
            else:
                self.get_logger().warn(f'작품 설명 없음: artwork_id="{artwork_id}" (PC3 success=false·빈 설명)')
            return self._finish(goal_handle, 'abort', False, why)

        # 2) 음성 합성 (캐시에 없을 때만)
        #    합성은 몇 초 걸려서 별도 스레드에서 하고, 여기서는 50 ms마다 취소를 확인한다.
        path = self.tts.cached(text)
        if path is None:
            self.get_logger().info(f'합성 중: {artwork_id}')
            out = {}

            def work():
                try:
                    out['path'] = self.tts.synthesize(text)
                except Exception as e:
                    out['error'] = e

            worker = threading.Thread(target=work, daemon=True)
            worker.start()
            hit = self._wait(goal_handle, stop, worker.is_alive)
            if hit:  # 합성은 뒤에서 마저 끝나 캐시에 남는다
                return self._finish(goal_handle, hit[0], False, hit[1])
            if 'error' in out:
                self.get_logger().error(
                    f'음성 합성 실패 ({artwork_id}, 인터넷 확인): {out["error"]}')
                return self._finish(goal_handle, 'abort', False, 'tts_error')
            path = out['path']

        # 3) 안내 음성이 끝나거나 멈출 때까지 기다린다 (새 안내가 오면 그것까지)
        hit = self._wait(goal_handle, stop, self._guide_playing)
        if hit:
            return self._finish(goal_handle, hit[0], False, hit[1])

        # 4) 재생 (진행률은 파일 크기로 추정한 길이 기준)
        player = self._start_player(path)
        if player is None:
            return self._finish(goal_handle, 'abort', False, 'player_error')
        self.get_logger().info(f'설명 시작: {artwork_id}')

        duration = max(os.path.getsize(path) * 8.0 / MP3_BITRATE_BPS, 0.1)  # 대략
        t0 = time.monotonic()
        last_fb = [0.0]
        feedback = ExplainArtwork.Feedback()

        def send_progress():
            now = time.monotonic()
            if now - last_fb[0] >= FEEDBACK_SEC:
                feedback.progress = float(min((now - t0) / duration, 1.0))
                goal_handle.publish_feedback(feedback)
                last_fb[0] = now

        hit = self._wait(goal_handle, stop, player.is_playing, send_progress)
        if hit:
            player.stop()
            return self._finish(goal_handle, hit[0], False, hit[1])
        if player.poll() != 0:
            self.get_logger().error(
                f'재생 실패 (종료 코드 {player.poll()}): {player.stderr_text()}')
            return self._finish(goal_handle, 'abort', False, 'player_error')
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        return self._finish(goal_handle, 'succeed', True, 'done')

    def stop_all(self):
        with self._lock:
            if self._explain_stop is not None:
                self._explain_stop.set()
            if self._guide is not None:
                self._guide.stop()


def main(args=None):
    rclpy.init(args=args)
    node = GuideAudioNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C / SIGTERM
    finally:
        node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
