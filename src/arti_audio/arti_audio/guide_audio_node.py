"""
작품 설명·안내 음성 재생 노드 (T35).

받는 것
  explain_artwork (arti_interfaces/action/ExplainArtwork) : 작품 설명 재생 (artwork_zone → 여기)
  guide_message   (std_msgs/String)                       : 안내 키 (guide_lines.yaml의 guides)
부르는 것
  /arti/description (arti_interfaces/srv/Description, 서비스 클라이언트 → PC3) : 작품 번호 → 설명 문장
    네임스페이스 없는 전역 이름 (PC3 서버 하나를 두 로봇이 같이 씀). 명세: docs/common/ros2_interfaces.md

작품 설명은 PC3(DB)에 요청해서 받고, 안내 문구는 로봇의 guide_lines.yaml에 있다.

흐름
  작품 번호 2 → /arti/description 요청 → PC3가 설명 문장 응답      (안내: 키 → guide_lines.yaml 문장)
             → edge-tts로 mp3 합성(캐시)    (tts.py)
             → 재생 명령(pw-play) → 스피커   (player.py)
  작품 설명 실패
    PC3가 없거나 description_timeout_sec 안에 응답이 없음 → reason no_description
    PC3가 success=false(없는 작품)·빈 설명을 줌          → reason no_artwork
    → 관람객에게 "작품 설명을 불러오지 못했습니다"(guide_lines.yaml explain_failed)를 말한다

규칙
  - 안내 음성이 오면 진행 중인 설명은 끊는다 (result.reason = preempted).
  - 안내 음성이 나오는 동안 새 설명 goal은 안내가 끝나거나 멈출 때까지 기다렸다가 재생한다.
  - 새 설명 goal이 오면 이전 설명은 끊는다 (preempted). 한 번에 설명 하나만.
  - 취소하면 0.3초 안에 멈춘다 (요청·합성·재생 중 모두 50 ms마다 확인).
  - 시작할 때 안내 문구와 (PC3가 응답하면) 작품 설명(artworks.yaml의 작품)을 미리 합성 → 첫 재생 지연을 줄인다.
    설명 문장은 매번 PC3에 묻는다 (DB 문장이 바뀌면 새로 합성).
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

from arti_audio.content import artwork_ids, GuideLines
from arti_audio.player import Player
from arti_audio.tts import TtsCache

POLL_SEC = 0.05                          # 취소·끊김 확인 주기
DESCRIPTION_SERVICE = '/arti/description'  # PC3 작품 설명 서비스 (팀 명세의 전역 이름, docs/common/ros2_interfaces.md)
TTS_CACHE_DIR = '~/.cache/arti_audio/tts'  # 합성한 mp3 (저장소 밖)
TTS_TIMEOUT_SEC = 10                     # 합성 서버(edge-tts) 접속 제한 [s] (받기는 3배)


class GuideAudioNode(Node):

    def __init__(self):
        super().__init__('guide_audio_node')
        # 파라미터 (바꿀 때는 -p 이름:=값). 현장에서 바꿀 일이 있는 것만 둔다
        self.declare_parameter('description_timeout_sec', 3.0)   # PC3 응답 제한 [s] — 넘으면 실패
        # 재생 명령 (뒤에 파일 경로가 붙는다). 기본은 PipeWire 기본 출력(노트북 스피커)
        #   음량 "pw-play --volume 0.8", 출력 장치(에어팟 등) "pw-play --target <sink 이름>" (wpctl status)
        self.declare_parameter('player_cmd', 'pw-play')

        p = self.get_parameter
        # 안내 문구·목소리 = 설치된 share/arti_audio/config/guide_lines.yaml
        self.lines = GuideLines(os.path.join(get_package_share_directory('arti_audio'), 'config', 'guide_lines.yaml'))
        # 미리 합성할 작품 목록 = 공용 arti_common/config/artworks.yaml
        self.artworks_file = os.path.join(get_package_share_directory('arti_common'), 'config', 'artworks.yaml')
        self.timeout = float(p('description_timeout_sec').value)
        self.tts = TtsCache(TTS_CACHE_DIR, self.lines.voice, self.lines.rate, TTS_TIMEOUT_SEC)
        self.player_cmd = shlex.split(p('player_cmd').value)

        self._lock = threading.Lock()
        self._guide = None          # 지금 나오는 안내 음성 Player
        self._explain_stop = None   # 지금 설명 중인 goal을 끊는 Event

        # 콜백 그룹을 Reentrant로 하는 이유 (main의 MultiThreadedExecutor와 짝)
        #   _execute(설명 액션)는 재생이 끝날 때까지 수십 초 동안 콜백 하나를 붙잡고 기다린다. 그동안에도
        #   ① guide_message 콜백이 돌아야 안내 음성이 설명을 끊을 수 있고 (preempted)
        #   ② cancel·새 goal 콜백이 돌아야 취소(0.3 s 안)·다음 설명 끊기가 되고
        #   ③ PC3 서비스 응답 콜백이 돌아야 _call()이 기다리는 future가 끝난다.
        #   MutuallyExclusive(기본)면 같은 그룹 콜백은 한 번에 하나뿐이라 ①②③이 _execute가 끝날 때까지 못 돈다
        #   → 서비스 응답을 못 받아 늘 시간 초과, 취소도 안 먹힘. Reentrant는 같은 그룹 콜백도 여러 스레드에서 동시에 돈다.
        #   (공유 상태 _guide·_explain_stop은 self._lock으로 보호)
        cb = ReentrantCallbackGroup()
        self.desc_client = self.create_client(Description, DESCRIPTION_SERVICE, callback_group=cb)
        self.create_subscription(String, 'guide_message', self._on_guide, 10, callback_group=cb)
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
        threading.Thread(target=self._prefetch, daemon=True).start()   # 시작할 때 미리 합성 (뒤에서)

    # ---------- PC3에 작품 설명 요청 ----------

    def _call(self, client, request, goal_handle=None, stop=None):
        """서비스를 부르고 응답을 기다린다 (최대 description_timeout_sec).
        goal_handle이 있으면 기다리는 중에도 50 ms마다 취소·끊김 확인.
        돌려줌: (응답, '') 또는 (None, 'no_response' / 'canceled' / 'preempted')"""
        deadline = time.monotonic() + self.timeout

        def wait(done):
            busy = lambda: not done() and time.monotonic() < deadline   # noqa: E731
            if goal_handle is None:
                while busy():
                    time.sleep(POLL_SEC)
                return None
            return self._wait(goal_handle, stop, busy)

        hit = wait(client.service_is_ready)
        if hit:
            return None, hit[1]
        if not client.service_is_ready():
            return None, 'no_response'
        future = client.call_async(request)
        hit = wait(future.done)
        if hit or not future.done():
            future.cancel()
            return None, hit[1] if hit else 'no_response'
        return future.result(), ''

    def _ask_description(self, artwork_id, goal_handle=None, stop=None):
        """작품 번호 → (설명, '') 또는 (None, no_artwork / no_description / canceled / preempted)."""
        if artwork_id <= 0:
            return None, 'no_artwork'
        res, why = self._call(self.desc_client, Description.Request(artwork_id=artwork_id), goal_handle, stop)
        if why:
            return None, 'no_description' if why == 'no_response' else why
        text = (res.description or '').strip()
        return (text, '') if res.success and text else (None, 'no_artwork')

    # ---------- 미리 합성 ----------

    def _prefetch(self):
        texts = [(f'guide.{k}', t) for k, t in self.lines.guides.items()]
        try:
            ids = artwork_ids(self.artworks_file)
        except Exception as e:  # 작품 목록이 이상해도 노드는 계속
            self.get_logger().warn(f'작품 목록 읽기 실패 (작품 설명 미리 합성 건너뜀): {e}')
            ids = []
        if ids and not self.desc_client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().warn(f'{self.desc_client.srv_name} 없음 (PC3 확인) → 작품 설명 미리 합성 건너뜀')
            ids = []
        for aid in ids:
            text, why = self._ask_description(aid)
            if text:
                texts.append((f'artwork.{aid}', text))
            else:
                self.get_logger().warn(f'작품 설명 못 받음 ({aid}: {why}) → 미리 합성 건너뜀')
        made, failed = 0, []
        for name, text in texts:
            if self.tts.cached(text):
                continue
            try:
                self.tts.synthesize(text)
                made += 1
            except Exception as e:
                failed.append(f'{name} ({type(e).__name__})')
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
        self._say(msg.data.strip())

    def _say(self, key):
        """안내 키(guide_lines.yaml guides) 하나를 재생한다. guide_message 토픽과 설명 실패 안내가 같이 쓴다."""
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

    def _fail(self, goal_handle, reason):
        """설명 실패: 관람객에게 "작품 설명을 불러오지 못했습니다"(guide_lines.yaml explain_failed)를 말하고 abort.
        no_description / no_artwork / tts_error 때 (취소·끊김·재생 명령 실패 때는 말하지 않음)."""
        self._say('explain_failed')
        return self._finish(goal_handle, 'abort', False, reason)

    def _wait(self, goal_handle, stop, busy):
        """busy()가 참인 동안 POLL_SEC마다 기다린다.
        취소·끊김이면 바로 (how, reason), 끝까지 기다렸으면 None."""
        while busy():
            if goal_handle.is_cancel_requested:
                return 'canceled', 'canceled'
            if stop.is_set():
                return 'abort', 'preempted'
            time.sleep(POLL_SEC)
        return None

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
                                        f'({self.timeout:.0f} s, PC3 확인) → 설명 안 함')
            else:
                self.get_logger().warn(f'작품 설명 없음: artwork_id={artwork_id} (PC3 success=false·빈 설명)')
            return self._fail(goal_handle, why)

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
                return self._fail(goal_handle, 'tts_error')
            path = out['path']

        # 3) 안내 음성이 끝나거나 멈출 때까지 기다린다 (새 안내가 오면 그것까지)
        hit = self._wait(goal_handle, stop, self._guide_playing)
        if hit:
            return self._finish(goal_handle, hit[0], False, hit[1])

        # 4) 재생 (끝나거나 취소·끊김까지 50 ms마다 확인)
        player = self._start_player(path)
        if player is None:
            return self._finish(goal_handle, 'abort', False, 'player_error')
        self.get_logger().info(f'설명 시작: {artwork_id}')

        hit = self._wait(goal_handle, stop, player.is_playing)
        if hit:
            player.stop()
            return self._finish(goal_handle, hit[0], False, hit[1])
        if player.poll() != 0:
            self.get_logger().error(
                f'재생 실패 (종료 코드 {player.poll()}): {player.stderr_text()}')
            return self._finish(goal_handle, 'abort', False, 'player_error')
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
