"""
작품 설명·안내 음성 재생 노드 (T35).

받는 것
  explain_artwork (arti_interfaces/action/ExplainArtwork) : 작품 설명 재생
  guide_message   (std_msgs/String)                       : 안내 키 (guide_lines.yaml의 guides)

흐름
  artwork_id → 작품 정보 출처(지금 샘플 YAML, 나중에 DB)에서 description
             → 읽을 문장 → edge-tts로 mp3 합성(캐시) → 재생 명령(pw-play) → 스피커

규칙
  - 안내 음성이 오면 진행 중인 설명은 끊는다 (result.reason = preempted).
  - 안내 음성이 나오는 동안 새 설명 goal은 안내가 끝나거나 멈출 때까지 기다렸다가 재생한다.
  - 새 설명 goal이 오면 이전 설명은 끊는다 (preempted). 한 번에 설명 하나만.
  - 취소하면 0.3초 안에 멈춘다 (합성·재생 중 모두 50 ms마다 확인).
  - 시작할 때 안내 문구와 (가능하면) 작품 설명을 미리 합성해 둔다 → 첫 재생 지연을 줄인다.
"""

import asyncio
import hashlib
import os
import shlex
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
from arti_interfaces.action import ExplainArtwork
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import yaml

POLL_SEC = 0.05          # 취소·끊김 확인 주기
FEEDBACK_SEC = 0.5       # 진행률 발행 주기
STOP_TIMEOUT_SEC = 0.2   # terminate 후 이 시간 안에 안 끝나면 kill (확인 주기 0.05 + 0.2 < 0.3초)
MP3_BITRATE_BPS = 48000  # edge-tts 출력 비트레이트 (진행률 추정용)


# ---------- 읽을 문장 준비 (안내 문구 YAML, 작품 정보) ----------

def _load_yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


class GuideLines:
    """guide_lines.yaml: TTS 설정, 안내 키 → 문장."""

    def __init__(self, path):
        data = _load_yaml(path)
        tts = data.get('tts') or {}
        self.voice = tts.get('voice', 'ko-KR-SunHiNeural')
        self.rate = tts.get('rate', '+0%')
        self.guides = {str(k): str(v).strip() for k, v in (data.get('guides') or {}).items()}

    def artwork_text(self, artwork):
        """작품 정보에서 읽을 문장 = description 그대로 (제목·작가는 읽지 않는다)."""
        text = str(artwork.get('description') or '').strip()
        if not text:
            raise ValueError('작품 description이 비어 있음')
        return text


class YamlArtworkSource:
    """작품 정보 출처 (지금은 artworks_sample.yaml). DB가 정해지면 get·all_artworks가 같은 클래스로 교체."""

    def __init__(self, path):
        rows = _load_yaml(path).get('artworks') or []
        self._artworks = {str(r['artwork_id']): r for r in rows if r.get('artwork_id')}

    def get(self, artwork_id):
        return self._artworks.get(artwork_id)

    def all_artworks(self):
        return dict(self._artworks)


# ---------- 문장 → mp3 합성 (edge-tts) + 파일 캐시 ----------

class TtsCache:
    """같은 (목소리, 빠르기, 문장)은 한 번만 합성. 파일 이름이 이 셋의 해시라 문장이 바뀌면 새로 만든다.
    합성에는 인터넷이 필요하다 (작품·안내 문장만 보낸다. 관람객 정보는 보내지 않는다)."""

    def __init__(self, cache_dir, voice, rate, timeout_sec=10):
        self.cache_dir = os.path.expanduser(cache_dir)
        self.voice = voice
        self.rate = rate
        self.timeout_sec = timeout_sec
        self._locks = {}
        self._locks_guard = threading.Lock()
        os.makedirs(self.cache_dir, exist_ok=True)

    def path_for(self, text):
        key = f'{self.voice}|{self.rate}|{text}'.encode('utf-8')
        return os.path.join(self.cache_dir, hashlib.sha1(key).hexdigest()[:20] + '.mp3')

    def cached(self, text):
        """이미 합성된 파일 경로, 없으면 None."""
        path = self.path_for(text)
        return path if os.path.isfile(path) else None

    def synthesize(self, text):
        """합성해서 파일 경로를 돌려준다 (이미 있으면 바로). 실패하면 예외."""
        path = self.path_for(text)
        with self._locks_guard:
            lock = self._locks.setdefault(path, threading.Lock())
        with lock:  # 같은 문장을 두 스레드가 동시에 합성하지 않게
            if os.path.isfile(path):
                return path
            import edge_tts  # arti venv에 있음 (requirements.txt)
            tmp = path + '.part'
            comm = edge_tts.Communicate(
                text, self.voice, rate=self.rate,
                connect_timeout=self.timeout_sec, receive_timeout=self.timeout_sec * 3)
            try:
                asyncio.run(comm.save(tmp))
                if os.path.getsize(tmp) == 0:
                    raise RuntimeError('합성 결과가 빈 파일')
                os.replace(tmp, path)  # 다 만든 뒤에 이름을 바꿔서 반쪽 파일을 재생하지 않게
            finally:
                if os.path.exists(tmp):
                    os.remove(tmp)
        return path


# ---------- 재생 ----------

class Player:
    """음성 파일 하나를 외부 명령으로 재생하는 프로세스 래퍼."""

    def __init__(self, cmd, path):
        self.proc = subprocess.Popen(
            cmd + [path], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

    def poll(self):
        return self.proc.poll()

    def is_playing(self):
        return self.proc.poll() is None

    def stop(self):
        if self.proc.poll() is not None:
            return
        self.proc.terminate()
        try:
            self.proc.wait(timeout=STOP_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()

    def stderr_text(self):
        try:
            return self.proc.stderr.read().decode(errors='replace').strip()
        except (OSError, ValueError):
            return ''


class GuideAudioNode(Node):

    def __init__(self):
        super().__init__('guide_audio_node')
        config = os.path.join(get_package_share_directory('arti_audio'), 'config')
        self.declare_parameter('guide_lines_file', '')
        self.declare_parameter('artworks_file', '')
        self.declare_parameter('tts_cache_dir', '~/.cache/arti_audio/tts')
        self.declare_parameter('tts_timeout_sec', 10)
        self.declare_parameter('prefetch', True)
        self.declare_parameter('player_cmd', 'pw-play')

        p = self.get_parameter
        self.lines = GuideLines(p('guide_lines_file').value
                                or os.path.join(config, 'guide_lines.yaml'))
        self.source = YamlArtworkSource(
            p('artworks_file').value or os.path.join(config, 'artworks_sample.yaml'))
        self.tts = TtsCache(p('tts_cache_dir').value, self.lines.voice, self.lines.rate,
                            int(p('tts_timeout_sec').value))
        self.player_cmd = shlex.split(p('player_cmd').value)

        self._lock = threading.Lock()
        self._guide = None          # 지금 나오는 안내 음성 Player
        self._explain_stop = None   # 지금 설명 중인 goal을 끊는 Event

        cb = ReentrantCallbackGroup()
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
            f'준비됨: 목소리 {self.lines.voice}, '
            f'재생 명령 "{" ".join(self.player_cmd)}", 캐시 {self.tts.cache_dir}')
        if p('prefetch').value:
            threading.Thread(target=self._prefetch, daemon=True).start()

    # ---------- 미리 합성 ----------

    def _prefetch(self):
        texts = [('guide', k, t) for k, t in self.lines.guides.items()]
        try:
            for aid, artwork in self.source.all_artworks().items():
                texts.append(('artwork', aid, self.lines.artwork_text(artwork)))
        except Exception as e:  # 작품 정보가 이상해도 노드는 계속
            self.get_logger().warn(f'작품 정보 읽기 실패 (미리 합성 건너뜀): {e}')
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

    def _interrupted(self, goal_handle, stop):
        """취소·끊김이면 (how, reason), 아니면 None."""
        if goal_handle.is_cancel_requested:
            return 'canceled', 'canceled'
        if stop.is_set():
            return 'abort', 'preempted'
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
        # 1) 작품 정보 → 읽을 문장
        try:
            artwork = self.source.get(artwork_id)
            text = self.lines.artwork_text(artwork) if artwork else None
        except Exception as e:
            self.get_logger().error(f'작품 정보 읽기 실패 ({artwork_id}): {e}')
            text = None
        if not text:
            self.get_logger().warn(f'작품 정보 없음: artwork_id="{artwork_id}"')
            return self._finish(goal_handle, 'abort', False, 'no_artwork')

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
            while worker.is_alive():
                hit = self._interrupted(goal_handle, stop)
                if hit:  # 합성은 뒤에서 마저 끝나 캐시에 남는다
                    return self._finish(goal_handle, hit[0], False, hit[1])
                time.sleep(POLL_SEC)
            if 'error' in out:
                self.get_logger().error(
                    f'음성 합성 실패 ({artwork_id}, 인터넷 확인): {out["error"]}')
                return self._finish(goal_handle, 'abort', False, 'tts_error')
            path = out['path']

        # 3) 안내 음성이 끝나거나 멈출 때까지 기다린다 (새 안내가 오면 그것까지, 기다리는 중에도 취소 확인)
        while self._guide_playing():
            hit = self._interrupted(goal_handle, stop)
            if hit:
                return self._finish(goal_handle, hit[0], False, hit[1])
            time.sleep(POLL_SEC)

        # 4) 재생
        player = self._start_player(path)
        if player is None:
            return self._finish(goal_handle, 'abort', False, 'player_error')
        self.get_logger().info(f'설명 시작: {artwork_id}')

        duration = max(os.path.getsize(path) * 8.0 / MP3_BITRATE_BPS, 0.1)  # 대략
        t0 = time.monotonic()
        last_fb = 0.0
        feedback = ExplainArtwork.Feedback()
        while player.is_playing():
            hit = self._interrupted(goal_handle, stop)
            if hit:
                player.stop()
                return self._finish(goal_handle, hit[0], False, hit[1])
            now = time.monotonic()
            if now - last_fb >= FEEDBACK_SEC:
                feedback.progress = float(min((now - t0) / duration, 1.0))
                goal_handle.publish_feedback(feedback)
                last_fb = now
            time.sleep(POLL_SEC)

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
