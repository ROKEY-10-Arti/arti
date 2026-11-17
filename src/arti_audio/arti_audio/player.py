"""음성 파일 재생: 외부 명령(pw-play 등) 프로세스 래퍼."""

import subprocess

STOP_TIMEOUT_SEC = 0.2   # terminate 후 이 시간 안에 안 끝나면 kill (노드의 확인 주기 0.05 + 0.2 < 0.3초)


class Player:
    """음성 파일 하나를 외부 명령으로 재생한다. cmd 뒤에 파일 경로가 붙는다."""

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
