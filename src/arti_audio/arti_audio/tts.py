"""문장 → mp3 합성 (edge-tts) + 파일 캐시."""

import asyncio
import hashlib
import os
import threading

import edge_tts 


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
