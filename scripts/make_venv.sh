#!/usr/bin/env bash
# arti 전용 Python venv 생성/갱신 (여러 번 실행해도 안전)
#   bash scripts/make_venv.sh
# 위치는 ARTI_VENV로 바꿀 수 있다 (기본 ~/venvs/arti_venv). .envrc가 같은 변수를 읽는다.
set -euo pipefail

ARTI_VENV="${ARTI_VENV:-$HOME/venvs/arti_venv}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 다른 venv(예: rokey_venv)가 켜져 있어도 시스템 python으로 만든다
if [ ! -x "$ARTI_VENV/bin/python" ]; then
  echo "[make_venv] 생성: $ARTI_VENV"
  /usr/bin/python3 -m venv --system-site-packages "$ARTI_VENV"
fi

# colcon은 apt 버전을 venv의 python으로 실행한다.
# → 빌드된 노드 실행 파일이 venv python을 쓰고, pip로 colcon/pytest를 따로 깔 필요가 없다.
cat > "$ARTI_VENV/bin/colcon" <<'WRAP'
#!/usr/bin/env bash
exec "$(dirname "$0")/python" -m colcon "$@"
WRAP
chmod +x "$ARTI_VENV/bin/colcon"

"$ARTI_VENV/bin/python" -m pip install -q -r "$REPO_ROOT/requirements.txt"
echo "[make_venv] 완료. 저장소 폴더에서 'direnv allow' 후 다시 들어오면 적용됩니다."
