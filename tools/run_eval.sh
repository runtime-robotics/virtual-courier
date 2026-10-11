#!/bin/bash
# Официальный прогон решения команды (для организаторов).
#
#   tools/run_eval.sh <team> <git-url> [ref] [scenario] [attempt]
#
#   team      — короткое имя команды (латиница, без пробелов), попадёт в results/*.json
#   git-url   — ссылка на форк команды (https://github.com/<user>/virtual-courier.git)
#               или локальный каталог с src/solution (тогда ref не используется)
#   ref       — тег/ветка/коммит (по умолчанию final)
#   scenario  — имя сценария без .yaml (по умолчанию public_1)
#   attempt   — номер попытки (по умолчанию 1)
#
# Переменные окружения:
#   SCENARIO_DIR  каталог со сценариями (для финала — закрытые сценарии), по умолчанию scene/scenarios
#   WEBOTS_GUI    true — показывать окно Webots (трансляция), false — headless. По умолчанию true
#   WEBOTS_MODE   realtime | fast. По умолчанию realtime
#
# Из репозитория команды берётся ТОЛЬКО каталог src/solution — остальные файлы
# (сцена, compose, образы) всегда организаторские.
set -euo pipefail

TEAM=${1:?team}
URL=${2:?git url}
REF=${3:-final}
SCENARIO=${4:-public_1}
ATTEMPT=${5:-1}

ROOT=$(cd "$(dirname "$0")/.." && pwd)
WORK="$ROOT/eval_work/$TEAM"
rm -rf "$WORK"
mkdir -p "$WORK" "$ROOT/results"

if [ -d "$URL" ] && [ ! -d "$URL/.git" ]; then
    echo ">>> [$TEAM] локальный каталог $URL"
    cp -r "$URL" "$WORK/repo"
    COMMIT="local"
else
    echo ">>> [$TEAM] клонирование $URL @ $REF"
    git clone --quiet "$URL" "$WORK/repo"
    git -C "$WORK/repo" checkout --quiet "$REF"
    COMMIT=$(git -C "$WORK/repo" rev-parse --short HEAD)
fi
if [ ! -f "$WORK/repo/src/solution/launch/start.launch.py" ]; then
    echo "!!! в репозитории нет src/solution/launch/start.launch.py" >&2
    exit 2
fi
cp -r "$WORK/repo/src/solution" "$WORK/solution"
rm -rf "$WORK/solution/build" "$WORK/solution/install" "$WORK/solution/log"

echo ">>> [$TEAM] коммит $COMMIT, сценарий $SCENARIO, попытка $ATTEMPT"
cd "$ROOT"
export SOLUTION_DIR="$WORK/solution"
export SCENARIO VC_ATTEMPT="$ATTEMPT" VC_TEAM="$TEAM"
export SCENARIO_DIR=${SCENARIO_DIR:-$ROOT/scene/scenarios}
export WEBOTS_GUI=${WEBOTS_GUI:-true}
export COMPOSE_PROJECT_NAME="vc_eval"

COMPOSE=(docker compose -f docker-compose.yaml -f docker-compose.eval.yaml)
[ "${NVIDIA:-false}" = "true" ] && COMPOSE+=(-f docker-compose.nvidia.yaml)

"${COMPOSE[@]}" down --remove-orphans >/dev/null 2>&1 || true
set +e
"${COMPOSE[@]}" up --no-build --abort-on-container-exit --exit-code-from scene
CODE=$?
set -e
"${COMPOSE[@]}" down --remove-orphans >/dev/null 2>&1 || true

LATEST=$(ls -t "$ROOT"/results/"${TEAM}_${SCENARIO}_${ATTEMPT}"_*.json 2>/dev/null | head -1 || true)
if [ -n "$LATEST" ]; then
    python3 - "$LATEST" "$COMMIT" <<'EOF'
import json, sys
path, commit = sys.argv[1], sys.argv[2]
r = json.load(open(path, encoding='utf-8'))
r['commit'] = commit
json.dump(r, open(path, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
print(f"\n=== {r['team']} | {r['scenario']} | попытка {r['attempt']} | коммит {commit}")
print(f"=== ИТОГ: {r['score']} баллов, {r['finish_reason']}, время {r['elapsed']} с, касаний {r['collisions']}")
for e in r['log']:
    print(f"   {e['t']:7.1f} с  {e['points']:+4d}  {e['text']}")
print(f"=== {path}")
EOF
else
    echo "!!! результат не найден (код выхода сцены $CODE)" >&2
    exit 1
fi
