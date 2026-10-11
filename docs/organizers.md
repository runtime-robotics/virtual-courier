# Памятка организаторам

## Образы

Образы собирает GitHub Actions (`.github/workflows/images.yml`) при пуше в `main` и публикует в GHCR:

- `ghcr.io/ssskiv/virtual-courier-scene:latest` — Webots + ROS 2 + сцена + судья;
- `ghcr.io/ssskiv/virtual-courier-solution:latest` — ROS 2 + Nav2 + slam_toolbox + OpenCV.

После первой публикации сделайте пакеты **публичными**: GitHub → Packages → пакет → Package settings → Change visibility.
Если репозиторий переезжает в организацию, замените `ghcr.io/ssskiv` в compose-файлах.

Локальная сборка: `docker compose build`.

## Закрытые сценарии

Храните их **вне репозитория** (он открытый), например в `~/vc-private/scenarios/final_1.yaml`.
Формат — как у `scene/scenarios/public_1.yaml`. Требования:

- `start.side` и `stations[].side` — стены на **границе** полигона, иначе генератор выдаст ошибку;
- старт и станции — в разных ячейках; лабиринт всегда связный (генератор строит идеальный лабиринт плюс `loops`);
- `obstacles` до ~16, иначе слишком тесно.

Предпросмотр (снимок обзорной камеры):

```bash
docker run --rm -e VC_SNAPSHOT=/out/final_1.jpg -e SCENARIO=/scenarios/final_1.yaml \
  -v ~/vc-private/scenarios:/scenarios:ro -v /tmp:/out --entrypoint bash \
  ghcr.io/ssskiv/virtual-courier-scene:latest -c \
  'Xvfb :99 -screen 0 1600x1000x24 & export DISPLAY=:99; sleep 1; source /opt/ros/humble/setup.bash;
   python3 /scene/generator/generate_world.py $SCENARIO --out /scene/webots/worlds --name current &&
   VC_WORLD_META=/scene/webots/worlds/current.json webots --batch --mode=realtime /scene/webots/worlds/current.wbt'
```

Перед финалом прогоните на каждом закрытом сценарии эталонное решение организаторов, чтобы убедиться,
что сценарий проходим за отведённое время. Эталон и закрытые сценарии лежат в **приватном** репозитории
организаторов `virtual-courier-reference` (в этот открытый репозиторий их не добавлять):

```bash
SCENARIO_DIR=../virtual-courier-reference/scenarios WEBOTS_GUI=false \
  tools/run_eval.sh reference ../virtual-courier-reference - final_1 1
```

## Зачётный прогон

```bash
xhost +local:docker
export SCENARIO_DIR=~/vc-private/scenarios
tools/run_eval.sh <team> https://github.com/<user>/virtual-courier.git final final_1 1
tools/run_eval.sh <team> https://github.com/<user>/virtual-courier.git final final_1 2
```

- Из репозитория команды берётся только `src/solution`. Сцена, compose и образы — организаторские.
- Решение запускается с `GUI=false`; окно Webots с табло показывается (`WEBOTS_GUI=true`), его и транслируем.
- Webots закрывается сам через 3 с после финиша, результат пишется в `results/<team>_<scenario>_<attempt>_<время>.json`
  вместе с хэшем коммита.
- Для GPU NVIDIA: `NVIDIA=true tools/run_eval.sh ...`.
- Чтобы прогнать всех быстрее без трансляции: `WEBOTS_GUI=false WEBOTS_MODE=fast tools/run_eval.sh ...`.

Таблица результатов (лучшая попытка на сценарий, сумма, бонус защиты):

```bash
tools/leaderboard.py --scenarios final_1,final_2 --defense defense.csv
```

`defense.csv` — строки `team,defense_percent` (0–20).

## Контроль запрещённого

Во время заезда можно проверить, что решение не подписано на топики судьи:

```bash
docker exec vc_scene bash -c "source /opt/ros/humble/setup.bash; ros2 topic info -v /referee/state"
```

Подписчиком должен быть только сам судья или ваши служебные инструменты. Дополнительно просмотрите код
на `referee`, `getPosition`, `supervisor`.

## Трансляция

Окно Webots: табло судьи в левом верхнем углу (команда, сценарий, время, баллы, последние события).
Камеру можно вращать мышью; *View → Follow Object* — слежение за роботом. Окно камеры робота
закрывается крестиком в его углу.
