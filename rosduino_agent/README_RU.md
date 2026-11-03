# rosduino_agent

[English](README.md)

**Узел ROS 2 для одной платы RosDuino.** Скетч [библиотеки Arduino](https://github.com/rosduino/rosduino_arduino) задаёт имя узла и объявляет топики и сервисы. Этот процесс открывает serial-порт, создаёт эти сущности и передаёт сообщения в обе стороны.

```bash
ros2 run rosduino_agent rosduino_agent --port /dev/ttyUSB0
ros2 topic pub --once /led std_msgs/msg/Bool "{data: true}"   # светодиод на плате загорелся
```

Обзор репозитория — [../README_RU.md](../README_RU.md). Заголовки для своих сообщений пишет [`rosduino_gen`](../rosduino_gen/README_RU.md).

## Что нужно

- Linux с ROS 2 **Lyrical** (основной дистрибутив) или **Jazzy**.
- Плата, прошитая RosDuino. Агент и библиотека говорят на 115200 бод, пока вы не поменяете скорость с обеих сторон.

На Jazzy плата не может *предоставлять* сервис. Топики и вызовы сервиса *с* платы работают.

На macOS и Windows запускайте агент на машине с Linux или в среде Linux, которая открывает serial-устройство. Если плата остаётся на другом компьютере, укажите агенту `tcp://host:port`.

## Сборка

Из корня репозитория:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select rosduino_agent
source install/setup.bash
```

Чтобы агент открывал порт без root, добавьте пользователя в группу `dialout` и войдите в систему заново:

```bash
sudo usermod -aG dialout "$USER"
```

## Запуск

Прошейте скетч и запустите агент на том же порту, который использовала IDE. Пример `Led` подписан на `/led`:

```bash
ros2 run rosduino_agent rosduino_agent --port /dev/ttyUSB0
```

```bash
ros2 node list          # /led_board
ros2 topic pub --once /led std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /led std_msgs/msg/Bool "{data: false}"
```

Перед заливкой нового скетча остановите агент, иначе порт занят.

| Плата в Linux | Обычный порт |
|---|---|
| Nano и другие USB-serial адаптеры | `/dev/ttyUSB0` |
| Uno, Micro, Mega, Due | `/dev/ttyACM0` |

`--port` — это serial-устройство или адрес TCP. Адрес TCP — исходящее соединение: агент повторяет попытку раз в секунду, пока кто-то не примет его и не перешлёт байты на плату. `--baud` относится к serial-устройству.

```bash
ros2 run rosduino_agent rosduino_agent --port /dev/ttyACM0 --baud 115200
ros2 run rosduino_agent rosduino_agent --port tcp://192.168.1.20:5760
```

Один процесс — один порт, одна плата и один узел ROS 2. Имя узла берётся из скетча (`ros.begin("led_board")`). Если такое имя уже есть в графе, агент берёт `led_board_2`, затем `led_board_3` и так далее. Непустой `--namespace` заменяет пространство имён из скетча.

Ctrl+C завершает сессию и убирает топики и сервисы узла. С `--keep-on-disconnect` при обрыве связи они остаются, пока агент не завершится.

### Несколько плат

[`launch/rosduino_agent.launch.py`](launch/rosduino_agent.launch.py) запускает по одному агенту на порт. `ports` — список через запятую. Без него launch-файл берёт `port`.

```bash
ros2 launch rosduino_agent rosduino_agent.launch.py port:=/dev/ttyUSB0
ros2 launch rosduino_agent rosduino_agent.launch.py ports:=/dev/ttyUSB0,/dev/ttyACM0 baud:=115200
```

## Параметры

Каждый флаг задаёт параметр ROS. Launch-файл передаёт эти параметры напрямую.

| Флаг | Параметр | По умолчанию | Эффект |
|---|---|---|---|
| `--port` | `port` | обязателен | Serial-устройство или `tcp://host:port` |
| `--baud` | `baud` | `115200` | Скорость serial. Должна совпадать со скетчем |
| `--namespace` | `namespace` | пусто | Заменяет пространство имён из скетча |
| `--keep-on-disconnect` | `keep_on_disconnect` | `false` | Оставить сущности ROS, когда связь пропала |
| `--service-timeout-ms` | `service_timeout_ms` | `5000` | Сколько может длиться вызов сервиса |
| `--no-diagnostics` | `diagnostics` | `true` | Параметр `false` отключает `/diagnostics` |
| `--log-level` | `log_level` | `info` | `debug`, `info`, `warn`, `error` или `fatal` |

```bash
ros2 run rosduino_agent rosduino_agent --ros-args -p port:=/dev/ttyUSB0 -p baud:=57600
```

## Диагностика

Если диагностику не отключали, агент публикует `diagnostic_msgs/msg/DiagnosticArray` в `/diagnostics` раз в секунду. Имя статуса — `rosduino: <node>`.

| Уровень | Когда |
|---|---|
| OK | Плата на связи, и каждая объявленная сущность создана |
| WARN | Связи с платой нет, или сущность не зарегистрировалась |
| ERROR | Порт так и не открылся |

`ros2 topic echo /diagnostics --field status` показывает порт, состояние сессии, версию протокола, профиль платы, число сущностей, число кадров и счётчики ошибок.

## Проверка сборки

```bash
colcon test --packages-select rosduino_agent
colcon test-result
```

## Лицензия

[Apache-2.0](../LICENSE).
