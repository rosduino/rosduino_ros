# rosduino

[English](README.md)

Метапакет компьютерной части [RosDuino](https://github.com/rosduino/rosduino_arduino). Зависимость от `rosduino` тянет оба пакета:

| Пакет | Что это |
|---|---|
| [`rosduino_agent`](../rosduino_agent/README_RU.md) | Один процесс на плату. Открывает serial-порт или TCP и показывает плату как узел ROS 2. |
| [`rosduino_gen`](../rosduino_gen/README_RU.md) | `rosduino-gen`: C++-заголовки для Arduino из файлов ROS 2 `.msg` и `.srv`. |

Обзор репозитория — [../README_RU.md](../README_RU.md).

## Установка

Из корня репозитория, с подключённым ROS 2:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-up-to rosduino
source install/setup.bash
```

В другом пакете:

```xml
<exec_depend>rosduino</exec_depend>
```

## Лицензия

[Apache-2.0](../LICENSE).
