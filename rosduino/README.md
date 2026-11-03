# rosduino

[Русский](README_RU.md)

Metapackage for the computer side of [RosDuino](https://github.com/rosduino/rosduino_arduino). A dependency on `rosduino` pulls in both packages:

| Package | What it is |
|---|---|
| [`rosduino_agent`](../rosduino_agent/README.md) | One process per board. Opens a serial port or a TCP connection and exposes the board as a ROS 2 node. |
| [`rosduino_gen`](../rosduino_gen/README.md) | `rosduino-gen`: C++ headers for Arduino from ROS 2 `.msg` and `.srv` files. |

The repository overview is [../README.md](../README.md).

## Install

From the repository root, with ROS 2 sourced:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-up-to rosduino
source install/setup.bash
```

In another package:

```xml
<exec_depend>rosduino</exec_depend>
```

## License

[Apache-2.0](../LICENSE).
