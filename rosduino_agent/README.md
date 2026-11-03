# rosduino_agent

[Русский](README_RU.md)

**The ROS 2 node for one RosDuino board.** The sketch on the [Arduino library](https://github.com/rosduino/rosduino_arduino) names the node and declares topics and services. This process opens the serial port, creates those entities, and forwards messages both ways.

```bash
ros2 run rosduino_agent rosduino_agent --port /dev/ttyUSB0
ros2 topic pub --once /led std_msgs/msg/Bool "{data: true}"   # the onboard LED turns on
```

The repository overview is [../README.md](../README.md). Headers for your own messages come from [`rosduino_gen`](../rosduino_gen/README.md).

## Requirements

- Linux with ROS 2 **Lyrical** (recommended) or **Jazzy**.
- A board flashed with RosDuino. The agent and the library talk at 115200 baud unless you change both sides.

On Jazzy the board cannot *offer* a service. Topics, and service calls *from* the board, still work.

On macOS and Windows, run the agent on a Linux machine or in a Linux environment that can open the serial device. If the board stays on another computer, point the agent at `tcp://host:port`.

## Build

From the repository root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select rosduino_agent
source install/setup.bash
```

Add your user to the `dialout` group so the agent can open the port without root, then log in again:

```bash
sudo usermod -aG dialout "$USER"
```

## Run

Flash a sketch, then start the agent on the same port the IDE used. The `Led` example subscribes to `/led`:

```bash
ros2 run rosduino_agent rosduino_agent --port /dev/ttyUSB0
```

```bash
ros2 node list          # /led_board
ros2 topic pub --once /led std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /led std_msgs/msg/Bool "{data: false}"
```

Stop the agent before uploading a new sketch, or the port stays busy.

| Board on Linux | Typical port |
|---|---|
| Nano, and other USB-serial adapters | `/dev/ttyUSB0` |
| Uno, Micro, Mega, Due | `/dev/ttyACM0` |

`--port` is a serial device or a TCP address. A TCP address is an outbound connection: the agent retries every second until something accepts it and forwards bytes to the board. `--baud` applies to a serial device.

```bash
ros2 run rosduino_agent rosduino_agent --port /dev/ttyACM0 --baud 115200
ros2 run rosduino_agent rosduino_agent --port tcp://192.168.1.20:5760
```

One process is one port, one board, and one ROS 2 node. The node name comes from the sketch (`ros.begin("led_board")`). If that name is already on the graph, the agent uses `led_board_2`, then `led_board_3`, and so on. A non-empty `--namespace` replaces the namespace from the sketch.

Ctrl+C ends the session and removes the node's topics and services. With `--keep-on-disconnect`, a dropped link leaves them in place until the agent exits.

### Several boards

[`launch/rosduino_agent.launch.py`](launch/rosduino_agent.launch.py) starts one agent per port. `ports` is a comma-separated list. Without it, the launch file uses `port`.

```bash
ros2 launch rosduino_agent rosduino_agent.launch.py port:=/dev/ttyUSB0
ros2 launch rosduino_agent rosduino_agent.launch.py ports:=/dev/ttyUSB0,/dev/ttyACM0 baud:=115200
```

## Options

Each flag sets a ROS parameter. A launch file passes those parameters directly.

| Flag | Parameter | Default | Effect |
|---|---|---|---|
| `--port` | `port` | required | Serial device, or `tcp://host:port` |
| `--baud` | `baud` | `115200` | Serial baud. Must match the sketch |
| `--namespace` | `namespace` | empty | Replaces the namespace from the sketch |
| `--keep-on-disconnect` | `keep_on_disconnect` | `false` | Keep ROS entities when the link drops |
| `--service-timeout-ms` | `service_timeout_ms` | `5000` | How long a service call may take |
| `--no-diagnostics` | `diagnostics` | `true` | Set the parameter to `false` to skip `/diagnostics` |
| `--log-level` | `log_level` | `info` | `debug`, `info`, `warn`, `error`, or `fatal` |

```bash
ros2 run rosduino_agent rosduino_agent --ros-args -p port:=/dev/ttyUSB0 -p baud:=57600
```

## Diagnostics

With diagnostics left on, the agent publishes `diagnostic_msgs/msg/DiagnosticArray` on `/diagnostics` at 1 Hz. The status name is `rosduino: <node>`.

| Level | When |
|---|---|
| OK | The board is connected and every declared entity exists |
| WARN | The link to the board is down, or an entity failed to register |
| ERROR | The port never opened |

`ros2 topic echo /diagnostics --field status` shows the port, session state, protocol version, board profile, entity counts, frame counts, and error counters.

## Check the build

```bash
colcon test --packages-select rosduino_agent
colcon test-result
```

## License

[Apache-2.0](../LICENSE).
