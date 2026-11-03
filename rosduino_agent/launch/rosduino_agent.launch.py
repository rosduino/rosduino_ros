# SPDX-License-Identifier: Apache-2.0
"""Launch one rosduino_agent per serial/TCP port."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_str(value):
    return str(value).lower() in ('1', 'true', 'yes')


def _launch_nodes(context, *_args, **_kwargs):
    ports_raw = LaunchConfiguration('ports').perform(context).strip()
    port = LaunchConfiguration('port').perform(context).strip()
    baud = LaunchConfiguration('baud').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    keep = _bool_str(LaunchConfiguration('keep_on_disconnect').perform(context))
    timeout = LaunchConfiguration('service_timeout_ms').perform(context)
    diagnostics = _bool_str(LaunchConfiguration('diagnostics').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)

    ports = [p.strip() for p in ports_raw.split(',') if p.strip()]
    if not ports:
        ports = [port]

    nodes = []
    for index, device in enumerate(ports):
        name = 'rosduino_agent' if index == 0 else 'rosduino_agent_{}'.format(index + 1)
        nodes.append(
            Node(
                package='rosduino_agent',
                executable='rosduino_agent',
                name=name,
                output='screen',
                parameters=[{
                    'port': device,
                    'baud': int(baud),
                    'namespace': namespace,
                    'keep_on_disconnect': keep,
                    'service_timeout_ms': int(timeout),
                    'diagnostics': diagnostics,
                    'log_level': log_level,
                }],
            )
        )
    return nodes


def generate_launch_description():
    """Return the launch description for one or more agent processes."""
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('ports', default_value=''),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('keep_on_disconnect', default_value='false'),
        DeclareLaunchArgument('service_timeout_ms', default_value='5000'),
        DeclareLaunchArgument('diagnostics', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_launch_nodes),
    ])
