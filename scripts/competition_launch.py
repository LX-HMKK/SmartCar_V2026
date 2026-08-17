#!/usr/bin/env python3
"""Run the fixed competition launch without ROS CLI extension discovery."""
from __future__ import annotations

import signal
import sys
from collections import OrderedDict


def parse_launch_arguments(arguments: list[str]) -> list[tuple[str, str]]:
    """Parse the vetted ``name:=value`` launch arguments used by competition."""

    parsed: OrderedDict[str, str] = OrderedDict()
    for argument in arguments:
        name, separator, value = argument.partition(":=")
        if not separator or not name or not value:
            raise ValueError(f"expected name:=value launch argument, got {argument!r}")
        # Match the standard launch parser: passing the same name twice lets
        # the final value replace the earlier one.
        parsed[name] = value
    return list(parsed.items())


def install_empty_node_extension_cache() -> None:
    """Avoid repeatedly scanning entry points when this image has no extensions.

    ROS 2 Humble constructs a fresh ``Node`` extension registry for every
    launch action.  On the RDK image the registry is empty.  Probe it once;
    if extensions are added on a future image, fall back to the upstream
    behavior for every later node instead of sharing extension instances.
    """

    import launch_ros.actions.node as node_module

    entry_points = node_module.importlib_metadata.entry_points()
    if hasattr(entry_points, "select"):
        node_extensions = entry_points.select(group="launch_ros.node_action")
    else:
        node_extensions = entry_points.get("launch_ros.node_action", [])
    if not node_extensions:
        node_module.get_extensions = lambda _logger: {}


def install_graceful_shutdown_signal(launch_service) -> None:
    """Use a dedicated signal for orderly shutdown of the background launcher.

    ``LaunchService`` treats SIGTERM as an immediate task cancellation, which
    can orphan child processes. The competition launcher runs under ``nohup``
    and may inherit ignored SIGINT, so cleanup asks this handler to emit the
    normal launch shutdown event instead.
    """

    def handle_shutdown(_signum, _frame) -> None:
        launch_service.shutdown(force_sync=True)

    signal.signal(signal.SIGUSR1, handle_shutdown)


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    try:
        launch_arguments = parse_launch_arguments(arguments)
    except ValueError as error:
        print(f"invalid competition launch argument: {error}", file=sys.stderr)
        return 2

    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    install_empty_node_extension_cache()
    launch_file = (
        get_package_share_directory("smartcar_bringup")
        + "/launch/smartcar_system.launch.py"
    )
    launch_service = LaunchService(argv=arguments, noninteractive=True)
    launch_description = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments=launch_arguments,
        ),
    ])
    launch_service.include_launch_description(launch_description)
    install_graceful_shutdown_signal(launch_service)
    return launch_service.run()


if __name__ == "__main__":
    raise SystemExit(main())
