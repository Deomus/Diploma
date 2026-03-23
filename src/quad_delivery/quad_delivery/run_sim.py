import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def main(args=None) -> int:
    launch_file = os.path.join(
        get_package_share_directory("quad_delivery"),
        "launch",
        "sim.launch.py",
    )

    launch_description = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
            )
        ]
    )

    launch_service = LaunchService(argv=args if args is not None else sys.argv[1:])
    launch_service.include_launch_description(launch_description)
    return launch_service.run()


if __name__ == "__main__":
    raise SystemExit(main())
