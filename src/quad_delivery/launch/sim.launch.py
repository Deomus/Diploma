from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_path = PathJoinSubstitution([FindPackageShare("quad_delivery"), "worlds", "empty_world.sdf"])
    model_path = PathJoinSubstitution([FindPackageShare("quad_delivery"), "models", "delivery_drone", "model.sdf"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-s -r ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_spawn_model.launch.py"])
        ),
        launch_arguments={
            "file": model_path,
            "entity_name": "delivery_drone",
            "x": "0.0",
            "y": "0.0",
            "z": "0.5",
        }.items(),
    )

    simple_mover = Node(
        package="quad_delivery",
        executable="simple_mover",
        name="simple_mover",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            gazebo,
            TimerAction(period=2.0, actions=[spawn_drone]),
            TimerAction(period=4.0, actions=[simple_mover]),
        ]
    )
