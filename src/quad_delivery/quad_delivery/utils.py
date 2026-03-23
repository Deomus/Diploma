import math
from typing import Iterable, Optional, Tuple


Vector3 = Tuple[float, float, float]


def distance_3d(point_a: Iterable[float], point_b: Iterable[float]) -> float:
    ax, ay, az = list(point_a)[:3]
    bx, by, bz = list(point_b)[:3]
    return math.sqrt((bx - ax) ** 2 + (by - ay) ** 2 + (bz - az) ** 2)


def normalize_vector(vector: Iterable[float]) -> Vector3:
    x, y, z = list(vector)[:3]
    magnitude = math.sqrt(x * x + y * y + z * z)
    if magnitude == 0.0:
        return 0.0, 0.0, 0.0
    return x / magnitude, y / magnitude, z / magnitude


def compute_step(current: Iterable[float], target: Iterable[float], max_step: float) -> Vector3:
    current_list = list(current)[:3]
    target_list = list(target)[:3]
    delta = (
        target_list[0] - current_list[0],
        target_list[1] - current_list[1],
        target_list[2] - current_list[2],
    )
    direction = normalize_vector(delta)
    distance = distance_3d(current_list, target_list)
    step_length = min(max_step, distance)

    return (
        direction[0] * step_length,
        direction[1] * step_length,
        direction[2] * step_length,
    )


def discover_pose_service(node) -> Tuple[Optional[str], Optional[str]]:
    for service_name, service_types in node.get_service_names_and_types():
        if "simulation_interfaces/srv/SetEntityState" in service_types:
            return "simulation_interfaces", service_name

    for service_name, service_types in node.get_service_names_and_types():
        if "ros_gz_interfaces/srv/SetEntityPose" in service_types:
            return "ros_gz_interfaces", service_name

    return None, None


def create_pose_client(node, backend: str, service_name: str):
    if backend == "simulation_interfaces":
        from simulation_interfaces.srv import SetEntityState

        return node.create_client(SetEntityState, service_name)

    if backend == "ros_gz_interfaces":
        from ros_gz_interfaces.srv import SetEntityPose

        return node.create_client(SetEntityPose, service_name)

    raise ValueError(f"Unsupported backend: {backend}")


def build_pose_request(backend: str, entity_name: str, position: Iterable[float]):
    x, y, z = list(position)[:3]

    if backend == "simulation_interfaces":
        from simulation_interfaces.msg import EntityState
        from simulation_interfaces.srv import SetEntityState

        request = SetEntityState.Request()
        request.entity = entity_name
        request.state = EntityState()
        request.state.header.frame_id = "world"
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = float(z)
        request.state.pose.orientation.w = 1.0
        return request

    if backend == "ros_gz_interfaces":
        from ros_gz_interfaces.msg import Entity
        from ros_gz_interfaces.srv import SetEntityPose

        request = SetEntityPose.Request()
        request.entity = Entity()
        request.entity.name = entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = float(z)
        request.pose.orientation.w = 1.0
        return request

    raise ValueError(f"Unsupported backend: {backend}")


def pose_update_succeeded(backend: str, response) -> Tuple[bool, str]:
    if backend == "simulation_interfaces":
        success = response.result.result == response.result.RESULT_OK
        return success, response.result.error_message

    if backend == "ros_gz_interfaces":
        return response.success, ""

    raise ValueError(f"Unsupported backend: {backend}")
