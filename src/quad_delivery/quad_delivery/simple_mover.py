from enum import Enum
from typing import List, Optional

import rclpy
from rclpy.node import Node

from quad_delivery.utils import (
    build_pose_request,
    compute_step,
    create_pose_client,
    discover_pose_service,
    distance_3d,
    pose_update_succeeded,
)


class MissionState(str, Enum):
    TAKEOFF = "TAKEOFF"
    CRUISE = "CRUISE"
    DONE = "DONE"


class SimpleMover(Node):
    """Move the drone from A to B with a constant speed."""

    def __init__(self):
        super().__init__("simple_mover")

        self.entity_name = "delivery_drone"
        self.speed = 5.0
        self.dt = 0.1
        self.tolerance = 0.2

        self.current_position = [0.0, 0.0, 0.5]
        self.takeoff_target = [0.0, 0.0, 10.0]
        self.cruise_target = [50.0, 0.0, 10.0]
        self.state = MissionState.TAKEOFF

        self.service_backend: Optional[str] = None
        self.service_name: Optional[str] = None
        self.cli = None
        self.pending_future = None
        self.status_counter = 0

        self.connect_timer = self.create_timer(1.0, self._try_connect_service)
        self.motion_timer = None
        self.get_logger().info("Mission state: TAKEOFF")

    def _try_connect_service(self) -> None:
        if self.cli is not None:
            return

        backend, service_name = discover_pose_service(self)
        if backend is None or service_name is None:
            self.get_logger().info("Waiting for Gazebo pose service...")
            return

        client = create_pose_client(self, backend, service_name)
        if not client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info(f"Waiting for service {service_name}...")
            return

        self.service_backend = backend
        self.service_name = service_name
        self.cli = client
        self.connect_timer.cancel()
        self.motion_timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(f"Connected to {service_name} using {backend}.")
        self._send_pose_update()

    def _on_timer(self) -> None:
        if self.pending_future is not None:
            if not self.pending_future.done():
                return
            self._handle_service_response()
            self.pending_future = None

        target = self._get_active_target()
        if target is None:
            return

        distance = distance_3d(self.current_position, target)

        if distance < self.tolerance:
            self.current_position = list(target)
            self._send_pose_update()
            self._advance_state()
            return

        step = compute_step(self.current_position, target, self.speed * self.dt)
        self.current_position[0] += step[0]
        self.current_position[1] += step[1]
        self.current_position[2] += step[2]
        self._send_pose_update()

        self.status_counter += 1
        if self.status_counter % 10 == 0:
            self.get_logger().info(
                f"State={self.state.value} "
                f"position=({self.current_position[0]:.2f}, "
                f"{self.current_position[1]:.2f}, "
                f"{self.current_position[2]:.2f})"
            )

    def _get_active_target(self) -> Optional[List[float]]:
        if self.state == MissionState.TAKEOFF:
            return self.takeoff_target
        if self.state == MissionState.CRUISE:
            return self.cruise_target
        return None

    def _advance_state(self) -> None:
        if self.state == MissionState.TAKEOFF:
            self.state = MissionState.CRUISE
            self.get_logger().info("State changed: CRUISE")
            return

        if self.state == MissionState.CRUISE:
            self.state = MissionState.DONE
            self.get_logger().info("State changed: DONE")
            self.get_logger().info("Drone reached target and stopped.")

    def _handle_service_response(self) -> None:
        try:
            response = self.pending_future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Pose update failed: {exc}")
            return

        success, error_message = pose_update_succeeded(self.service_backend, response)
        if not success:
            details = error_message or "unknown service error"
            self.get_logger().warning(f"Gazebo rejected pose update: {details}")

    def _send_pose_update(self) -> None:
        if self.cli is None:
            return

        request = build_pose_request(self.service_backend, self.entity_name, self.current_position)
        self.pending_future = self.cli.call_async(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
