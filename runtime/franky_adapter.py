from __future__ import annotations

import time
from typing import Any, Dict, Optional

from robot_adapter import RobotAdapter


class FrankyAdapter(RobotAdapter):
    """Adapter for Franka robots using the franky library.

    Requires:
    - franky-control Python package
    - Real-time kernel with realtime permissions
    - Franka Control Interface (FCI) enabled via Web UI
    """

    DEFAULT_GRIPPER_SPEED = 0.02  # m/s
    DEFAULT_GRIPPER_FORCE = 10.0  # N
    DEFAULT_GRASP_WIDTH = 0.0  # fully closed

    def __init__(
        self,
        *,
        host: str = "",
        port: int | None = None,
        driver: str = "franky_ip",
        extras: Dict[str, Any] | None = None,
    ) -> None:
        self.host = host or "172.16.0.2"
        self.port = int(port) if port is not None else None
        self.driver = str(driver or "franky_ip")
        self.extras = extras if isinstance(extras, dict) else {}
        self._robot: Any = None
        self._gripper: Any = None
        self._connected = False
        self._busy = False
        self._faulted = False
        self._gripper_closed: bool | None = None
        self._gripper_position: float | None = None
        self._tool_pose: list[float] = []
        self._joint_state: list[float] = []

    def connect(self) -> Dict[str, Any]:
        from franky import Robot,Gripper

        self._robot = Robot(self.host)
        self._gripper = Gripper(self.host)  # Gripper shares the same IP
        self._connected = True
        return {
            "ok": True,
            "driver": self.driver,
            "host": self.host,
            "port": self.port,
        }

    def close(self) -> None:
        if self._robot is not None:
            self._robot = None
        self._gripper = None
        self._connected = False

    def recover_from_errors(self) -> Dict[str, Any]:
        """Attempt to recover from robot errors. Returns status dict."""
        if self._robot is None or not self._connected:
            return {"ok": False, "error": "Robot not connected"}
        try:
            self._robot.recover_from_errors()
            self._faulted = False
            return {"ok": True, "recovered": True}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def get_runtime_state(self) -> Dict[str, Any]:
        state = {
            "source": self.driver,
            "connected": self._connected,
            "busy": self._busy,
            "faulted": self._faulted,
            "tool_pose": list(self._tool_pose),
            "joint_state": list(self._joint_state),
            "gripper_closed": self._gripper_closed,
            "gripper_position": self._gripper_position,
        }

        if self._robot is not None and self._connected:
            try:
                cartesian_state = self._robot.current_cartesian_state
                ee_pose = cartesian_state.pose.end_effector_pose
                self._tool_pose = [
                    ee_pose.translation.x,
                    ee_pose.translation.y,
                    ee_pose.translation.z,
                    ee_pose.rotation.x,
                    ee_pose.rotation.y,
                    ee_pose.rotation.z,
                    ee_pose.rotation.w,
                ]

                joint_state = self._robot.current_joint_state
                self._joint_state = list(joint_state.position)
            except Exception:
                pass

        return state

    def execute_action(self, action: Dict[str, Any], execute_motion: bool = False) -> Dict[str, Any]:
        action = action if isinstance(action, dict) else {}
        action_type = str(action.get("type", "")).strip().lower()

        if action_type == "wait":
            seconds = float(action.get("seconds", action.get("duration_s", 0.5)))
            time.sleep(max(0.0, min(seconds, 30.0)))
            return {
                "ok": True,
                "driver": self.driver,
                "action_type": action_type,
                "executed": True,
                "dry_run": False,
            }

        if not execute_motion:
            return {
                "ok": True,
                "driver": self.driver,
                "action_type": action_type,
                "executed": False,
                "dry_run": True,
            }

        if self._robot is None or not self._connected:
            return {
                "ok": False,
                "driver": self.driver,
                "action_type": action_type,
                "executed": False,
                "error": "Robot not connected",
            }

        self._busy = True
        try:
            if action_type == "movej":
                self._execute_movej(action)
            elif action_type == "movel":
                self._execute_movel(action)
            elif action_type == "open_gripper":
                self._execute_open_gripper(action)
            elif action_type == "close_gripper":
                self._execute_close_gripper(action)
            else:
                raise RuntimeError(f"Unsupported action type for {self.driver}: {action_type}")

            return {
                "ok": True,
                "driver": self.driver,
                "action_type": action_type,
                "executed": True,
                "dry_run": False,
            }
        except Exception as e:
            self._faulted = True
            return {
                "ok": False,
                "driver": self.driver,
                "action_type": action_type,
                "executed": False,
                "error": str(e),
            }
        finally:
            self._busy = False

    def _execute_movej(self, action: Dict[str, Any]) -> None:
        from franky import JointMotion

        joints = action.get("joints", [])
        if not isinstance(joints, list) or len(joints) != 7:
            raise ValueError("movej requires 7 joint values")

        self._robot.relative_dynamics_factor = 0.05
        motion = JointMotion([float(v) for v in joints])
        self._robot.move(motion)
        self._joint_state = [float(v) for v in joints]

    def _execute_movel(self, action: Dict[str, Any]) -> None:
        from franky import Affine, CartesianMotion

        pose = action.get("pose", [])
        if not isinstance(pose, list) or len(pose) < 6:
            raise ValueError("movel requires at least 6 pose values (x, y, z, rx, ry, rz)")

        self._robot.relative_dynamics_factor = 0.05

        # pose format: [x, y, z, rx, ry, rz] (position in mm + Euler angles in degrees)
        # or [x, y, z, rx, ry, rz, rw] (position in mm + quaternion)

        print(f"[DEBUG] Moving to pose: {pose[:3]} mm + {pose[3:]} angles")
        if len(pose) == 7:
            # quaternion format: position in mm -> m
            pos_m = [v / 1000.0 for v in pose[:3]]
            self._robot.move(CartesianMotion(Affine(pos_m, pose[3:])))
        else:
            # Euler angles: position in mm -> m, angles in degrees -> radians
            import math
            from scipy.spatial.transform import Rotation

            x, y, z = [v / 1000.0 for v in pose[:3]]  # mm -> m
            rx_deg, ry_deg, rz_deg = pose[3:6]
            print(f"[DEBUG] position (m): {x}, {y}, {z}")
            rx_rad = math.radians(rx_deg)
            ry_rad = math.radians(ry_deg)
            rz_rad = math.radians(rz_deg)
            print(f"[DEBUG] rotation (rad): {rx_rad}, {ry_rad}, {rz_rad}")
            quat = Rotation.from_euler("xyz", [rx_rad, ry_rad, rz_rad]).as_quat()
            affine = Affine([x, y, z], quat)
            self._robot.move(CartesianMotion(affine))

        self._tool_pose = [float(v) for v in pose[:7]] if len(pose) >= 7 else [float(v) for v in pose] + [0.0] * (7 - len(pose))
        print(f"[DEBUG] Tool pose set to: {self._tool_pose}")


    def _execute_open_gripper(self, action: Dict[str, Any]) -> None:
        speed = float(action.get("speed", self.DEFAULT_GRIPPER_SPEED))
        self._gripper.open(speed)
        self._gripper_closed = False
        self._gripper_position = 1.0

    def _execute_close_gripper(self, action: Dict[str, Any]) -> None:
        speed = float(action.get("speed", self.DEFAULT_GRIPPER_SPEED))
        force = float(action.get("force", self.DEFAULT_GRIPPER_FORCE))
        width = float(action.get("width", self.DEFAULT_GRASP_WIDTH))
        self._gripper.grasp(width, speed, force)
        self._gripper_closed = True
        self._gripper_position = 0.0