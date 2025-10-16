"""
Decision-making logic.
"""

from __future__ import annotations

import math
from pymavlink import mavutil
from ..common.modules.logger import logger


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================


class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> tuple[bool, Command | None]:
        """
        Falliable create (instantiation) method to create a Command object.
        """
        try:
            instance = Command(cls.__private_key, connection, target, local_logger)
            return True, instance
        except (OSError, ValueError, RuntimeError) as e:
            local_logger.error(f"Failed to create Command instance: {e}", True)
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        # Do any intializiation here
        self.connection = connection
        self.target = target
        self.logger = local_logger
        self.height_tolerance = 0.5
        self.angle_tolerance = 5
        self.turning_speed = 5

    def run(self, telemetry_data: object) -> tuple[bool, dict]:
        """Make a decision based on received telemetry data."""

        # Initialize persistent velocity samples
        if not hasattr(self, "x_samples"):
            self.x_samples = []
            self.y_samples = []
            self.z_samples = []

        # --- Compute average velocity vector ---
        if (
            telemetry_data.x_velocity is not None
            and telemetry_data.y_velocity is not None
            and telemetry_data.z_velocity is not None
        ):
            self.x_samples.append(telemetry_data.x_velocity)
            self.y_samples.append(telemetry_data.y_velocity)
            self.z_samples.append(telemetry_data.z_velocity)

            avg_velocity = (
                sum(self.x_samples) / len(self.x_samples),
                sum(self.y_samples) / len(self.y_samples),
                sum(self.z_samples) / len(self.z_samples),
            )
        else:
            avg_velocity = (0.0, 0.0, 0.0)

        # --- Compute altitude difference ---
        current_altitude = telemetry_data.z
        altitude_diff = self.target.z - current_altitude

        # --- Compute yaw difference ---
        desired_yaw = math.atan2(self.target.y - telemetry_data.y, self.target.x - telemetry_data.x)
        current_yaw = telemetry_data.yaw
        yaw_diff = (desired_yaw - current_yaw) * 180 / math.pi
        yaw_diff = (yaw_diff + 180) % 360 - 180

        # --- Determine action ---
        if abs(altitude_diff) > self.height_tolerance:
            action = "CHANGE_ALTITUDE"
        elif abs(yaw_diff) > self.angle_tolerance:
            action = "CHANGING_YAW"
        else:
            action = "HOLDING_YAW"

        # --- Return structured decision (no logging or MAVLink commands) ---
        return True, {
            "avg_velocity": {"x": avg_velocity[0], "y": avg_velocity[1], "z": avg_velocity[2]},
            "altitude_diff": altitude_diff,
            "yaw_diff": yaw_diff,
            "action": action,
        }


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
