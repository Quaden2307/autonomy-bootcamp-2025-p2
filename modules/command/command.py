from __future__ import annotations

"""
Decision-making logic.
"""

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
        args: object,  # Put your own arguments here
        local_logger: logger.Logger,
    ) -> tuple[bool, Command | None]:
        """
        Falliable create (instantiation) method to create a Command object.
        """
        try:
            instance = Command(cls.__private_key, connection, target, args, local_logger)
            return True, instance
        except (OSError, ValueError, RuntimeError) as e:
            local_logger.error(f"Failed to create Command instance: {e}", True)
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        args: object,  # Put your own arguments here
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        # Do any intializiation here
        self.connection = connection
        self.target = target
        self.args = args
        self.logger = local_logger
        self.height_tolerance = 0.5
        self.angle_tolerance = 5
        self.turning_speed = 5

    def run(self, telemetry_data: object, avg_velocity: float | None = None) -> tuple[bool, str]:
        """Make a decision based on received telemetry data."""
        if avg_velocity is not None:
            self.logger.info(f"Average velocity (from worker): {avg_velocity:.3f} m/s", True)
        current_altitude = telemetry_data.z
        altitude_diff = self.target.z - current_altitude

        # Altitude adjustment logic
        if abs(altitude_diff) > self.height_tolerance:
            self.logger.info(f"Decision: CHANGE_ALTITUDE by {altitude_diff}", True)
            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                0,
                abs(altitude_diff),
                0,
                0,
                0,
                0,
                0,
                self.target.z,
            )
            return True, f"CHANGE_ALTITUDE: {altitude_diff}"

        # Yaw adjustment logic
        desired_yaw = math.atan2(self.target.y - telemetry_data.y, self.target.x - telemetry_data.x)
        current_yaw = telemetry_data.yaw
        yaw_diff = (desired_yaw - current_yaw) * 180 / math.pi
        yaw_diff = (yaw_diff + 180) % 360 - 180

        if abs(yaw_diff) > self.angle_tolerance:
            direction = 1 if yaw_diff > 0 else -1
            self.logger.info(f"Decision: CHANGING_YAW by {yaw_diff} degrees", True)
            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                abs(yaw_diff),
                self.turning_speed,
                direction,
                1,
                0,
                0,
                0,
            )
            return True, f"CHANGING_YAW: {yaw_diff}"

        self.logger.info("Decision: HOLDING_YAW (within tolerance)", True)
        return True, "HOLDING_YAW"


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
