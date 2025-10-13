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
HEIGHT_TOLERANCE = 0.5
ANGLE_TOLERANCE = 5
TURNING_SPEED = 5


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
    ) -> "Command":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        return Command(
            cls.__private_key, connection, target, args, local_logger
        )  #  Create a Command object

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

    def run(self, telemetry_data: object) -> tuple[bool, str]:
        """Make a decision based on received telemetry data."""
        current_altitude = telemetry_data.z

        if current_altitude < self.target.z - HEIGHT_TOLERANCE:
            delta_altitude = self.target.z - current_altitude
            self.logger.info(f"Decision: CHANGE_ALTITUDE by {delta_altitude}", True)
            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                0,
                self.target.z,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            return True, f"CHANGE_ALTITUDE: {delta_altitude}"

        if current_altitude > self.target.z + HEIGHT_TOLERANCE:
            delta_altitude = self.target.z - current_altitude
            self.logger.info(f"Decision: CHANGE_ALTITUDE by {delta_altitude}", True)
            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                0,
                self.target.z,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            return True, f"CHANGE_ALTITUDE: {delta_altitude}"

        desired_yaw = math.atan2(self.target.y - telemetry_data.y, self.target.x - telemetry_data.x)
        current_yaw = telemetry_data.yaw

        yaw_diff = (desired_yaw - current_yaw) * 180 / math.pi
        yaw_diff = (yaw_diff + 180) % 360 - 180

        if abs(yaw_diff) > ANGLE_TOLERANCE:
            self.logger.info(f"Decision: CHANGING_YAW by {yaw_diff} degrees", True)
            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                yaw_diff,
                TURNING_SPEED,
                1,
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
