"""
Heartbeat sending logic.
"""

from pymavlink import mavutil
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatSender:
    """
    HeartbeatSender class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
 
    ) -> tuple[bool, "HeartbeatSender | None"]:
        """
        Falliable create (instantiation) method to create a HeartbeatSender object.
        """
        try:
            instance = HeartbeatSender(cls.__private_key, connection, local_logger)
            local_logger.info("HeartbeatSender instance created successfully", True)
            return True, instance
        except (OSError, ValueError, RuntimeError) as e:
            local_logger.error(f"Failed to create HeartbeatSender: {e}", True)
            return False, None  

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatSender.__private_key, "Use create() method"

        # Do any intializiation here
        self.connection = connection
        self.logger = local_logger
        self.system_id = 1
        self.component_id = 1

    def run(
        self,

    ) -> tuple[bool, str]:
        """
        Attempt to send a heartbeat message.
        """

        try:
            self.connection.mav.heartbeat_send(
                type=mavutil.mavlink.MAV_TYPE_GCS,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self.logger.debug("Heartbeat sent successfully", True)
            return True, "HEARTBEAT_SENT"
        except (OSError, ValueError, RuntimeError) as e:
            self.logger.error(f"Failed to send heartbeat: {e}", True)
            return False, "HEARTBEAT_FAILED"
        # Send a heartbeat message


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
