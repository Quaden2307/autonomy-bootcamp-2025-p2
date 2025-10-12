"""
Heartbeat receiving logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatReceiver:
    """
    HeartbeatReceiver class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        args: object,  # Put your own arguments here
        local_logger: logger.Logger,
    ) -> "HeartbeatReceiver":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        return HeartbeatReceiver(cls.__private_key, connection, args, local_logger)

    # Create a HeartbeatReceiver object

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        args: object,  # Put your own arguments here
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        # Do any intializiation here

        self.connection = connection
        self.logger = local_logger
        self.args = args

        self.missed_heartbeats = 0
        self.max_missed = (
            5  # Max number of heartbeats that can be missed before the connection lost
        )

    def run(
        self,
        args: object,  # Put your own arguments here
    ) -> tuple[bool, object]:
        """
        Attempt to recieve a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        """

        _ = args
        
        msg = self.connection.recv_match(type="HEARTBEAT", blocking=True)
        if msg is None:
            self.missed_heartbeats += 1
            self.logger.warning(f"Missed heartbeat ({self.missed_heartbeats})", True)

            if self.missed_heartbeats >= self.max_missed:
                self.logger.error("Connection lost", True)
                return False, "DISCONNECTED"

            return False, None

        self.missed_heartbeats = 0
        self.logger.info("Heartbeat received", True)
        return True, "HEARTBEAT_OK"


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
