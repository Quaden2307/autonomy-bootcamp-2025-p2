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
        local_logger: logger.Logger,
    ) -> "HeartbeatReceiver":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        return HeartbeatReceiver(cls.__private_key, connection, local_logger)

    # Create a HeartbeatReceiver object

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        # Do any intializiation here

        self.connection = connection
        self.logger = local_logger

        self.missed_heartbeats = 0
        self.max_missed = (
            5  # Max number of heartbeats that can be missed before the connection lost
        )
        self.connected = False

    def run(
        self,
    ) -> tuple[bool, object]:
        """
        Attempt to recieve a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        """

        msg = self.connection.recv_match(type="HEARTBEAT", blocking=True)
        if msg is None:
            self.missed_heartbeats += 1

            self.logger.warning(f"Missed heartbeat ({self.missed_heartbeats})", True)

            if self.missed_heartbeats >= self.max_missed:
                if self.connected:
                    self.logger.error("Drone disconnected", True)
                    self.connected = False
                return False, "DISCONNECTED"

            return False, None

        self.missed_heartbeats = 0
        if not self.connected:
            self.logger.info("Drone Connected", True)
            self.connected = True
        else:
            self.logger.debug("HEARTBEAT_OK", True)
        return True, "HEARTBEAT_OK"


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
