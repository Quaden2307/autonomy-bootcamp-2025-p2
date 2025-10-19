"""
Heartbeat receiving logic.
"""

from __future__ import annotations

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
    ) -> tuple[bool, "HeartbeatReceiver" | None]:
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        try:
            instance = HeartbeatReceiver(cls.__private_key, connection, local_logger)
            return True, instance
        except (OSError, ValueError, RuntimeError) as e:
            local_logger.error(f"Failed to create HeartbeatReceiver: {e}", True)
            return False, None

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

    def run(self) -> tuple[bool, dict]:
        """Receive a heartbeat and determine connection status."""
        msg = self.connection.recv_match(type="HEARTBEAT", blocking=False)

        # No message received
        if msg is None:
            self.missed_heartbeats += 1
            if self.missed_heartbeats >= self.max_missed:
                if self.connected:
                    self.connected = False
                    return False, {"status": "DISCONNECTED", "log": "Drone disconnected"}
            return False, {"status": "WAITING", "log": ""}

        # Successful heartbeat
        self.missed_heartbeats = 0
        was_connected = self.connected
        self.connected = True

        if not was_connected:

            return True, {"status": "CONNECTED", "log": "Drone Connected!"}

        return True, {"status": "HEARTBEAT_OK", "log": ""}


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
