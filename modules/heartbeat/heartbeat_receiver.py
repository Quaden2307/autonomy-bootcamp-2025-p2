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
        """Initialize the HeartbeatReceiver object."""
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        # Do any intializiation here

        self.connection = connection
        self.logger = local_logger

        self.missed_heartbeats = 0
        self.max_missed = (
            5  # Max number of heartbeats that can be missed before the connection lost
        )
        self.connected = False

    def run(self) -> dict:
        """Receive a heartbeat and determine connection status."""
        msg = self.connection.recv_match(type="HEARTBEAT", blocking=False)
        if msg is None:
            self.missed_heartbeats += 1
        else:
            self.missed_heartbeats = 0

        if self.missed_heartbeats < self.max_missed:
            if not self.connected:
                self.connected = True
                return {"status": "CONNECTED", "log": "Drone Connected!"}
            return {"status": "CONNECTED", "log": ""}

        if self.connected:
            self.connected = False
            return {"status": "DISCONNECTED", "log": "Drone Disconnected!"}
        return {"status": "DISCONNECTED", "log": ""}


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
