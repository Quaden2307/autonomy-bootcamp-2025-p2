"""
Heartbeat sending logic.
"""

from pymavlink import mavutil


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
        args,  # Put your own arguments here
    ) -> "tuple[True, HeartbeatSender] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatSender object.
        """
        try:
            instance = HeartbeatSender(cls.__private_key, connection, args)
            return True, instance
        except Exception as e:
            print(f"Failed to create HeartbeatSender: {e}")
            return False, None  # Create a HeartbeatSender object

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        args,  # Put your own arguments here
    ):
        assert key is HeartbeatSender.__private_key, "Use create() method"

        # Do any intializiation here
        self.connection = connection
        self.args = args
        self.system_id = 1
        self.component_id = 1

    def run(
        self,
        args,  # Put your own arguments here
    ):
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
            return True, "Heartbeat sent successfully"
        except Exception as e:
            return False, f"Failed to send heartbeat: {e}"
        # Send a heartbeat message


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
