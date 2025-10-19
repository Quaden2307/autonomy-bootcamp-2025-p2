"""
Heartbeat worker that sends heartbeats periodically.
"""

import os
import pathlib
import time

from pymavlink import mavutil

from utilities.workers import worker_controller
from . import heartbeat_sender
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def heartbeat_sender_worker(
    connection: mavutil.mavfile,
    controller: worker_controller.WorkerController,
    # Add other necessary worker arguments here
) -> None:
    """
    Worker process.


    """
    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (heartbeat_sender.HeartbeatSender)
    while not controller.is_exit_requested():
        controller.check_pause()

        try:
            # Send a MAVLink HEARTBEAT message
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,  # generic system type
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # no autopilot
                0, 0, 0, 0  # base_mode, custom_mode, system_status
            )
            local_logger.debug("Sent heartbeat message", True)
        except Exception as e:
            local_logger.error(f"Failed to send heartbeat: {e}", True)

        time.sleep(1.0)  # 1 Hz heartbeat rate

    local_logger.info("Heartbeat sender worker shutting down.", True)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
