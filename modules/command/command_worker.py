"""
Command worker to make decisions based on Telemetry Data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import command
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def command_worker(
    connection: mavutil.mavfile,
    target: command.Position,
    input_queue: queue_proxy_wrapper.QueueProxyWrapper,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
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
    # Instantiate class object (command.Command)
    success, command_instance = command.Command.create(connection, target, local_logger)
    if not success:
        local_logger.error("Failed to create Command instance", True)
        return

    while not controller.is_exit_requested():
        controller.check_pause()
        telemetry_data = input_queue.queue.get()

        if telemetry_data is None:
            continue

        result, decision_data = command_instance.run(telemetry_data)
        if not result:
            continue

        action = decision_data["action"]

        # === MAVLink execution logic ===
        if action == "CHANGE_ALTITUDE":
            altitude_diff = decision_data["altitude_diff"]
            connection.mav.command_long_send(
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
                target.z,
            )

        elif action == "CHANGING_YAW":
            yaw_diff = decision_data["yaw_diff"]
            direction = -1 if yaw_diff > 0 else 1
            connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                abs(yaw_diff),
                command_instance.turning_speed,
                direction,
                1,
                0,
                0,
                0,
            )

        # === Push decision to output queue ===
        output_queue.queue.put(
            {
                "type": "decision",
                "data": decision_data,
            }
        )

    local_logger.info("Worker Shutting Down", True)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
