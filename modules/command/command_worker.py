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
    args,  # Place your own arguments here
    input_queue: queue_proxy_wrapper.QueueProxyWrapper,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
    # Add other necessary worker arguments here
) -> None:
    """
    Worker process.

    args... describe what the arguments are
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
    command_instance = command.Command.create(connection, target, args, local_logger)
    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()
        telemetry_data = input_queue.queue.get()

        if telemetry_data is None:
            break

        result, decision = command_instance.run(telemetry_data)
        if not result:
            continue

        output_queue.queue.put(decision)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
