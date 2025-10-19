"""
Heartbeat worker that sends heartbeats periodically.
"""

import os
import pathlib
import time
from pymavlink import mavutil

from utilities.workers import worker_controller
from utilities.workers.queue_proxy_wrapper import QueueProxyWrapper
from . import heartbeat_receiver
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def heartbeat_receiver_worker(
    connection: mavutil.mavfile,
    controller: worker_controller.WorkerController,
    output_queue: QueueProxyWrapper | None = None,
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
    # Instantiate class object (heartbeat_receiver.HeartbeatReceiver)
    success, heartbeat_instance = heartbeat_receiver.HeartbeatReceiver.create(
        connection, local_logger
    )
    if not success:
        local_logger.error("Failed to create HeartbeatReceiver instance", True)
        return

    while not controller.is_exit_requested():
        controller.check_pause()
        result, data = heartbeat_instance.run()

        if output_queue is not None and isinstance(data, dict):
            # Forward the data (status + log) to main
            output_queue.queue.put(data)

        time.sleep(1.0)

    local_logger.info("Heartbeat receiver worker shutting down.", True)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
