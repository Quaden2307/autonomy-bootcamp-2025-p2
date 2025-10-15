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
    args: object,  # Place your own arguments here
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
    success, command_instance = command.Command.create(connection, target, args, local_logger)
    if not success or command_instance is None:
        local_logger.error("Failed to create Command instance", True)
        return  # Main loop: do work.

    velocity_samples = []
    while not controller.is_exit_requested():
        controller.check_pause()
        telemetry_data = input_queue.queue.get()

        if telemetry_data is None:
            continue

        avg_velocity = None

        if (
            telemetry_data.x_velocity is not None
            and telemetry_data.y_velocity is not None
            and telemetry_data.z_velocity is not None
        ):

            speed = (
                telemetry_data.x_velocity**2
                + telemetry_data.y_velocity**2
                + telemetry_data.z_velocity**2
            ) ** 0.5

            velocity_samples.append(speed)
            avg_velocity = sum(velocity_samples) / len(velocity_samples)
            local_logger.info(f"Average velocity so far: {avg_velocity:.3f} m/s", True)

        result, decision = command_instance.run(telemetry_data, avg_velocity)
        if not result:
            continue

        output_queue.queue.put(decision)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
