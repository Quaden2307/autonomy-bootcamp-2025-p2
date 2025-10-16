"""
Bootcamp F2025

Main process to setup and manage all the other working processes
"""

import multiprocessing as mp
import queue
import time

from pymavlink import mavutil

from modules.common.modules.logger import logger
from modules.common.modules.logger import logger_main_setup
from modules.common.modules.read_yaml import read_yaml
from modules.command import command
from modules.command import command_worker
from modules.heartbeat import heartbeat_receiver_worker
from modules.heartbeat import heartbeat_sender_worker
from modules.telemetry import telemetry_worker
from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from utilities.workers import worker_manager


# MAVLink connection
CONNECTION_STRING = "tcp:localhost:12345"

# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Set queue max sizes (<= 0 for infinity)
QUEUE_MAXSIZE = 0
# Set worker counts
NUM_HEARTBEAT_SENDERS = 1
NUM_HEARTBEAT_RECEIVERS = 1
NUM_TELEMETRY = 1
NUM_COMMAND = 1
# Any other constants
RUN_TIME = 100
TARGET_POSITION = command.Position(10, 20, 30)
# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================


def main() -> int:
    """
    Main function.
    """
    # Configuration settings
    result, config = read_yaml.open_config(logger.CONFIG_FILE_PATH)
    if not result:
        print("ERROR: Failed to load configuration file")
        return -1

    # Get Pylance to stop complaining
    assert config is not None

    # Setup main logger
    result, main_logger, _ = logger_main_setup.setup_main_logger(config)
    if not result:
        print("ERROR: Failed to create main logger")
        return -1

    # Get Pylance to stop complaining
    assert main_logger is not None

    # Create a connection to the drone. Assume that this is safe to pass around to all processes
    # In reality, this will not work, but to simplify the bootamp, preetend it is allowed
    # To test, you will run each of your workers individually to see if they work
    # (test "drones" are provided for you test your workers)
    # NOTE: If you want to have type annotations for the connection, it is of type mavutil.mavfile
    connection = mavutil.mavlink_connection(CONNECTION_STRING)
    connection.wait_heartbeat(timeout=30)  # Wait for the "drone" to connect

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Create a worker controller
    controller = worker_controller.WorkerController()

    # Create a multiprocess manager for synchronized queues
    manager = mp.Manager()
    # Create queues

    heartbeat_out = queue_proxy_wrapper.QueueProxyWrapper(manager, maxsize=QUEUE_MAXSIZE)

    telemetry_out = queue_proxy_wrapper.QueueProxyWrapper(manager, maxsize=QUEUE_MAXSIZE)

    command_out = queue_proxy_wrapper.QueueProxyWrapper(manager, maxsize=QUEUE_MAXSIZE)
    # Create worker properties for each worker type (what inputs it takes, how many workers)

    # Heartbeat sender
    heartbeat_sender_props = worker_manager.WorkerProperties.create(
        heartbeat_sender_worker.heartbeat_sender_worker,  # target (function)
        (
            connection,
            {"controller": controller},
        ),  # work_arguments
        NUM_HEARTBEAT_SENDERS,  # count
        [],  # input_queues
        [],  # output_queues
        controller,  # controller
        main_logger,
    )

    # Heartbeat receiver
    heartbeat_receiver_props = worker_manager.WorkerProperties.create(
        heartbeat_receiver_worker.heartbeat_receiver_worker,
        (connection, {"controller": controller, "output_queue": heartbeat_out}),
        NUM_HEARTBEAT_RECEIVERS,
        [],
        [heartbeat_out],
        controller,
        main_logger,
    )

    # Telemetry
    telemetry_props = worker_manager.WorkerProperties.create(
        telemetry_worker.telemetry_worker,
        (
            connection,
            {"controller": controller, "output_queue": telemetry_out},
        ),
        NUM_TELEMETRY,
        [],
        [telemetry_out],
        controller,
        main_logger,
    )
    # Command
    command_props = worker_manager.WorkerProperties.create(
        command_worker.command_worker,
        (connection, TARGET_POSITION, None, telemetry_out, command_out, controller, main_logger),
        NUM_COMMAND,
        [telemetry_out],  # inputs
        [command_out],  # outputs
        controller,
        main_logger,
    )

    workers = []

    for props, name in [
        (heartbeat_sender_props, "heartbeat sender"),
        (heartbeat_receiver_props, "heartbeat receiver"),
        (telemetry_props, "telemetry"),
        (command_props, "command"),
    ]:
        result, worker = worker_manager.WorkerManager.create(
            props, controller
        )
        if not result or worker is None:
            main_logger.error(f"Failed to create {name} worker manager", True)
            return -1
        workers.append(worker)

    for w in workers:
        w.start_workers()

    main_logger.info("All workers started successfully")

    # Main's work: read from all queues that output to main, and log any commands that we make
    # Continue running for 100 seconds or until the drone disconnects
    start_time = time.time()
    while time.time() - start_time < RUN_TIME and controller.is_exit_requested() is False:
        try:
            while not heartbeat_out.queue.empty():
                msg = heartbeat_out.queue.get_nowait()

                if isinstance(msg, dict) and "status" in msg:
                    status = msg["status"].upper()
                    if status == "DISCONNECTED":
                        main_logger.warning("Drone disconnected — shutting down workers", True)
                        controller.request_exit()
                        break
                    if status == "CONNECTED":
                        main_logger.info("Connection status: connected")
                    else:
                        main_logger.debug(f"Heartbeat status: {status}")
                else:
                    main_logger.info(f"Heartbeat: {msg}")

            if not telemetry_out.queue.empty():
                data = telemetry_out.queue.get_nowait()
                main_logger.info(f"Telemetry: {data}")

            if not command_out.queue.empty():
                message = command_out.queue.get_nowait()
                if isinstance(message, dict) and "type" in message:
                    if message["type"] == "velocity":
                        v = message["data"]
                        main_logger.info(
                            f"Average velocity vector: x={v['x']:.3f}, y={v['y']:.3f}, z={v['z']:.3f} m/s"
                        )
                    elif message["type"] == "decision":
                        main_logger.info(f"Command decision: {message['data']}")
                else:
                    main_logger.warning(f"Unexpected message format: {message}")

        except queue.Empty:
            continue
        except (OSError, RuntimeError, ValueError) as e:
            main_logger.error(f"Error while reading queues: {e}")
            break

        # small sleep to avoid pegging CPU
        time.sleep(0.1)
    # Stop the processes
    controller.request_exit()
    main_logger.info("Requested exit")

    # Drain queues (end to start)
    queue_proxy_wrapper.QueueProxyWrapper.drain_all([command_out, telemetry_out, heartbeat_out])
    main_logger.info("Queues cleared")

    # Clean up worker processes
    for w in reversed(workers):
        w.stop_workers()

    for w in reversed(workers):
        w.join_workers()
    main_logger.info("Stopped")

    # We can reset controller in case we want to reuse it
    # Alternatively, create a new WorkerController instance

    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Failed with return code {result_main}")
    else:
        print("Success!")
