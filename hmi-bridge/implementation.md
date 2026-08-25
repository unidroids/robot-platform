# HMI Bridge Microservice Walkthrough

The HMI Bridge microservice has been successfully implemented and integrated according to the design plan. Below is a summary of the implemented features.

## Architecture & File Structure

The project has been set up in `/opt/projects/robotour/hmi-bridge/` utilizing a similar structure to the `fusion` and `gamepad` services:

- **`main.py`**: The entry point. Initializes the service on port `9020`, defines the `DEVICE_ID = "120453749J000566"`, starts the ZMQ pollers on background threads, and orchestrates the shutdown sequence.
- **`service.py`**: Contains `HMIService`, which manages the raw TCP socket server on port `9020`. Handles incoming commands (`PING`, `SYNC`, `STATUS`, `EXIT`, `SHUTDOWN`).
  - Sets up the `adb` bridge via `adb reverse` for ports 9000, 8001, and 8002, and `adb forward` for 9021 and 9022.
  - Generates robust command-line output indicating what mappings are established and any errors.
- **`poller.py`**: A `ZMQPoller` that sets up a `SUB` socket listening on all interfaces `0.0.0.0` for incoming ZMQ `PUB` messages from the Android device.
  - Automatically translates the incoming 3-frame ZMQ payload by routing the final two frames to the dynamically created IPC socket `ipc:///tmp/<channel_name>`.
- **`client.py`**: Provides the `HMIClient` utility class for seamless programmatic interaction (e.g., triggering a `SYNC` programmatically) with the `9020` port.

## Integration

The global logger (`/opt/projects/robotour/logger/logger_service.py`) was also modified to incorporate the new incoming data streams from the HMI app:
- `ipc:///tmp/robot-terminal`
- `ipc:///tmp/robot-qrscaner`

These have been appended to the existing `endpoints` list, and `zmq` will poll these channels identically to the others.

## Verification
- Code structure follows the required conventions.
- ZMQ Pub/Sub routing conforms to the specified pattern.
- The `adb` commands enforce strict port-forwarding between the host robot and the remote HMI client running on the Infinix phone.
- The raw python files were successfully validated using `py_compile`.
