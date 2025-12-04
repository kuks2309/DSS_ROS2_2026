# Copilot / AI Agent Instructions for dss_controller

This project contains two related components: the Python `dss_controller` package (ROS2 node + VSS client) and the `dss_ros2_bridge` bridge (ROS2/C++ + Protobuf/NATS). The guidance below focuses on actionable patterns, build/run commands, and code examples an AI agent should follow when making changes.

- **Big picture**: `dss_controller` is a pure-Python ROS2 node that uses a local Protobuf-generated module (`dss_pb2`) and a NATS-based VSS client (`DSSVssClient`) to communicate with a simulation/vehicle. Drive commands are sent via UDP; state requests use NATS subjects (`VSS.Get`/`VSS.Set`). The bridge (`dss_ros2_bridge`) is built with `colcon` and provides the ROS⇄DSS runtime integration.

- **Build & run (common tasks)**:
  - Install Python package (development): `cd ~/ros2_ws/src/dss_controller && pip3 install -e .`
  - Source ROS2 (Humble): `source /opt/ros/humble/setup.bash`
  - Build full workspace (bridge + packages): from `~/ros2_ws`: `colcon build --symlink-install`
  - Run bridge launch: `source install/setup.bash && ros2 launch dss_ros2_bridge launch.py`
  - Run controller node: `ros2 run dss_controller dss_controller` or `python3 dss_controller/dss_controller_node.py` (ensure ROS env is sourced).

- **Key files to read before edits**:
  - `dss_controller/dss_controller/dss_controller_node.py` — main ROS2 node, subscriptions, and control timer.
  - `dss_controller/dss_controller/dss_vss_client.py` — singleton VSS client, NATS async wrappers, UDP drive control.
  - `dss_controller/dss_controller/dss_pb2.py` — generated Protobuf messages used for UDP control payloads.
  - `../dss_ros2_bridge/README.md` — bridge build/installation and Protobuf/C++ expectations.

- **Project-specific patterns & conventions**:
  - Singleton client: use `DSSVssClient.singleton()` to obtain the shared client. Avoid creating multiple instances.
  - Async model: `DSSVssClient` runs an `asyncio` event loop in a background thread. Use `run_coroutine_threadsafe` via the provided helpers (`get`, `set`, `request_async`) instead of creating new loops.
  - Drive control uses UDP with a Protobuf message: see `dss_vss_client.set_drive_control(throttle, steer, brake)` which constructs `dss_pb2.DssSetControl()` and `SerializeToString()`.
  - NATS usage: requests go to subjects like `VSS.Get` and `VSS.Set`. The client returns `(data, status)` or futures via `asyncio.run_coroutine_threadsafe`.

- **When modifying code that touches communication**:
  - For UDP drive control changes, update `dss_pb2` usage in `dss_vss_client.py` and ensure serialization remains `SerializeToString()`.
  - For NATS subject changes, update both client calls and any bridge configuration/scripts that publish/subscribe those subjects (search for `VSS.Get` / `VSS.Set`).
  - Preserve the background `asyncio` thread model — tests and runtime expect a single loop managed by `DSSVssClient`.

- **Examples (copyable patterns)**
  - Instantiate client and call set/get (from `dss_controller_node.py`):
    ```python
    client = DSSVssClient.singleton()
    client.start(ip="172.26.160.1", drive_port=8886, vss_port=4222)
    client.set("Vehicle.Cabin.Door.Row1.DriverSide.Switch", "Open")
    fut = client.get("Vehicle.Cabin.Tailgate.Position")
    reply, status = fut.result()
    ```
  - Subscribe to ROS topics (node pattern):
    ```python
    self.imu_sub = self.create_subscription(Imu, "/dss/sensor/imu", self._on_imu, 10)
    ```

- **Build / test tips for agents**:
  - After Python changes, prefer `pip3 install -e .` to pick up local edits for iterative testing.
  - When changes touch the bridge, run `colcon build --symlink-install` from workspace root, then `source install/setup.bash` before running `ros2 launch`.
  - There are no unit tests in the repo—manual runtime checks are expected: verify NATS connections (console prints), UDP send messages, and ROS topic flow.

- **Common pitfalls to avoid**:
  - Do not create a second `asyncio` event loop in the main thread — use the client-provided background loop.
  - Avoid hard-coding different IP/port without documenting expected network topology (defaults in code use `172.26.160.1:8886/4222`).
  - When updating Protobuf messages, regenerate `dss_pb2.py` consistently and verify imports in `setup.py`/package metadata.

If anything in this summary is unclear or you want more concrete examples (e.g., a small test harness for the client or a sample launch override), tell me which area to expand and I will iterate.
