# X19-Core: Purdue ROV ZeroMQ & Protobuf System

This repository hosts the core communication, sensing, and control architecture for the Purdue ROV **X-19** underwater vehicle. The system uses **ZeroMQ (ZMQ)** for low-latency inter-process communication and **Protocol Buffers (Protobuf)** for type-safe, cross-language serialization (C++ and Python).

---

## 📂 Codebase Directory Layout

```
X19-Core/
├── config/                  # Configuration files (ports, address bindings)
├── launch.sh                # Bare-bones node launcher script (no tmux)
├── launch.yaml              # Declarative node configuration
├── src/                    # Communication wrappers & middleware
│   ├── cpp/                 # C++ wrapper library
│   └── python/              # Python wrapper library (python.messaging package)
│       └── messaging/
│           ├── publisher.py
│           └── subscriber.py
├── nodes/                   # Independent executable nodes
│   ├── cpp/                 # C++ nodes (thrusters, pid controls, etc.)
│   └── python/              # Python nodes (video, pilot control, etc.)
├── proto/                   # Protobuf message schemas
├── scripts/                 # Utility scripts (setup, compilation, launch runner)
│   ├── compile_protos.sh    # Protobuf schema compiler
│   ├── launch.py            # Lightweight YAML process runner
│   └── setup.sh             # System dependency provisioner
├── testing/                 # Validation and testing scripts
├── run.sh                   # Test runner shortcut script
└── MVPs.md                  # Development checklist & milestones
```

---

## ⚙️ Setup & Installation

### 1. System & Python Dependencies
Prerequisites include the Protobuf Compiler (`protoc`), ZeroMQ development headers, and Python 3. 

Run the automated setup script to provision your system:
```bash
./scripts/setup.sh
```
*(This installs `protobuf-compiler` and `libzmq3-dev` via apt-get, and builds the Python dependencies defined in `requirements.txt` into your virtual environment).*

### 2. Compile Protobuf Schemas
Compile your `.proto` files in the `proto/` directory to generate C++ and Python bindings:
```bash
./scripts/compile_protos.sh
```
This outputs compiled bindings directly to `src/protocols/cpp` and `src/protocols/python`.

---

## 🚀 Running the Code

### 1. Multi-Node Launcher (`launch.sh` & `launch.yaml`)
To launch all vehicle nodes on the Raspberry Pi without `tmux`:

```bash
# 1. Launch with default surface address (192.168.1.7:5556)
./launch.sh

# 2. Launch with a specific Surface laptop IP address
./launch.sh -i 192.168.1.100

# 3. View options
./launch.sh --help
```

#### How the Launch Workflow Operates:
1. **Dependency Verification**: [`launch.sh`](launch.sh) verifies that `python3` and `pyyaml` are installed. If missing, it alerts you with instructions to install them via `pip install -r requirements.txt`.
2. **Environment & IP Setup**: Formats the Surface IP into `tcp://<IP>:5556` and exports `SURFACE_ZMQ_ADDRESS`.
3. **Execution**: Hands off [`launch.yaml`](launch.yaml) to [`scripts/launch.py`](scripts/launch.py), which starts each defined node in the background and tags console logs with node labels (e.g. `[videos]`).
4. **Graceful Cleanup**: Pressing `Ctrl+C` cleanly shuts down all nodes and terminates any spawned child processes (e.g., `ffmpeg` camera streams).

#### Adding Future Nodes:
To add new nodes (e.g. thrusters, telemetry), simply add their entry to [`launch.yaml`](launch.yaml):
```yaml
nodes:
  - name: "videos"
    cmd: "python3 src/python/videos/get_ip.py"

  # Future nodes:
  - name: "thrusters"
    cmd: "python3 src/python/thrusters/thruster_node.py"
```

### 2. Standalone Test Scripts (`run.sh`)
Use the root [run.sh](file:///home/aditya/purdue/ROV/X-19/X19-Core/run.sh) script to execute standalone test scripts:
```bash
# Run the hello_pub test node
./run.sh

# Recompile protobuf schemas AND run the hello_pub test node
./run.sh -p
```

---

## 🛠️ Messaging Wrapper API

To make writing nodes simple and robust, lightweight wrappers wrap ZMQ sockets into easy-to-use classes.

### C++ API

The C++ wrappers reside in the `CppMsg` namespace and provide RAII-compliant management of ZeroMQ sockets and Protobuf serialization.

#### 1. Publisher (`CppMsg::Publisher`)
Creates a ZMQ `PUB` socket that serializes and publishes Protobuf messages on a specific topic.

```cpp
#include "src/cpp/messaging/publisher.hpp"
#include "src/protocols/cpp/telemetry.pb.h"

// Initialize publisher (binds to address by default)
CppMsg::Publisher publisher("tcp://127.0.0.1:5555", "telemetry", /*bind=*/true);

// Create and publish a Protobuf message
telemetry::SensorData msg;
msg.set_depth(1.24);
bool success = publisher.publish(msg);

// Clean up (also automatically closed via destructor)
publisher.close();
```

*   `Publisher(std::string addressStr, std::string topicStr, bool bind = true)`
    *   `addressStr`: ZMQ endpoint address (e.g., `tcp://*:5555`).
    *   `topicStr`: Topic string prefixed as the first multipart frame.
    *   `bind`: Binds the socket if `true`; connects if `false`.
*   `publish(const google::protobuf::MessageLite &protoMessage) -> bool`
    *   Serializes any message derived from `MessageLite` and publishes it in a two-part frame (`[topic, payload]`). Returns `true` if successful.
*   `close() -> void`
    *   Closes the underlying ZeroMQ socket.

#### 2. Subscriber (`CppMsg::Subscriber<MessageType>`)
A templated subscriber class that manages a ZMQ `SUB` socket, subscribes to a topic filter, and deserializes incoming Protobuf frames to invoke a user-defined callback.

```cpp
#include <iostream>
#include <chrono>
#include "src/cpp/messaging/subscriber.hpp"
#include "src/protocols/cpp/telemetry.pb.h"

void telemetry_callback(const telemetry::SensorData &data) {
    std::cout << "Received depth: " << data.depth() << std::endl;
}

// Initialize subscriber (connects to address by default)
CppMsg::Subscriber<telemetry::SensorData> subscriber(
    "tcp://127.0.0.1:5555",
    "telemetry",
    telemetry_callback,
    /*bind=*/false
);

// Option A: Single non-blocking / timed poll
subscriber.spinOnce(std::chrono::milliseconds(100));

// Option B: Asynchronous reception in a dedicated worker thread
subscriber.startSpinThreaded();
// ... execute work ...
subscriber.stopSpinThreaded();

// Clean up (also automatically closed via destructor)
subscriber.close();
```

*   `Subscriber(std::string addressStr, std::string topicStr, CallbackType callbackFn, bool bind = false)`
    *   `addressStr`: Target publisher endpoint (e.g., `tcp://127.0.0.1:5555`).
    *   `topicStr`: Topic prefix filter to subscribe to.
    *   `callbackFn`: Callback signature matching `std::function<void(const MessageType &)>`.
    *   `bind`: Binds socket if `true`; connects if `false`.
*   `spinOnce(std::chrono::milliseconds timeout_ms = 10ms) -> bool`
    *   Polls the socket for incoming frames within the given timeout. Parses the payload, executes the callback, and drains any remaining frames. Returns `true` if a message was handled.
*   `spin() -> void`
    *   Enters a continuous blocking receive loop on the calling thread.
*   `startSpinThreaded() -> bool`
    *   Spawns a background thread running `spinThreaded()`. Returns `true` if started.
*   `stopSpinThreaded() -> bool`
    *   Signals the worker thread to stop and joins it. Returns `true` if joined.
*   `close() -> void`
    *   Stops any active background thread and closes the underlying ZeroMQ socket.

---

### Python API

#### 1. Publisher (`python.messaging.Publisher`)
Creates a ZMQ `PUB` socket that serializes and publishes Protobuf messages on a specific topic.

```python
from src.python.messaging import Publisher
from src.protocols.python import telemetry_pb2

# Initialize publisher (binds to address by default)
publisher = Publisher(address="tcp://127.0.0.1:5555", topic="telemetry")

# Create and publish a Protobuf message
msg = telemetry_pb2.SensorData(depth=1.24)
publisher.publish(msg)

# Clean up
publisher.close()
```

*   `__init__(address: str, topic: str, bind: bool = True)`
    *   `address`: ZMQ address endpoint (e.g., `tcp://*:5555`).
    *   `topic`: Topic name string.
    *   `bind`: Binds socket to the port if `True`; connects if `False`.
*   `publish(proto_message)`
    *   Serializes the Protobuf class instance and publishes it.
*   `close()`
    *   Closes the underlying ZMQ socket.

#### 2. Subscriber (`core.messaging.Subscriber`)
Creates a ZMQ `SUB` socket that connects to a publisher, subscribes to a topic, and parses incoming Protobuf payloads.

```python
from src.python.messaging import Subscriber
from src.protocols.python import telemetry_pb2

def telemetry_callback(data):
    print(f"Received depth: {data.depth}")

# Initialize subscriber (connects to address by default)
subscriber = Subscriber(
    address="tcp://127.0.0.1:5555",
    topic="telemetry",
    message_type=telemetry_pb2.SensorData,
    callback=telemetry_callback
)

# Process a single incoming message (timeout in milliseconds)
subscriber.spin_once(timeout_ms=100)

# Clean up
subscriber.close()
```

*   `__init__(address: str, topic: str, message_type, callback, bind: bool = False)`
    *   `address`: Target publisher endpoint (e.g., `tcp://127.0.0.1:5555`).
    *   `topic`: Subscribed topic string.
    *   `message_type`: Protobuf class type used to deserialize payloads.
    *   `callback`: Callback function signature `def callback(message)`.
    *   `bind`: Binds socket if `True`; connects if `False`.
*   `spin_once(timeout_ms: int)`
    *   Polls the socket. If data is available, it deserializes the payload and invokes the callback. Returns `True` if processed, otherwise `False`.
*   `close()`
    *   Closes the underlying ZMQ socket.

#### 3. Running The Code
- set python path to dir root
```bash
export PYTHONPATH=<project_root>
```
- run publisher using virtual env first
```bash
./.venv/bin/python ./testing/hello_pub.py 
```
- run subscriber using virtual env next
```bash
./.venv/bin/python ./testing/hello_sub.py 
```

---

## 📹 Video Streaming & ZMQ IP Discovery

`X19-Core` contains the camera acquisition and low-latency H.264 RTSP streaming pipeline for the ROV cameras.

```
┌──────────────────────────────────────────────────────────────────┐
│                   X19-Core Video Subsystem                       │
│                                                                  │
│  1. get_ip.py (ZMQ Subscriber on port 5556)                      │
│     - Connects to tcp://<SURFACE_IP>:5556 on topic 'surface_ip'  │
│     - Deserializes Protobuf payload: telemetry_pb2.test(msg=ip)  │
│                                                                  │
│  2. V4L2 Device Auto-Discovery                                   │
│     - Runs 'v4l2-ctl --list-devices'                             │
│     - Detects attached exploreHD / Arducam / Intel cameras       │
│                                                                  │
│  3. FFmpeg RTSP Streamers (videos_launch.py)                     │
│     - Spawns background FFmpeg process for each camera           │
│     - Encodes H.264 / MJPEG video with zerolatency preset        │
│     - Pushes RTSP stream to rtsp://<SURFACE_IP>:8554/camera<N>   │
└──────────────────────────────────────────────────────────────────┘
```

### Video Nodes in `src/python/videos/`:
- **[`get_ip.py`](file:///home/aditya/purdue/ROV/X-19/X19-Core/src/python/videos/get_ip.py)**: ZMQ subscriber node listening for Surface IP broadcast. Once IP is received, triggers camera discovery and starts streaming processes.
- **[`videos_launch.py`](file:///home/aditya/purdue/ROV/X-19/X19-Core/src/python/videos/videos_launch.py)**: Low-latency FFmpeg RTSP camera streamer node (`--ip`, `--device`, `--camera-number`).
- **[`cv_camera_connect.py`](file:///home/aditya/purdue/ROV/X-19/X19-Core/src/python/videos/cv_camera_connect.py)**: DepthAI camera RTSP streamer node for OAK/DepthAI hardware.

---

## 🧭 IMU Node (OAK-D W Pro / BNO086)

[`src/python/sensors/imu_node.py`](src/python/sensors/imu_node.py) reads the OAK-D W Pro's onboard BNO086 IMU via DepthAI and publishes each sample as an `ImuData` protobuf message (see `proto/telemetry.proto`) over ZMQ - the sensor feed for the upcoming vertical-stabilization PID loop.

### OAK-D / BNO086 Configuration

The pipeline enables three report types at a configurable rate (default 100 Hz, `--rate-hz`):
- **`ACCELEROMETER`** - calibrated linear acceleration (m/s²)
- **`GYROSCOPE_CALIBRATED`** - calibrated angular velocity (rad/s)
- **`ROTATION_VECTOR`** - the BNO086's on-chip sensor-fused orientation (quaternion), used instead of integrating raw gyro data on the host so orientation doesn't drift

Batching is set to `setBatchReportThreshold(1)` / `setMaxBatchReports(10)` - samples are sent as soon as one is ready (lowest latency, favoring responsiveness for a control-loop input over USB throughput). Note the BNO086 rounds the requested rate up to the nearest supported rate, and Luxonis's docs note gyro rates above ~400 Hz can jitter on RVC2 hardware - the 100 Hz default is well under that.

### Dependencies

- **DepthAI v2.x, not v3.** `pip install depthai` installs the latest release (v3), which restructured the pipeline API (`dai.node.XLinkOut` no longer exists) and breaks this file as well as the existing `cv_camera_connect.py`. Install the v2 line explicitly:
  ```bash
  pip install "depthai<3"
  ```
  - IMU node docs: https://docs.luxonis.com/software/depthai-components/nodes/imu/
  - Official IMU example source (field names/API used in `packet_to_proto`): https://github.com/luxonis/depthai-python/tree/main/examples/IMU
- **udev rule** so DepthAI can access the camera over USB without root (one-time, per machine):
  ```bash
  echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
  Then **physically unplug and replug the camera** - a device already enumerated before the rule existed needs to re-enumerate to pick up the new permissions.

### Running

```bash
export PYTHONPATH=$(pwd)
.venv/bin/python src/python/sensors/imu_node.py
```
Defaults to `tcp://127.0.0.1:5557`, topic `imu` (see the port table below). Only one process can hold the DepthAI device open at a time - make sure nothing else (e.g. `cv_camera_connect.py`) is using the camera first.

To test the ZMQ/protobuf wiring without the camera attached, `testing/imu_fixture_publisher.py` publishes synthetic data on the same address/topic/schema.

---

## 🌐 Network Ports & Field Deployment Guide

When running on the Raspberry Pi connected to the Surface Topside Laptop via Ethernet/tether network:

| Port | Protocol | Target Node | Purpose |
|:---:|:---:|:---|:---|
| **5555** | ZMQ (TCP) | Surface Telemetry | Receives telemetry packets on topic `telemetry` |
| **5556** | ZMQ (TCP) | Surface IP Publisher | Receives Surface IP broadcast on topic `surface_ip` |
| **5557** | ZMQ (TCP) | IMU Node | Publishes `ImuData` packets on topic `imu` |
| **8554** | RTSP (TCP) | Go2RTC Server | Pushes RTSP camera streams to Surface media server |

### Running on Raspberry Pi (Hardware Network Setup):
1. **On Surface Topside Laptop**: Start `go2rtc_node.py` (or run `./launch --field` in `X19-Surface`). Note the Surface laptop IP address (e.g. `192.168.1.100`).
2. **On Raspberry Pi (`X19-Core`)**: Launch all vehicle nodes pointing to the Surface laptop's IP address:
   ```bash
   ./launch.sh -i 192.168.1.100
   ```
   *(This automatically forwards the Surface address to `get_ip.py`, discovers local cameras, and streams RTSP video back to the Surface computer).*