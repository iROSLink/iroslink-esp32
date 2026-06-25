# iroslink-esp32

ESP32 firmware that receives ROS 2 drive commands over WiFi (via Zenoh) and controls a differential-drive robot with a suction fan and battery monitoring.

## Wiring

| Signal         | ESP32 GPIO |
|----------------|-----------|
| Left  IN1      | 25        |
| Left  IN2      | 33        |
| Right IN1      | 26        |
| Right IN2      | 27        |
| Fan ESC signal | 19        |
| Battery ADC    | 34        |
| Boot button    | 0         |

## How it works

```
ROS 2 (zenoh router)
  └─ /cmd_vel ──► rmw_zenoh_cpp ──► Zenoh router ──► WiFi ──► ESP32
                                                               ├── left / right motors
                                                               ├── suction fan ESC
                                                               └── battery voltage (published back)
```

| ROS 2 topic        | Direction | Description               |
|--------------------|-----------|---------------------------|
| `/cmd_vel`         | subscribe | drive commands            |
| `/suction_fan/speed` | subscribe | fan throttle 0.0–1.0    |
| `/battery/voltage` | publish   | battery voltage every 10 s |

## Setup

### 1. Install PlatformIO in VSCode

Install the **PlatformIO IDE** extension from the VSCode marketplace, then open this folder (`iroslink-esp32/`) in VSCode. PlatformIO will automatically install all dependencies including zenoh-pico.

### 2. Add WiFi credentials

```bash
cp include/secret.h.template include/secret.h
```

Edit `include/secret.h`:
```c
#define WIFI_SSID        "your-network"
#define WIFI_PASSWORD    "your-password"
#define ROUTER_MDNS_HOST "your-pc-hostname"  // e.g. "iPhone" → connects to iPhone.local:7447
```

The firmware tries to find the Zenoh router by mDNS first, then falls back to the WiFi gateway, then `192.168.2.1`. You can change this in the `src/main.cpp`

### 3. Flash

1. Plug in the ESP32 via USB
2. In the VSCode bottom toolbar click **→ Upload** (or open the PlatformIO sidebar → `esp32dev` → **Upload**)
3. After flashing, click **Serial Monitor** (plug icon) in the same toolbar — baud rate is set automatically to 74880

Expected output once connected:
```
Connecting to WiFi... OK — IP: 192.168.x.x
mDNS: mypc.local → tcp/192.168.x.x:7447
Opening Zenoh Session... OK
>> cmd_vel  lin.x=0.500  ang.z=0.000  lag=3ms  10.0Hz
bat=11.82V
```

## Running on ROS 2 Jazzy (Linux)

If you previously ran ROS 2 with a different middleware, clear the daemon first:
```bash
pkill -9 -f ros && ros2 daemon stop
```

**Terminal 1** — Zenoh router (bridge between ROS 2 and ESP32):
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Terminal 2** — keyboard teleoperation:
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Joystick

Install (one-time):
```bash
sudo apt install ros-jazzy-teleop-twist-joy ros-jazzy-joy
```

**Terminal 3** — gamepad driver:
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run joy joy_node
```

**Terminal 4** — joystick → `/cmd_vel`:
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run teleop_twist_joy teleop_node --ros-args \
    -p axis_linear.x:=1 \      # left stick up/down = forward/back
    -p axis_angular.yaw:=0 \   # left stick left/right = turn
    -p scale_angular.yaw:=1.0 \
    -p enable_button:=4 \      # hold L1/LB as deadman switch
    -p publish_stamped_twist:=false
```

> Hold the enable button (L1 / LB) while moving the stick. Releasing it stops the robot.

## Boot button (GPIO 0)

Press the onboard BOOT button to toggle the fan between 50% throttle and off — useful for testing without a ROS 2 connection.

---

See [INTERNALS.md](INTERNALS.md) for CDR wire format, drive mixing math, and ESC timing details.
