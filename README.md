# iroslink-esp32

ESP32 firmware that receives ROS 2 drive commands over WiFi (via Zenoh) and controls a differential-drive robot with a suction fan and battery monitoring.

## LED status (GPIO 2)

| Pattern | Meaning |
|---------|---------|
| Fast blink 100ms on, 100ms off | Connecting to WiFi |
| Blink 100ms on, 2000ms off  | Connecting to Router |
| Single flash on each message | cmd\_vel received and applied |
| Slow blink 1s on / 1 s off | No cmd\_vel for >500 ms — motors stopped, waiting for comms |



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
ROS 2 
  └─ /cmd_vel ──► rmw_zenoh_cpp ──► Zenoh router (or iROSLink) ──► WiFi ──► ESP32
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
#define WIFI_NETWORKS \
    { "home-network",   "home-password"   }, \
    { "office-network", "office-password" }

#define ROUTER_MDNS_HOST "your-router-hostname"  // e.g. "iphone" → connects to iphone.local:7447
```

Add as many `WIFI_NETWORKS` entries as needed — the firmware tries each in order (8 s timeout per network, loops until one connects). The Zenoh router is discovered via mDNS first, then falls back to the WiFi gateway, then `192.168.2.1`.

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


## Boot button (GPIO 0)

Press the onboard BOOT button to toggle the fan between 50% throttle and off — useful for testing without a ROS 2 connection.

## Known limitations

- **Router disconnect not detectable mid-session.** If the Zenoh router goes offline after the session is established, the firmware cannot detect this via the zenoh-pico API (`z_publisher_put` always returns 1 with `Z_FEATURE_BATCHING=1` regardless of connection state; `z_declare_background_transport_events_listener` crashes on ESP32). The library will silently auto-reconnect in client mode (within ≤30 s via lease expiry). The LED "reconnecting" blink pattern will **not** activate during this window. The 5-minute silence watchdog (`ESP.restart()`) remains as last resort.

---

See [INTERNALS.md](INTERNALS.md) for CDR wire format, drive mixing math, and ESC timing details.
