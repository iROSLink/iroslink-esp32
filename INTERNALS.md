# Internals

Reference for contributors or anyone debugging the firmware.

## Zenoh key expression

rmw_zenoh_cpp maps `/cmd_vel` to:
```
0/cmd_vel/geometry_msgs::msg::dds_::Twist_/<type_hash>
```
Firmware subscribes to `0/cmd_vel/geometry_msgs::msg::dds_::Twist_/**` to match any type hash. Same pattern applies to `/suction_fan/speed` and `/battery/voltage`.

## CDR decoding

`geometry_msgs/msg/Twist` — CDR XCDR1 (little-endian), 52 bytes:

| Offset | Size | Field      |
|--------|------|------------|
| 0–3    | 4    | CDR header |
| 4–11   | 8    | linear.x   |
| 12–19  | 8    | linear.y   |
| 20–27  | 8    | linear.z   |
| 28–35  | 8    | angular.x  |
| 36–43  | 8    | angular.y  |
| 44–51  | 8    | angular.z  |

`std_msgs/msg/Float32` (fan throttle / battery voltage) — 8 bytes:

| Offset | Size | Field          |
|--------|------|----------------|
| 0–3    | 4    | CDR header     |
| 4–7    | 4    | data (float32) |

## Differential drive mixing

```
left_speed  = linear_x × 510 - angular_z × 255
right_speed = linear_x × 510 + angular_z × 255
```

Clamped to [-200, 200] (8-bit PWM). Scaled so teleop_twist_keyboard defaults (0.5 m/s linear, 1.0 rad/s angular) map near full PWM. Adjust `LINEAR_SCALE` and `ANGULAR_SCALE` in `drive()` to tune.

## ESC pulse timing

Standard RC ESC protocol: 50 Hz, 1050–2100 µs. On boot the firmware holds 1050 µs for 2 s to arm. Throttle maps Float32 [0.0, 1.0] → [1050, 2100] µs via 16-bit LEDC (1 count ≈ 0.305 µs).

## Battery ADC

Resistor divider R1=100 kΩ, R2=27 kΩ → V_adc = V_bat × 27/127. ADC attenuation set to 11 dB (0–3.3 V range). Voltage published every 10 s via a dedicated FreeRTOS task to avoid blocking the Zenoh session mutex in `loop()`.

## Zenoh session watchdog

If no `/cmd_vel` message is received for 60 s after the first one, the firmware calls `ESP.restart()`. This recovers from stalled Zenoh reconnects where locked mutexes prevent normal operation.
