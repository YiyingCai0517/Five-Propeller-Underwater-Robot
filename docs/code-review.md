# Code review findings

This is a static review of the checked-in firmware, PC tool, and ROS export. It records risks without silently changing the behavior of a tested prototype. Severity reflects potential impact during physical operation.

## High priority

| Area | Finding | Impact | Recommended correction |
| --- | --- | --- | --- |
| Depth conversion | `MS5837_30BA_data.Pressure` is already stored in mbar after the final `/ 10`, but `MS5837_GetDepth()` applies the coefficient for 0.1 mbar. The reported depth is therefore approximately 10× too small. | Incorrect setpoint tracking and unsafe depth control | Use approximately `delta_mbar * 0.0101972f` for fresh water, then calibrate density and atmospheric zero with a known depth |
| Command parser | Function payload lengths are not required to be exact, and float fields are not checked with `isfinite()` or physical bounds. | Malformed, `NaN`, or extreme commands can propagate into state, angle normalization, timing, and PWM logic | Require exact lengths and validate depth, surge, yaw, and edge time before queueing |
| Link safety | There is no command-age or LoRa-loss watchdog in active modes. | The last motion command can continue after the operator link is lost | Timestamp each valid command and force neutral/idle after a short, tested timeout |
| IMU parser | TLV parsing reads `payload->data_len` before proving that two TLV header bytes remain; malformed lengths can underflow `payload_len` and advance outside the frame. | Out-of-bounds reads or stale/invalid attitude data | Check remaining bytes before every header and payload access; reject the entire frame on inconsistency |

## Medium priority

| Area | Finding | Impact / next step |
| --- | --- | --- |
| DMA length | `imu_data_len` is `uint8_t` although the buffer is 256 bytes. A full DMA buffer produces `256`, which wraps to zero. | Change the variable and related calculations to `uint16_t` or `size_t` |
| Semaphore initialization | Both binary semaphores are created with an initial count of 1. | Control and communication tasks can run once before real data arrives; initialize them to 0 |
| MS5837 errors | I²C return codes and repeated CRC failure are not propagated. Transactions use 10 s timeouts. | A failed sensor can block the depth task or leave stale values; use bounded timeouts, return status, and declare sensor data invalid |
| MS5837 OSR names | Constants labelled `OSR_8192` use commands `0x44` and `0x54`, which correspond to a lower oversampling setting in the sensor command map. | Rename or correct the commands and use the matching conversion delay |
| PID transitions | Most mode changes do not reset selected integrators or derivative history. | Carry-over can cause a transient when a new target or mode is selected; use explicit bumpless transfer/reset rules |
| PID robustness | Derivative is taken directly from error without filtering, and integration continues while output is saturated. | Noise amplification and windup; add derivative filtering and conditional/back-calculation anti-windup |
| Mixer saturation | All five thrusters are normalized as one group. | Horizontal saturation can reduce vertical authority and vice versa; use group-aware limits or constrained allocation |
| Speed estimate | Accelerometer integration with fixed per-sample decay depends on update rate and accumulates bias. | Treat as indicative telemetry; use time-constant-based decay and a suitable velocity sensor/fusion method for navigation |
| Protocol evolution | No version, sequence number, acknowledgement, or duplicate detection. | Hard to diagnose packet loss and maintain compatibility; add these fields in a versioned frame |

## ROS / URDF findings

- The original `base_link` inertial origin places one coordinate about 1.27 m away from a body whose local mesh extent is much smaller, indicating a likely coordinate swap in the SolidWorks export.
- The original `rfw` link has zero mass and zero inertia, which is invalid for physics simulation.
- Visual meshes are also used as collision meshes and are unnecessarily detailed for real-time collision checking.
- `display.launch` references a missing `urdf.rviz` file.
- The package has no underwater dynamics, buoyancy, drag, thruster, current, or sensor plugins.

`simulatedrobot_fixed.urdf` and the `*_fixed.launch` files preserve the exported package while correcting the two inertial defects for visualization. They still do not make the package a validated underwater physics model.

## Validation limits

The review environment did not have the target robot, calibrated sensors, ESCs, or a water tank. Firmware behavior must be confirmed on a bench with propellers removed before any water test. See [Safety and known limitations](safety.md).
