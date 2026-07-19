# LoRa command and telemetry protocol

The application protocol uses little-endian IEEE-754 single-precision floats.

## Application frame

```text
AA 55 | function | payload_length | payload... | checksum
```

The checksum is the low eight bits of the sum of every byte from `AA` through the final payload byte.

When the E22 module is configured for fixed-point transmission, the PC prepends a three-byte destination header:

```text
address_high | address_low | channel | application_frame
```

The checked-in PC tool sends to STM32 address `0x0001` on channel `0x17`. Firmware telemetry targets PC address `0x0000` on the same channel. These values must match the actual E22 configuration.

## PC-to-robot commands

| Function | Name | Payload | Intended constraints |
| ---: | --- | --- | --- |
| `0x01` | `HOVER` | `float depth_m` | Non-negative and within the tested tank limit |
| `0x02` | `MOVE` | `float surge` | `[-1, 1]` |
| `0x03` | `STOP` | none | Immediate transition to idle |
| `0x04` | `TURN` | `float target_yaw_deg` | Finite angle; normalize before use |
| `0x05` | `SQUARE` | `float edge_time_s`, `float surge` | Positive bounded time; surge in `[-1, 1]` |

Example `STOP` application frame:

```text
AA 55 03 00 02
```

## Robot-to-PC telemetry

Telemetry function `0x80` carries a packed 33-byte payload:

| Offset | Type | Field | Unit |
| ---: | --- | --- | --- |
| 0 | `uint8` | mode | enum |
| 1 | `float` | current depth | m |
| 5 | `float` | yaw | deg |
| 9 | `float` | pitch | deg |
| 13 | `float` | roll | deg |
| 17 | `float` | target depth | m |
| 21 | `float` | target yaw | deg |
| 25 | `float` | estimated forward speed | m/s |
| 29 | `float` | requested surge | normalized |

Nominal telemetry interval is 500 ms.

## Parser limitations in the current firmware

The current protocol has no version, sequence number, acknowledgement, replay protection, or link watchdog. Firmware accepts payloads that are longer than required and does not reject non-finite floats or out-of-range depth/time/yaw values. The receive-length expression also uses an eight-bit result, which is unsafe for the 256-byte IMU DMA buffer.

Before extending the protocol:

- require the exact payload length for each function;
- validate `isfinite()` and physical bounds before queueing a command;
- use a 16-bit receive length for 256-byte buffers;
- add a monotonically increasing sequence number and explicit protocol version;
- add an independent command-age watchdog that returns the vehicle to neutral;
- document byte order and compatibility changes in this file.
