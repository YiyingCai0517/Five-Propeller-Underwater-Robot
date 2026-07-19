# Hardware and budget snapshot

This page is transcribed from the course report's project budget. It documents the prototype configuration; it is not a current purchasing recommendation, price quote, or complete wiring bill of materials.

![Prototype during final assembly](assets/prototype-assembly.jpg)

## Installed system

| Subsystem | Selected hardware | Role |
| --- | --- | --- |
| Main controller | STM32F407 board | FreeRTOS tasks, control, communication, PWM |
| Attitude sensor | H30 IMU | Yaw, pitch, roll, and acceleration over UART6 |
| Depth sensor | MS5837-B30 / MS5837-30BA | Pressure and depth feedback over I²C1 |
| Radio | E22-400T30D LoRa | Commands and telemetry over UART3 |
| Horizontal propulsion | 2 × Viocean T60 installed | Surge and differential yaw |
| Vertical propulsion | 3 × vertical thruster | Heave, pitch, and roll |
| Battery | Custom pack using Samsung 21700-50S cells | Reported 5000 mAh / approximately 111 Wh |
| DC-DC | 5 V / 3 A module | Low-voltage electronics supply |
| Pressure housing | 130 mm × 250 mm transparent cylinder | Dry electronics enclosure |
| Main switch | M22 waterproof switch | External power isolation |

The report describes three horizontal T60 units in the purchase list while the mechanical and control design installs two. The third purchased unit is therefore treated here as a spare; verify the actual inventory before ordering or assembling.

## Historical budget

| Item | Model / specification | Unit price (CNY) | Quantity | Subtotal (CNY) |
| --- | --- | ---: | ---: | ---: |
| Pressure housing | 130 mm × 250 mm | 370.0 | 1 | 370.0 |
| Horizontal thruster | Viocean T60 | 255.0 | 3 | 765.0 |
| Vertical thruster | Not recorded | 55.0 | 3 | 165.0 |
| Vent / cable gland bolt | M10 | 20.0 | 9 | 180.0 |
| Main controller | STM32F407 | 87.0 | 1 | 87.0 |
| LoRa module | E22-400T30D | 38.8 | 1 | 38.8 |
| DC-DC converter | 5 V / 3 A | 22.7 | 1 | 22.7 |
| Battery | Samsung 21700-50S pack | 209.5 | 1 | 209.5 |
| Attitude sensor | H30 | 254.0 | 1 | 254.0 |
| Wireless debugger | Not recorded | 95.8 | 1 | 95.8 |
| Depth sensor | MS5837-B30 | 209.0 | 1 | 209.0 |
| Waterproof switch | M22 | 24.9 | 1 | 24.9 |
| Self-tapping screws | M2.3 × 18 | — | 100 | 3.3 |
| Bolts and nuts | M3 × 20 / M5 × 30 | — | 130 | 16.3 |
| **Total** |  |  |  | **2441.3** |

Prices are historical values copied from the report and may omit shipping, fabrication, consumables, chargers, test equipment, replacement parts, and safety equipment.

## Electrical information still needed

The repository does not yet contain a reviewed schematic, connector pinout, fuse selection, maximum-current calculation, battery protection specification, or waterproof connector wiring table. Add those before treating this as a reproducible hardware build.
