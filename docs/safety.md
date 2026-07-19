# Safety and known limitations

This is a course prototype, not a certified vehicle. Five exposed propellers, a high-energy battery, water, pressure seals, and remote commands create mechanical, electrical, and operational hazards.

## Do not use closed-loop depth control yet

The current pressure-to-depth coefficient appears approximately 10× too small. Correct it and validate the complete sensor path at several known depths before enabling `HOVER`, `AUTO`, `TURN`, or `SQUARE` in water.

The firmware also lacks a command-loss watchdog. An active mode can continue using its last target after the LoRa link disappears. Test and enable a neutral fail-safe before untethered operation.

## Bench checklist

- Remove propellers or mechanically isolate every thruster before firmware and direction tests.
- Use a current-limited supply for first power-up.
- Confirm 1500 µs neutral and each ESC's actual accepted range.
- Confirm motor indices, physical positions, positive directions, and yaw/pitch/roll signs one channel at a time.
- Verify the physical emergency power disconnect is accessible to a second person.
- Reject non-finite and out-of-range commands before motor output.
- Verify IMU timeout, command timeout, sensor-error handling, and reboot behavior.
- Keep strong motor power wiring separated from sensor and communication wiring.

## Pressure housing checklist

- Inspect O-rings, grooves, cable glands, threaded holes, windows, and printed parts before every test.
- Perform a dry vacuum-hold or equivalent leak test before adding electronics.
- Perform initial immersion with a dummy load or water indicator, not the live battery and controller.
- Increase depth and duration gradually; record leakage and deformation.
- Do not claim a depth rating until the complete assembled housing has been pressure tested with a documented safety factor.

## Water-test checklist

- Use a tether or physical recovery line even if control is wireless.
- Keep people clear of every propeller and never hold the vehicle while propulsion is enabled.
- Begin with low output and a hard software depth/output limit.
- Have one operator control the vehicle and another control the physical power disconnect.
- Stop immediately on leakage, unexpected motion, radio loss, sensor invalidity, battery heating, or unusual current.
- Treat LoRa underwater performance as range-limited; complete submersion can severely attenuate radio signals.

## Battery

The report describes a custom Samsung 21700-50S-based pack of approximately 111 Wh. The repository does not document the cell configuration, BMS, fusing, waterproofing, charger, current rating, or fault protection. Those details must be reviewed independently by a qualified person before reproduction.

## Software status

Known software and model issues are tracked in [Code review findings](code-review.md). Numerical simulation is not proof of physical stability, waterproofing, pressure tolerance, radio range, or safe autonomous behavior.
