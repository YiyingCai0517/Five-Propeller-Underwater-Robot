# Contributing

Thank you for helping improve the Five-Propeller Underwater Robot project.

## Before opening a change

1. Create a focused branch from `main`.
2. Keep firmware, PC tooling, mechanical files, simulation, and documentation in their existing top-level directories.
3. Do not commit generated build output, IDE metadata, caches, telemetry logs, or replacement ZIP archives.
4. For control or protocol changes, update the relevant file under `docs/` in the same pull request.
5. Treat motor activation, depth control, and communication failsafes as safety-sensitive changes.

## Validation

Run the checks that apply to your change:

```bash
python -m py_compile tools/pc/lora_pc.py
python simulation/python/scripts/smoke_test.py
cmake --preset Debug -S firmware
cmake --build --preset Debug
```

Firmware checks require Ninja and the Arm GNU Toolchain. If hardware or toolchain access is unavailable, state that clearly in the pull request.

For mechanical or URDF changes, include:

- units and coordinate-frame conventions;
- mass, center of mass, and center of buoyancy assumptions;
- thruster position, axis, positive direction, and ESC channel;
- a screenshot or rendered preview when geometry changes.

For simulation changes, state which parameters are measured, CAD-derived, or assumed. Do not present idealized numerical errors as physical test accuracy.

## Pull requests

Describe what changed, why it changed, user or developer impact, validation performed, and any known limitation. Prefer small reviewable commits over large archive replacements.
