# PC LoRa ground station

`lora_pc.py` is an interactive serial console for command transmission, telemetry display, and optional CSV logging.

## Install

Python 3.10 or later is recommended.

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r tools\pc\requirements.txt
```

On macOS/Linux, activate with `source .venv/bin/activate`.

## Find the serial port

Windows PowerShell:

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Description
```

Linux:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

## Run

From the repository root:

```powershell
python tools\pc\lora_pc.py COM6 9600
```

Replace `COM6` and `9600` with the actual device and configured baud rate.

## Commands

| Command | Example | Meaning |
| --- | --- | --- |
| `hover <depth>` | `hover 0.15` | Hold 0.15 m depth |
| `move <surge>` | `move 0.2` | Apply normalized forward thrust |
| `stop` | `stop` | Return to idle/neutral |
| `turn <yaw>` | `turn 90` | Turn to an absolute yaw in degrees |
| `square <seconds> <surge>` | `square 5 0.2` | Timed four-leg demonstration |
| `log start` / `log stop` | `log start` | Write telemetry CSV in the current directory |
| `help` | `help` | Show help |
| `quit` | `quit` | Close the serial port |

The E22 fixed-transmission address and channel are constants near the top of the script. They must match the radio configuration and firmware. See `docs/protocol.md`.

## Safety

The console currently validates only some surge limits. The firmware does not yet have a command-loss watchdog. Keep an independent physical power disconnect available, send `stop` before closing the tool, and do not rely on the radio link as the only emergency stop.
