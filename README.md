# 26K66_CapstanDrive

Code for one capstan-driven joint. A PC drives the ODrive S1 directly over USB, and an MCP server lets an LLM (Claude Code, etc.) operate the joint.

## Hardware

- ODrive S1 (firmware 0.6.11), 24 V supply LRS-350-24, 15 A fuse, 2 Ω brake resistor
- Motor: EaglePower LA8308 KV90 (20 pole pairs). Position comes from the ODrive's onboard encoder
- Reduction: 8:1 from the pinion to the output drum (capstan)
- Connection: PC to ODrive over USB-C. The original Teensy 4.1 + CAN setup is not used for now

## Measured mechanics (2026-11-25)

- **Direction**: + motor turns move the drum **down** (`MOTOR_DIR_UP = -1` in `config.py`)
- **Travel**: about 2.43–2.47 motor turns until the stops are first touched (configured: 2.47). The stops are compliant: pushed hard by hand, the drum goes as far as 2.62 turns
- **Load**: almost only friction (0.3–1 A to move slowly). The drum does not fall when the motor is off
- **Reference**: the ODrive resets its position to 0 at every boot. After a boot, hold the drum on the bottom stop by hand and call `set_bottom_here`. The reference survives the 24 V supply being off as long as USB stays connected

## Settings saved to the ODrive (2026-11-25)

| Setting | Before | After |
|---|---|---|
| `axis0.controller.config.vel_limit` | 26 turns/s | 8 turns/s |
| `axis0.config.motor.current_soft_max` | 20 A | 10 A |
| `axis0.config.motor.torque_constant` | 0.0766 (KV 108) | 0.0919 (KV 90) |

Backups of the configuration before and after the change are in `odrive_config/`. Restore one with `odrivetool restore-config <file>`.

## Files

| File | Contents |
|---|---|
| `config.py` | Mechanics and safety limits (gear ratio, direction, travel, speed, current, gains, reference checks) |
| `joint.py` | Control and safety core. Angles are drum degrees above the bottom stop |
| `server.py` | MCP server (stdio) |
| `measure_range.py` | Measures direction and travel while you move the drum by hand. The motor does not move |
| `tune_gains.py` | Gain tuning. **Moves the motor**, so it requires `--i-am-watching` |
| `measure_load.py` | Measures the current needed to move at each position (friction and load). **Moves the motor** |
| `tests/sim_test.py` | Hardware-free simulation test of `joint.py` |
| `odrive_config/` | ODrive configuration backups |

## Setup

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# USB access (Linux, once)
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/91-odrive.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Register with Claude Code
claude mcp add capstan -- "$PWD/.venv/bin/python" "$PWD/server.py"

# Test without hardware
.venv/bin/python tests/sim_test.py
```

## MCP tools

Angles are drum degrees above the bottom stop, up = +. Targets must be within 5.4–105.8°.

| Tool | What it does |
|---|---|
| `status` | Reads the state (nothing moves) |
| `set_bottom_here` | Sets the reference at the position where a human holds the drum on the bottom stop. Also returns the shift from the previous reference (a large one suggests the cable slipped) |
| `check_top_here` | Call it while a human holds the drum on the top stop: compares the measured travel with the configured one (2.47 turns) and gives a verdict. Stores nothing and moves nothing |
| `enable` / `disable` | Motor on (holds where it is) / off (goes limp at once) |
| `move_to` / `move_by` | Moves to an absolute / relative angle, waits until it settles and returns the final error |
| `go_home` | Returns to the park position near the bottom (6°) and turns the motor off (0° is the bottom stop itself) |
| `metrics` | Figures since the server started (moves, final error mean / 95th percentile / max, share within 0.1°, move times, peak current and speed, total travel). Nothing moves |
| `clear_errors` | Clears the ODrive's error flags |
| `release_usb` | Releases the USB connection (for the Web GUI or `odrivetool`) |

Typical flow: `status` → (if there is no reference, a human holds the drum on the bottom stop, then) `set_bottom_here` → `enable` → `move_to` → `go_home`

Logging: while the motor is on, angle, target, velocity and current (about 50 Hz) and every operation are written with timestamps to `logs/telemetry-YYYYMMDD.jsonl` (for graphs and videos; not tracked by git).

## Safety design

- Speed is capped at 45°/s and current at 6 A. Both are set at runtime only and never saved to the ODrive. The 8 turns/s and 10 A saved on the ODrive are the last line of defense
- Out-of-range targets are refused before anything moves
- A supervisor thread (about 50 Hz) turns the motor off as soon as it sees an ODrive error, a drop out of closed loop, a position outside the travel, over-speed, or drift right after enabling
- **Detecting contact**: if the median current over 0.3 s reaches 3.5 A, the motor is turned off. Normal moves need at most 2.54 A, so it never keeps pushing
- ODrive watchdog: if the server stops, the motor turns off within 0.5 s (0.48 s measured on the hardware)
- The reference becomes invalid at every ODrive boot, and nothing moves until it is set again
- The top stop is computed (bottom + 2.47 turns of travel). After re-tensioning the cable or replacing parts, hold the drum on the top stop by hand and check it with `check_top_here`. If the gap between the top stop and the allowed maximum is less than 2.7° (half the margin), it asks for a re-measurement; if the top stop is below the allowed maximum, it returns UNSAFE
- To automate referencing, fit limit switches at the stops and use the ODrive's built-in homing (do not drive the motor into the stops to find them)

## Notes

- While the MCP server holds the USB connection, the Web GUI cannot connect (free it with `release_usb`)
- The original Teensy sample code assumes the opposite direction (+ = up), so do not use it: it would drive into the bottom stop
- Whenever the motor moves, someone must be watching and ready to cut the 24 V supply

## Status (2026-11-25)

- Verified on the hardware: holding, watchdog stop, moves, refusal of out-of-range commands
- Gains tuned (`tune_gains.py`): position 35 / velocity 0.5 / integrator 2.5. The final position error went from 0.4–0.5° with the ODrive defaults to within 0.03°
- The bottom reference is set by hand (`set_bottom_here`)
