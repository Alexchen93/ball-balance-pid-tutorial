# Current Status

Updated: 2026-09-13 Asia/Taipei

## Completed in the current local working state

- Camera READY calibration now has the three required phases completed: B, C, and D.
- P remains a pause/hold state and is not treated as a completed calibration phase.
- RUN startup keeps compatibility with the Nano `POS` handshake: current firmware should answer valid `POS` with `POS,OK`; the Python side can still tolerate the existing READY telemetry compatibility path when matching fresh coordinates are observed.
- Platform corner order was corrected from the previous TL-BL-BR-TR order to TL-TR-BR-BL.
- The tracked camera config currently records the latest HSV and zero-reference calibration values.
- The launcher now ignores USB serial devices that exist but are not readable/writable by the current user, and reports blocked devices separately.

## Current diagnostic focus

- P-only diagnostic is staged in Nano firmware and requires reflashing `Arduino_Nano_Ball_Balance.ino`: X `Kp=0.10`, Y `Kp=0.10`; both axes use `Ki=0.00`, `Kd=0.00`.
- Nano firmware is the single PID parameter source/control authority. Camera Vision does not save, send, or override PID settings.
- Camera Vision restart or `camera_config.json` changes do not change Nano PID. Any PID change requires editing and reflashing the `.ino`.
- Hardware question to verify: when the ball is held at a fixed edge error, does P-only control create a stable one-direction tilt instead of oscillating or reversing?

## Not yet proven on hardware

- The P-only diagnostic still needs to be retested after restarting Camera Vision; do not claim the PID issue is fixed yet.
- Before claiming closed-loop stability, rerun C/D calibration after restart and do single-axis direction tests.
- Do not treat this repo state as completed stable closed-loop control yet.

## Observed issue

- USB/CH340 re-enumeration was observed during testing. It is recorded as a symptom, not as the only confirmed root cause.

## Local files intentionally not committed

- `Camera_Vision/camera_config.json.backup-20260912-225701` is an untracked local backup created during calibration/config changes. It is kept out of the commit unless repository tracking rules are deliberately changed.
