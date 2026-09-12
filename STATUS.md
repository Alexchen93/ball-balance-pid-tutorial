# Current Status

Updated: 2026-09-13 Asia/Taipei

## Completed in the current local working state

- Camera READY calibration now has the three required phases completed: B, C, and D.
- P remains a pause/hold state and is not treated as a completed calibration phase.
- RUN startup keeps compatibility with the Nano `POS` handshake: current firmware should answer valid `POS` with `POS,OK`; the Python side can still tolerate the existing READY telemetry compatibility path when matching fresh coordinates are observed.
- Platform corner order was corrected from the previous TL-BL-BR-TR order to TL-TR-BR-BL.
- The tracked camera config currently records the latest HSV and zero-reference calibration values.
- The launcher now ignores USB serial devices that exist but are not readable/writable by the current user, and reports blocked devices separately.

## Not yet proven on hardware

- The current source/config state is ready to commit, but real PID behavior still needs to be retested after restarting the system.
- Before claiming closed-loop stability, rerun C/D calibration after restart and do single-axis direction tests.
- Do not treat this repo state as completed stable closed-loop control yet.

## Observed issue

- USB/CH340 re-enumeration was observed during testing. It is recorded as a symptom, not as the only confirmed root cause.

## Local files intentionally not committed

- `Camera_Vision/camera_config.json.backup-20260912-225701` is an untracked local backup created during calibration/config changes. It is kept out of the commit unless repository tracking rules are deliberately changed.
