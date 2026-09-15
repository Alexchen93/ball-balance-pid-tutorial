# Current Status

Updated: 2026-09-13 Asia/Taipei

## Completed in the current local working state

- Camera READY calibration now has the three required phases completed: B, C, and D.
- P remains a pause/hold state and is not treated as a completed calibration phase.
- RUN startup keeps compatibility with the Nano `POS` handshake: current firmware should answer valid `POS` with `POS,OK`; the Python side can still tolerate the existing READY telemetry compatibility path when matching fresh coordinates are observed.
- Camera Vision C calibration requires TL-TR-BR-BL click order, validates the four-point geometry, and rejects invalid or misordered geometry without overwriting the previous valid config.
- Stored platform corner configs are checked on load/apply: invalid or misordered geometry blocks homography and requires C recalibration.
- `camera_config.json` is a tracked editable default; live calibration changes should be reviewed before committing.
- The launcher now ignores USB serial devices that exist but are not readable/writable by the current user, and reports blocked devices separately.

## Current diagnostic focus

- New Nano normalized P-only mapping is implemented but not hardware-verified: `e_norm = clamp((target_mm - ball_mm) / POSITION_LIMIT_AXIS_MM, -1, +1)`, `u_norm = clamp(Kp*e_norm, -1, +1)`, `tilt_deg = u_norm * MAX_PLATFORM_TILT_AXIS_DEG`.
- Previous P-only（Kp=.10, Ki=Kd=0）實機方向響應成功；that result does not validate the new Kp=1/max-tilt mapping.
- Nano firmware is the single PID parameter source/control authority. Camera Vision does not save, send, or override PID settings.
- Camera Vision restart or `camera_config.json` changes do not change Nano PID or max tilt. Any PID/max-tilt change requires editing and reflashing the `.ino`.
- Continue with the Nano single-authority target: firmware remains the only PID parameter/control source; Camera Vision remains vision/serial state only.

## Not yet proven on hardware

- Do not claim closed-loop stability yet; the current result only confirms P-only direction response.
- Before claiming completed stable control, reflash the updated `.ino`, rerun C/D calibration after restart, and do single-axis tests under the Nano-owned normalized PID constants.
- Keep D/parameter tuning and mapping checks as follow-up candidates if oscillation returns.

## Observed issue

- USB/CH340 re-enumeration was observed during testing. It is recorded as a symptom, not as the only confirmed root cause.

## Local files intentionally not committed

- `Camera_Vision/camera_config.json.backup-20260912-225701` is an untracked local backup created during calibration/config changes. It is kept out of the commit unless repository tracking rules are deliberately changed.
