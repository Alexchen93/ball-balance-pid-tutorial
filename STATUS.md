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

- P-only（Kp=.10, Ki=Kd=0）實機方向響應成功；初步支持先前震盪與 D/參數或映射相關，但尚未宣稱閉迴路穩定完成。
- Nano firmware is the single PID parameter source/control authority. Camera Vision does not save, send, or override PID settings.
- Camera Vision restart or `camera_config.json` changes do not change Nano PID. Any PID change requires editing and reflashing the `.ino`.
- Continue with the Nano single-authority target: firmware remains the only PID parameter/control source; Camera Vision remains vision/serial state only.

## Not yet proven on hardware

- Do not claim closed-loop stability yet; the current result only confirms P-only direction response.
- Before claiming completed stable control, rerun C/D calibration after restart and do single-axis tests under the Nano-owned PID constants.
- Keep D/parameter tuning and mapping checks as follow-up candidates if oscillation returns.

## Observed issue

- USB/CH340 re-enumeration was observed during testing. It is recorded as a symptom, not as the only confirmed root cause.

## Local files intentionally not committed

- `Camera_Vision/camera_config.json.backup-20260912-225701` is an untracked local backup created during calibration/config changes. It is kept out of the commit unless repository tracking rules are deliberately changed.
