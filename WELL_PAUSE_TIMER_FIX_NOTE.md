# Well pause timer bug fix

## Bug
`wellPauseStart` was being reset to `0` on ESP32 reboot and on web action `reset_pause`, while start permission logic still used elapsed-time arithmetic (`now - wellPauseStart >= pauseMs`).

That caused an incorrect implicit pause from boot time: the well could not start until uptime reached `pauseMs` (default ~10 minutes), even though pause had just been reset.

## Fix
- Unified pause semantics: `wellPauseStart == 0` now means **no active pause countdown**.
- Added a shared helper (`isWellPauseCountdownActive`) and used it in:
  - pause active reporting,
  - pause reset behavior,
  - WAIT-mode start permission logic.
- WAIT-mode start now goes through `canWellStartInWaitMode(...)`, which only blocks start when an actual active pause countdown exists.

## Result
After ESP32 reboot or manual `reset_pause`, pause remaining becomes `0` immediately and the well can start right away if all other protections/conditions allow it (INIT delay, telemetry validity, `needPump`, manual force-off, and active alarms/blocks).
