# MPPT Firmware Change Log

## Unreleased — selectable charge algorithm + per-board config robustness

### New features

**1. Selectable charge algorithm**
- Added `MpptAlgo` enum (`PerturbObserve`, `SafeCharge`), a `volatile activeAlgo`
  selector, and `setMpptAlgo()` (resets PIDs on switch). `mpptUpdate()` dispatches
  to the chosen algorithm.
- Compile-time default via `DEFAULT_MPPT_ALGO` in `const.h` — **defaults to the
  existing P&O**, so on-car behavior is unchanged until you opt in. Runtime switch
  also possible.
- Files: `include/mppt.h`, `src/mppt.cpp`, `include/const.h`.

**2. Algorithm 2 — Incremental Conductance MPPT with CC–CV battery limiting**
  (`src/mppt.cpp`)
- Tracks panel MPP via incremental conductance (`dI/dV` vs `-I/V`, with deadband).
- **CV taper** at `battVolt >= 121.2V` (4.18 V/cell), **CC backoff** above
  `min(BMS limit, 34A)`, **hard stop** (converters off) if boost disabled or
  `battVolt >= 121.8V` (4.20 V/cell). Limits shed power by pushing array voltage
  toward Voc.
- No temperature cutoff (charge current is small vs. cell rating).

**3. Battery-safety constants** (`include/const.h`)
- New block derived from 29S10P INR21700-50S + 40A fuse: `V_BATT_MAX`, `V_BATT_CV`,
  `I_CHG_MAX_FUSE` (85% of fuse), plus tunables `INCCOND_STEP`, `INCCOND_DEADBAND`,
  `CC_CV_BACKOFF_STEP`. Centralized for easy tuning.

### Per-board string configuration (2-board / 5-string setup)

**4. `NUM_ARRAYS` is now the only knob to change between boards.**
- Previously, setting `NUM_ARRAYS` to 2 would **fail to compile** (hardcoded
  3-element initializers). Fixed:
  - `arrayPins` entries 2 & 3 now `#if NUM_ARRAYS >= 2/3` guarded
    (`src/IOManagement.cpp`).
  - `targetVoltage`/`targetVoltage_C` zero-filled, seeded to `INIT_VOLT` in
    `initMPPT()` (`src/mppt.cpp`).
  - P&O statics (`stepSize`, `stepSize_C`) seeded via loop instead of brace lists
    (`src/mppt.cpp`).
- Added a prominent PER-BOARD CONFIG banner at `NUM_ARRAYS` (`include/const.h`).
- Verified by building at both `NUM_ARRAYS=3` and `=2` — both succeed
  (RAM 4.1% -> 3.8%, confirming arrays resize).

### Bug fixes

**5.** P&O constant-current branch used `arrayData[0].voltage` for every array
  -> now `arrayData[i]` (`src/mppt.cpp`).
**6.** SOC-based mode switch in `updateData()` now only runs under P&O — otherwise
  it forced the converters off underneath the safe-charge algo
  (`src/IOManagement.cpp`).
**7.** `DEBUG_PRINT==2` read `arrayData[2]` unconditionally (out-of-bounds on a
  2-string board) and passed the `targetVoltage` array where a float was expected
  (UB). Debug printer and CSV header now loop over `NUM_ARRAYS`; `DEBUG_PRINT==1`
  target-voltage print fixed to `targetVoltage[0]` (`src/main.cpp`).

### Open items (NOT changed — assigned to follow-up owner)

- **CAN ID collision between the two boards.** Both boards transmit on the same IDs
  (0x400–0x416). On a shared bus the 2-string and 3-string boards overwrite each
  other, and `format.json` only defines 3 strings total (not 5). Likely needs a
  per-board CAN ID offset and a 5-string data-format spec. Left alone pending bus
  architecture decision.
- Stale `// offset by 3*i` comment in `canMppt.cpp` (code correctly uses `5*i`).

### Testing & CI

**8. Native unit-test harness** (`test/test_mppt_core/`, `[env:native]`)
- Extracted the pure decision logic into `include/mppt_core.h` +
  `src/mppt_core.cpp` (no globals/timers/hardware; includes only `const.h`).
  `mppt.cpp` now calls these, so the tested code is the code that runs on the car.
- 26 Unity tests run a desktop PV-parabola + battery simulation covering the
  SafeCharge safety envelope: hard stop, CV/CC thresholds, current-limit
  selection (incl. zero-BMS gating and fuse-spec check), incremental-conductance
  direction (incl. the flat-voltage branch), the limiting override and CC/CV
  power-shedding, MPP convergence (asserted on captured power), and that pack
  voltage/charge current never breach their caps.
- Run locally with `pio test -e native`. **26/26 passing.**
- Note: this is a *logic/decision* simulation only — it does **not** model the
  converter/PID/PWM timing, so it is not a substitute for a bench test.

**9. GitHub Actions CI** (`.github/workflows/native-tests.yml`)
- Runs `pio test -e native` automatically on every push and pull request.

**Dependencies — what other people need.**
- *To run the native tests* they need only a host C++ compiler (`gcc`/`g++`) and
  PlatformIO. On Linux/macOS gcc is already present; on Windows install a host
  GCC (e.g. WinLibs via `winget install BrechtSanders.WinLibs.POSIX.UCRT`) and
  reopen the terminal. The CI runner has gcc preinstalled, so no extra setup.
- The native tests do **not** need the ARM toolchain, the Arduino framework, or
  the `embedded-pio` submodule (they compile only `mppt_core.cpp` + `const.h`).
- *To build/flash the board firmware* the `embedded-pio` submodule is still
  required (see Notes).

### Notes

- `embedded-pio` submodule must be initialized to build the **firmware**
  (`git submodule update --init`). Not needed for the native tests.
- Algorithm 2 is **not bench-tested** — constants/steps need hardware validation
  before charging a real pack.
