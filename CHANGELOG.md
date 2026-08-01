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

### Notes

- `embedded-pio` submodule must be initialized to build
  (`git submodule update --init`).
- Algorithm 2 is **not bench-tested** — constants/steps need hardware validation
  before charging a real pack.
