# Changelog

All notable fixes applied to the as-submitted Pololu 3Pi+ code when assembling this
`final/` version. Each change is also marked inline with a `// FIX:` comment.

## Bug fixes — magnet

- **Magnetometer calibration min/max initialised inverted** (`magnet.ino`,
  `sensor_mag_calibration`). `maximum` was seeded to `+9999.9` and `minimum` to `-9999.9`,
  so the running max/min could never update — the recorded range stayed at ±9999.9,
  scaling collapsed, and `isOnMag()` could never trigger. Now seeds `maximum = -9999.9`,
  `minimum = 9999.9` (matching the correct logic in the old `extra.h` backup).
- **`millis()` timestamps stored in `unsigned` (16-bit)** (`magnet.ino`: `pose_ms`,
  `pose_ts`). Changed to `unsigned long` to avoid ~65 s overflow of the pose-update timer.
- **`Motors_c::setPWM()` never clamped power** (`final_motor_2.h`). Now
  `constrain(abs(pwr), 0, MAX_PWM)` so out-of-range demands can't wrap past 255.
- **`#define REV HIGH`** (`final_motor_2.h`) was identical to `FWD`. Set to `LOW`.
- **`LineSensors_c` constructor mistyped as `LineSensor_c`** (`LineSensors.h`). Renamed so
  it is an actual constructor.
- **`Magnetometer_c::initialise()` declared `bool` but returned nothing** (`Magnetometer.h`).
  Added `return true;`.

## Bug fixes — leader-follower (applied to both `leader/` and `follower/`)

- **PID derivative term was dead code** (`PID.h::update`). `last_error` was overwritten with
  the current error before `diff_error` was computed, forcing the D term to 0 regardless of
  `Kd`. `last_error` is now updated at the end of `update()`, so the D term works.
  - Consequence: `Kd` is now live. The system was tuned with D inactive, so the sketches set
    `Kd = 0` (`leader.ino`, `follower.ino`) to reproduce the demonstrated PI behaviour. The
    previous value of `200` was never actually applied; re-tune on hardware to enable D.
- **`map()` called on a float** (`follower.ino`). `map(AngC, -1, 1, 0.0, 5.0)` used
  integer-only arithmetic, truncating `AngC` to -1/0/1 and losing all proportional
  resolution. Replaced with a floating-point map `(AngC + 1) * 2.5` → `[0, 5]`.
- **`Motors_c::setPWM()` never clamped power** (`Motors.h`). Same clamp fix as magnet.
- **`LineSensors_c` constructor mistyped as `LineSensor_c`** (`LineSensors.h`). Renamed.

## Not changed (recommended improvements — see project READMEs)

Left as-is to avoid untested behavioural changes; documented for future work:

- Leader IR emitter is starved by blocking `delay(6000)` in `leader.ino::loop()` — the IR
  carrier isn't toggled while the robot is moving. Should use a non-blocking timer / ISR.
- `leader.ino` and `follower.ino` duplicate the entire helper/driver layer — factor shared
  code into a common header.
- Several `set*/check*` functions contain unreachable code after `return`, and `set_beep()`
  is "non-blocking" in name only (`delay()`-based).
- magnet's `setSearch()` blocking `while` loops defeat the non-blocking state-machine design;
  the large `&&`/`||` sensor condition can be simplified.
- Pulse period measured with `millis()` (1 ms) is coarse for ~100 Hz signals — `micros()`
  would give finer frequency resolution.
