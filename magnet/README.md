# Magnet — autonomous search-and-return robot

A single Pololu 3Pi+ robot that:

1. Beeps, then **calibrates** its line sensors and magnetometer by spinning in place (a
   ~720° / `12.56` rad rotation).
2. Drives a **search sweep** (`setSearch`), using the 5 downward line sensors to stay
   inside the arena and odometry (`Kinematics_c`) to track its pose.
3. Watches the **magnetometer**; when the field magnitude crosses a threshold
   (`isOnMag()`), it stops and beeps ("magnet found").
4. **Returns home** to the (0,0) start using odometry (`setReturn` / `checkReturn`), or
   does so anyway once `searchTime` elapses.

## Files

| File | Role |
|------|------|
| `magnet.ino` | Main program (state machine, calibration, search, return). Originally `final_Bot.ino`. |
| `final_motor_2.h` | `Motors_c` — PWM motor driver (Labsheet 1). |
| `LineSensors.h` | `LineSensors_c` — 5× IR reflectance sensors, ADC + calibration (Labsheet 2). |
| `Magnetometer.h` | `Magnetometer_c` — LIS3MDL over I²C, hard-iron calibration (Labsheet 3). |
| `Kinematics.h` | `Kinematics_c` — differential-drive odometry x/y/θ (Labsheet 4). |
| `Encoders.h` | Quadrature encoder ISRs (provided, do not edit). |

> Dropped from this copy: `extra.h` — a broken/older backup of the sketch's setup (it
> redefines `Max_Sensor` twice and includes a `PID.h`/OLED the final sketch doesn't use).
> It is **not** `#include`d by the sketch, so removing it changes nothing.
> The near-identical `final_h_wo_return/` variant (shorter search time, no return-home,
> slightly different turn PWMs) is also not included — `final_Bot` is the more complete one.

## Status

Runs as a blocking state machine. The fixes below have been **applied** to this copy (each
is also marked with a `// FIX:` comment in the source); they are listed here, roughly by
importance, with the reasoning.

### 1. Magnetometer calibration min/max are initialised inverted  ⚠️ likely functional bug
In `magnet.ino`, `sensor_mag_calibration()`:

```cpp
magnetometer.maximum[n] = 9999.9;    // should be -9999.9
magnetometer.minimum[n] = -9999.9;   // should be  9999.9
```

Because `maximum` starts at +9999.9 and `minimum` at −9999.9, the `if (reading > maximum)`
/ `if (reading < minimum)` updates can **never** fire, so the recorded range stays
±9999.9. The resulting `Scaling = 1/(Range/2)` becomes a tiny (≈0.0001) value, the
calibrated magnitude is driven near zero, and `isOnMag()` (`magnitude > 2`) may **never
trigger** — i.e. the robot might never "find" the magnet. The correct initialisation is
the one already present in `extra.h`:

```cpp
magnetometer.maximum[n] = -9999.9;
magnetometer.minimum[n] =  9999.9;
```

### 2. `millis()` timestamps stored in `unsigned` (16-bit) instead of `unsigned long`
`unsigned pose_ms; unsigned pose_ts;` overflow every ~65 s, corrupting the pose-update
timing. Use `unsigned long`.

### 3. `final_motor_2.h` `setPWM()` never clamps power
The clamp-to-`[0, MAX_PWM]` code was left as TODO comments; it calls
`analogWrite(L_PWM, abs(left_pwr))` directly. Safe here only because all demands are small
(30–56). Add `analogWrite(L_PWM, constrain(abs(left_pwr), 0, MAX_PWM));`. Also `#define FWD
HIGH` and `#define REV HIGH` are both `HIGH` (and unused) — misleading.

### 4. `LineSensors.h` constructor is mis-named
The class is `LineSensors_c` but the constructor is written `LineSensor_c()` (no `s`), so
it is a never-called method rather than a real constructor. Harmless today (init is
explicit) but a latent trap. Rename to `LineSensors_c()`.

### 5. `Magnetometer::initialise()` is declared `bool` but returns nothing
Add `return true;` (or make it `void`) to avoid an undefined return value.

### 6. Style / robustness (non-breaking)
- `wheel_speed()` uses `alpha = 0`, so the filtered speed never leaves 0 — but it isn't
  used for control, so it's dead code; remove or set a real α (e.g. 0.3) if you want it.
- The big `if` in `setSearch()` mixes `&&`/`||` without parentheses; the extra terms are
  redundant. It reduces to `if (isOnLine(2) || isOnLine(3) || isOnLine(4))` and the
  mirrored `if (isOnLine(0) || isOnLine(1))`.
- `setSearch`/`checkReturn` use inner `while(...)` blocking loops, defeating the
  non-blocking `set*/check*` design. Fine for this task, but a fully non-blocking loop
  would be more responsive.
- `isOnMag()` threshold (`> 2`) is a magic number — promote to a `#define` so it's easy to
  retune per environment.
