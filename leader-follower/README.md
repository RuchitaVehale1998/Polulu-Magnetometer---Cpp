# Leader–Follower — two-robot IR tracking system

Two Pololu 3Pi+ robots. The **leader** emits infrared; the **follower** uses its two
forward QTR-RC reflectance sensors (plus a TSOP IR receiver in this final version) to
estimate distance and angle to the leader and maintain alignment using a PID speed
controller with dynamic speed and statistical (mean / variance / std-dev) error
correction.

This is the work documented in the project report *"Data Transmission and Correction using
QTR-RC Reflectance Sensors in Pololu 3Pi+ for a Leader-Follower System"* (Das, Shirgur,
**Vehale**, Pawar — University of Bristol). Headline result: a reported **97.93% reduction
in tracking error**. (The report PDF and experimental data live with the original
submission, outside this code repo.)

## Files

```
leader/      leader.ino   + Motors.h Encoders.h Kinematics.h LineSensors.h PID.h
follower/    follower.ino + Motors.h Encoders.h Kinematics.h LineSensors.h PID.h
```

> This is the **final TSOP-enabled version** (from `Final_Assessment/QTR_n_TSOP_EXP_*`).
> The earlier prototypes in `Assessment_2/` (bit/HEX comms experiments, `calibrated_follower`,
> `trial_leader/follower`, IR-range calibration sketches) are kept in the original
> location, not copied here.

## How it works (follower)

1. Calibrate the two forward sensors by spinning (`doCalibration`, scales readings to
   −1…+1).
2. Each loop: read calibrated analog value `AngC`, measure the digital sensor's capacitor
   **discharge time**, and detect the leader's IR pulse period → frequency.
3. Map `AngC` to a **dynamic speed** and drive forward/backward via PID to hold the target
   band: forward when `−0.03 < AngC < 0.4`, backward when `0.4 < AngC < 0.88`, else stop.
4. Compute running mean/variance/std-dev over the last 50 samples for noise rejection.

The leader simply drives forward/back at constant speed while toggling its IR emitter.

## Status

This is the final, demonstrated version. The fixes below have been **applied** to this copy
(each marked with a `// FIX:` comment). Because the D-term and `map()` fixes change control
behaviour, `Kd` is set to **0** in the sketches to preserve the demonstrated PI behaviour —
re-tune on hardware before enabling D.

### 1. PID D-term is dead code  ⚠️ functional bug
In `PID.h::update()`:

```cpp
error = measurement - demand;
last_error = error;                         // <-- set too early
...
diff_error = (error - last_error) / float_dt;   // <-- always 0
d_term = diff_error * d_gain;                    // <-- always 0
```

`last_error` is overwritten with the current `error` *before* the derivative is computed,
so `diff_error` is always 0 and `d_gain` (200) has no effect — the controller is really
just PI. Fix: keep the previous error and assign `last_error` **after** computing
`diff_error`:

```cpp
error = measurement - demand;
p_term = p_gain * error;
i_sum += error * float_dt;
i_term = i_gain * i_sum;
diff_error = (error - last_error) / float_dt;   // uses PREVIOUS error
d_term = diff_error * d_gain;
feedback = p_term + i_term + d_term;
last_error = error;                              // update last
return feedback;
```

### 2. `map()` called on a float  ⚠️ functional bug
In `follower.ino`:

```cpp
float dyn_speed = map(AngC, -1, 1, 0.0, 5.0);   // AngC is a float in [-1,1]
```

Arduino's `map()` is **integer (long) only**, so `AngC` is truncated to −1/0/1 and the
result loses all proportional resolution. Replace with a float map:

```cpp
float dyn_speed = (AngC + 1.0f) * (5.0f / 2.0f);  // [-1,1] -> [0,5]
```

### 3. Leader IR emitter is starved by blocking `delay()`
`leader.ino`'s `loop()` toggles the emitter once, then calls `delay(6000)` twice for the
forward/back moves. During those 6 s the emitter isn't toggled, so the "100 Hz" IR carrier
described in the report isn't actually emitted while the robot is moving. Drive the emitter
from a non-blocking timer (or a timer ISR / `tone()`) instead of `delay()`.

### 4. Minor / maintainability
- `check_fwd()` / `check_turn()` etc. contain unreachable code after `return` (`return
  true; fwd = true;`). Remove the dead line.
- `set_beep()` is named "non-blocking" but uses `delay()`. Rename or make it truly
  non-blocking with a stop-timestamp.
- `leader.ino` and `follower.ino` duplicate the entire helper/driver layer (`set_fwd`,
  `set_turn`, `setReturn`, low-pass, `pid_n_enc`). Factor the shared parts into a common
  header to avoid drift between the two.
- Pulse period is measured with `millis()` (1 ms resolution); for ~100 Hz (10 ms) signals
  this quantises the frequency heavily — `micros()` would be finer if frequency matters.
- `emit` (pin 11) is also the line-sensor `EMIT_PIN`; fine for the leader, but keep in mind
  the shared pin when extending the follower.
