# Robotic Systems — Final Projects

Two robotics projects built on the **Pololu 3Pi+ (ATmega32U4)** platform, programmed in
Arduino C++. This repository is a cleaned-up, **code-only** version of the best/working
sketch for each project, with the bugs from review fixed. (The reports, experimental data,
and plots live with the original submission and are intentionally not included here.)

```
.
├── magnet/                  Project 1 — autonomous magnet search-and-return robot
│   ├── magnet.ino           (single Arduino sketch + driver headers)
│   ├── Encoders.h  Kinematics.h  LineSensors.h  Magnetometer.h  final_motor_2.h
│   └── README.md            what it does, status, fixes applied
│
└── leader-follower/         Project 2 — two-robot IR leader–follower system
    ├── leader/              leader.ino  (+ headers)  — the IR emitter / lead robot
    ├── follower/            follower.ino (+ headers) — tracks the leader via QTR + TSOP IR
    └── README.md            what it does, status, fixes applied
```

## Project summaries

| | **magnet** | **leader-follower** |
|---|---|---|
| Type | Single robot, autonomous | Two robots (leader + follower) |
| Goal | Search an area, detect a magnet, return home | Follower keeps fixed distance/angle to a moving leader |
| Key sensors | 5× line sensors, LIS3MDL magnetometer, encoders | 2× forward QTR-RC IR + TSOP IR, encoders |
| Control | Odometry state-machine (turn/forward/return) | PID speed control + dynamic speed + statistical error correction |
| Source of truth | `final_Bot/` (canonical; `final_h_wo_return/` is a variant) | `Final_Assessment/` QTR_n_TSOP zips (final, TSOP-enabled) |

## Build / upload (both projects)

Open the relevant `.ino` in the Arduino IDE (with the **Pololu A-Star / 3Pi+ 32U4** board
support and the Pololu libraries installed: `PololuOLED`, `Pushbutton`, `LIS3MDL`,
`Encoder`). All headers must sit next to the `.ino` (they do). Select the 3Pi+ 32U4 board
and upload.

## Code review — headline findings

The bug fixes from the review **have been applied** to this `final/` copy — see
[`CHANGELOG.md`](CHANGELOG.md) for the full list, and each project's `README.md` for the
line-by-line explanation. Every change is also marked inline with a `// FIX:` comment. The
original development folders one level up are untouched.

**magnet** — fixed: magnetometer calibration min/max were initialised **inverted** (could
stop magnet detection from ever triggering); `millis()` timestamps stored in 16-bit
`unsigned`; PWM never clamped; mistyped `LineSensors_c` constructor; `Magnetometer::
initialise()` returning nothing.

**leader-follower** — fixed: PID **D-term was dead code** (`d_gain` did nothing); `map()`
called on a **float** (integer-only → lost resolution); PWM clamp; mistyped constructor.
Because the system was tuned with the D-term inactive, `Kd` is set to **0** in the sketches
to preserve the demonstrated (PI) behaviour — re-tune on hardware before enabling D.
