Ball-and-beam device workflow

Core rule
- `q ...` sets the desired ball position.
- `r` starts closed-loop control.
- Once control is already running, send more `q ...` commands directly. Do not send another `r`.

Target commands

```text
q          # print current target and stored presets
q c        # center of the runner
q n        # near-sensor endpoint of the runner
q f        # far endpoint of the runner
q <cm>     # explicit offset from center, example: q -2.0
```

Important
- `q` commands do not move the motor by themselves.
- If the firmware is idle, motion starts only after `r`.
- If the firmware is already `RUNNING`, `q c`, `q n`, `q f`, and `q <cm>` change target live.

Fresh calibration from scratch

Use this when:
- firmware was reflashed
- mechanics changed
- `s` shows any calibration flag as `no`

Physical action before each command
- before `l`: hold the beam fully DOWN, ball at the far end from the ultrasonic sensor
- before `u`: hold the beam fully UP, ball at the near end close to the ultrasonic sensor
- before `p`: place the ball at the physical center of the runner and hold it steady
- before `b`: remove hands and keep clear of the mechanism

Exact command sequence

```text
s
d
l
u
p
e 1
b
v
s
```

What the calibration commands now mean
- `l` stores the lower actuator limit and the far runner endpoint
- `u` stores the upper actuator limit and the near runner endpoint
- `p` stores the physical runner center and the actuator control origin
- `b` checks the commanded motor direction against actual actuator motion

What to check after the final `s`
- `CAL,zero_calibrated=yes`
- `CAL,limits_calibrated=yes`
- `CAL,sign_calibrated=yes`
- `ACT_CFG,...,v=1`

If `ACT_CFG,...,v=0`, redo `p`, then `v`, then `s`.

First closed-loop run after calibration

```text
q c
e 1
r
```

What happens
- `q c` sets the target to runner center
- `r` goes directly into `RUNNING`
- there is no center-search phase anymore

What you do physically
- keep hands off the beam and ball
- let the controller bring the ball toward center

Normal daily use after calibration is saved

Center target

```text
s
q c
e 1
r
k
e 0
```

Near-side target

```text
s
q n
e 1
r
k
e 0
```

Far-side target

```text
s
q f
e 1
r
k
e 0
```

Custom target

```text
s
q -2.0
e 1
r
k
e 0
```

Change target during a run

If the system is already running, send these directly:

```text
q n
q f
q c
q <cm>
```

Do not send another `r` for a live target change.

Stop

```text
k
e 0
```

Meaning
- `k` stops closed-loop control
- `e 0` disables the motor driver

If `r` prints `ERR,run_blocked`

Read the `BLOCK,...` lines.

Common cases
- `BLOCK,zero`
  - runner center / control origin was not captured
  - run `p`
- `BLOCK,limits`
  - lower and upper endpoints are not captured
  - run `l`, then `u`
- `BLOCK,sign`
  - sign jog has not been done
  - run `b`
- `BLOCK,sensors`
  - sonar or AS5600 is not currently valid
  - run `s` and wait for `SENSORS,angle=ok,pos=ok`
- `BLOCK,faults`
  - clear with `f`, then inspect `s`

Useful status lines
- `SONAR_CFG,s=...,u=1`
  - confirms sonar sign and that upper limit is the near-sensor end
- `ACT_CFG,t=...,v=...`
  - `t` = saved actuator control origin
  - `v=1` = origin is valid and `r` may start
  - `v=0` = origin is missing; capture `p` again

Local serial-logger commands

```text
/bringup
/std center_reg
/std step3
/std disturb
/diag
/quit
```

Recommended manual workflow
1. Build and upload firmware.
2. Start the serial logger.
3. Calibrate once: `d`, `l`, `u`, `p`, `e 1`, `b`, `v`, `s`.
4. Start a center run: `q c`, `e 1`, `r`.
5. Change target live: `q n`, `q f`, `q c`.
6. Stop with `k`, `e 0`.
