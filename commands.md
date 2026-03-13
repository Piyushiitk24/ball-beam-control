Ball-and-beam device workflow

Core rule
- `q ...` sets the desired ball position.
- `r` starts closed-loop control.
- Once control is already running, send more `q ...` commands directly. Do not send another `r`.
- each logger session creates one timestamped run folder under `data/runs/`
- end each test with `/quit` so the run is stopped cleanly and plots are generated automatically

Recommended morning hardware workflow

Terminal A:

```bash
cd /Users/piyush/code/ball-beam-control
ls /dev/cu.usb*
pio run -e nano_new
pio run -e nano_new -t upload --upload-port /dev/cu.usbserial-A10N20X1
```

Terminal B:

```bash
cd /Users/piyush/code/ball-beam-control
./.venv/bin/python analysis/serial_logger.py --port /dev/cu.usbserial-A10N20X1
```

Logger terminal:

```text
/bringup
s
q c
e 1
r
q f
q n
q c
s
/quit
```

Hold times for the retest:
- first `q c`: about `8 s`
- `q f`: about `8 s`
- `q n`: about `8 s`
- final `q c`: about `15 s`

Important calibration note
- `/bringup` already sends `d`, `l`, `u`, `p`, `e 1`, `b`, and `v`
- if you use `/bringup`, do not send a second manual `v`
- if you calibrate manually, you must send `v` after `b` or the calibration is not saved

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

This manual sequence includes `v` because you are saving calibration yourself. The `/bringup` helper includes that same save step internally.

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

Recommended clean logger exit

```text
/quit
```

Meaning
- `k` stops closed-loop control
- `e 0` disables the motor driver
- `/quit` sends `k`, then `e 0`, exits the logger, and writes the summary plot into the run folder

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

What they do
- `/bringup`: guided calibration flow including save with `v`
- `/diag`: prints a status snapshot
- `/quit`: stop safely, disable driver, close logger, auto-generate plot

Recommended manual workflow
1. Build and upload firmware.
2. Start the serial logger.
3. Calibrate once with `/bringup`, or manually with `d`, `l`, `u`, `p`, `e 1`, `b`, `v`, `s`.
4. Start the retest run: `q c`, `e 1`, `r`.
5. Change target live in this order: `q f`, `q n`, `q c`.
6. Capture a final `s`.
7. End with `/quit`.
