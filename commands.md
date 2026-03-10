Calibration

```text
d          # reset calibration and save defaults
l          # capture lower actuator limit (beam down, ball far)
u          # capture upper actuator limit (beam up, ball near)
e 1        # enable driver
b          # stepper sign jog
v          # save calibration
s          # verify status
```

Optional override

```text
p          # capture manual sonar-center override
```

Setpoint control

```text
q          # print current target and calibrated presets
q c        # center target
q n        # midpoint between center and near limit
q f        # midpoint between center and far limit
q <cm>     # explicit target in cm relative to center
```

Run control

```text
t          # telemetry on/off
r          # start closed loop
k          # stop
e 0        # disable driver
f          # clear faults
x          # compact fault + sonar diagnostics
```

Local serial logger commands

```text
/bringup          # guided calibration flow
/std center_reg   # standard center-regulation run
/std step3        # standard 3-position step-tracking run
/std disturb      # standard disturbance-rejection run
/diag             # send s, x
/quit             # send k, e 0, and exit
```

Standard run durations

```text
center_reg   12 s
step3        36 s
disturb      18 s
```
