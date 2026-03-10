s              ← status / fault summary
d              ← reset calibration + save EEPROM
p              ← optional manual sonar-center override
l              ← capture lower actuator limit + far sonar distance
u              ← capture upper actuator limit + near sonar distance
                 firmware then derives sonar center midpoint and seeds actuator trim at travel midpoint
e 1            ← enable stepper driver
b              ← stepper sign test jog (check delta ≈ 22.7°)
g              ← optional sonar-direction validation only
v              ← save calibration and any learned trim
s              ← verify: all cal=yes, SONAR_CFG,s=-1,m=o,u=1, ACT_CFG shows trim source
t              ← telemetry toggle
r              ← run
k              ← stop
e 0            ← disable driver

# Standard calibration
t          # telemetry OFF
l          # capture lower limit (beam fully DOWN / ball far)
u          # capture upper limit (beam fully UP / ball near)
e 1        # enable driver
b          # sign calibration jog
v          # save limits + trim seed
s          # confirm all flags = yes

# Optional manual ball-center override
p          # capture sonar center manually if the auto midpoint needs correction
v          # save the manual center

# Closed-loop run
e 1
r
t

# Notes
# - `a` is no longer part of calibration.
# - `theta_deg` and `theta_cmd_deg` in telemetry are actuator-relative coordinates around the learned trim.
# - During RUNNING the firmware can learn actuator trim; save it with `v` if the new trim works well.
