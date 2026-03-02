d              ← reset cal + save EEPROM (clean slate)
a              ← level beam by hand, then press a (captures AS5600 zero)
p              ← place ball at beam center, press p (captures sonar zero)
l              ← tilt beam motor-DOWN by hand, press l (captures lower limit)
u              ← tilt beam motor-UP by hand, press u (captures upper limit + auto-sets AS5600 sign)
b              ← stepper sign test (200-step jog, returns to start; check delta > 1°)
v              ← save all to EEPROM
s              ← verify: STATE=READY, all cal=yes, faults=none
t              ← enable telemetry (so you can see what happens)
r              ← run
k              ← stop when donet
