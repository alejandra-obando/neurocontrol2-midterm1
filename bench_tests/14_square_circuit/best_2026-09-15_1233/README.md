# The best lap of the square — 2026-09-15, 12:33

**It is the best result of the session (at that time).** The complete log of
this run (the script snapshot is kept in the original project archive). Point
to go back to.

## What distinguishes it: the heading is tracked in absolute terms

Until this run each turn was independent, it overshot by about one degree
towards the same side and nothing compensated it, so the lap closed 7 or 8
degrees off. Here **the heading the robot SHOULD have** is tracked (n × 90
from zero) and whatever is left over from one turn is discounted in the next
stretch.

🔑 **And it is not corrected by turning while stopped**, which was already
tried and discarded on Sep 2 because the smallest motion of the car is larger
than the error to correct. It is corrected **while rolling**, taking advantage
of the firmware moving its own reference with the turn rate it is sent:

    yawRef += pedidoW * 180.0f / PI * dt;

A `w` pulse at the start of the stretch shifts the reference by the missing
degrees, and the loop takes the robot there while it advances, which is where
it does have authority.

## The figures

- **Turns**: +93.7 · +91.9 · +89.9 · +90.3. Mean error +1.47. The four at the
  first attempt, in about 4 s, with no stalls.
- **The heading ends at +354.18 where it should read 360**, i.e. 5.8 degrees of
  final deviation, against 8.2 for the best previous run.
- **Stops**: 56, 83 and 89 mm with a threshold of 95.
- **Re-centering after the turn**, at 7 cm from the right wall: −2, −16 and
  +0 mm.
- What was left over after each turn, with the compensation already in place:
  −1.87, +2.75 and −4.44 degrees.

## The configuration

Front stop at **95 mm** with progressive braking from 330 and stopping by
anticipation (it brakes based on where the robot will be 0.35 s from now).
Turn with the parameters validated on Sep 14, `tolgiro` 4.4 and `gradfreno`
45. Lateral target **7 cm** from the right wall with the 17 mm of inertia
compensated. Centering by **taps**: fixed force at 40, which is the executable
minimum, and what is modulated is how often it is applied.

## What is still wrong, and already diagnosed

🔴 **The heading compensation overshoots and oscillates with growing amplitude**
(−1.87, +2.75, −4.44). The cause has been found: the firmware **jumps to
W_ARRANQUE, 36 degrees/s, as soon as any `w` is requested**, because otherwise
the short-gearbox wheel does not break free. Asking it for 8.8 degrees/s, each
pulse moved four times more than computed. **The pulse time cannot be computed
blindly**: the loop has to be closed on `yaw_ref`, which travels in the
telemetry, giving short pulses until the reference is where it should be. That
is what is being tested next.

⚠️ **The front sensor is still uncalibrated below 150 mm.**
