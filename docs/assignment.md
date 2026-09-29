# Assignment — Midterm Exam 1 (Neurocontrol 2)

**Design and construction of an autonomous robot able to enter a maze,
navigate it and find the exit.** Universidad Autónoma de Occidente, Cali,
Colombia — rubric dated 2026-08-28 (translated from the original Spanish).

The maze also carries **instructions painted as geometric figures on the
walls**:

| Figure | Instruction |
|---|---|
| Right triangle (▶) | turn right 90° |
| Left triangle (◀) | turn left 90° |
| Circle | U-turn: go back |
| Square | stop for 10 s and re-evaluate: the wall in front may or may not disappear |

## Rubric

| # | Criterion | Weight | Excellent | Good | Acceptable | Insufficient |
|---|---|---|---|---|---|---|
| 1 | Mechanical design of the robot | 10 % | Robust, compact, stable structure; proper placement of wheels, sensors, battery and center of mass | Functional design with small deficiencies | Works, but with stability or construction problems | Deficient design or unable to complete the test |
| 2 | Locomotion system | 10 % | Excellent control of advance, turn and trajectory; good calibration between motors | Adequate control with minor errors | Frequent deviations | Unstable or hardly controllable motion |
| 3 | Sensors and perception of the environment | 15 % | Reliably detects walls, corners, intersections and obstacles | Adequate detection with occasional errors | Partially detects the environment | Unreliable readings or badly integrated sensors |
| 4 | Low-level control | 10 % | Precise motor control; uses feedback, PID or an equivalent, correctly calibrated strategy | Stable control with small oscillations | Basic control with noticeable errors | Unstable or mostly open-loop control |
| 5 | Maze navigation algorithm | 20 % | Clearly defined, robust strategy able to solve different configurations | Solves most configurations | Works only under certain conditions | Incomplete strategy or unable to solve the maze |
| 6 | Autonomy | 10 % | The test is done completely without human intervention | Requires a minor intervention | Requires several interventions | Mainly manual operation |
| 7 | Success in finding the exit | 10 % | Finds the exit consistently | Exits in most attempts | Exits occasionally | Does not manage to exit |
| 8 | Time and efficiency of the route | 5 % | Efficient trajectory, few backtracks and short time | Reasonably efficient route | Long route or many errors | Extremely inefficient navigation |
| 9 | Robustness and repeatability | 5 % | Consistent results over multiple runs | Small variability | Significant variability | Non-reproducible result |
| 10 | Documentation and technical argumentation | 5 % | Design, models, algorithm, experiments and results rigorously documented | Adequate documentation | Incomplete documentation | No technical justification |

Grade intervals: Insufficient (0.0 < I ≤ 2.0) · Acceptable (2.0 < A ≤ 3.5) ·
Good (3.5 < G ≤ 4.5) · Excellent (4.5 < E ≤ 5.0).

The original title asks for a *differential* robot; the team built a
four-wheel **mecanum** platform, which can also drive as a differential robot
and additionally translate sideways.
