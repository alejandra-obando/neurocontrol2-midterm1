# The learning agent, in detail

This note describes exactly what kind of reinforcement learning the robot
uses, as implemented in [`raspberry_pi/rl.py`](../raspberry_pi/rl.py), and
where each piece lives in the code.

## Classification

| Question | Answer in this project |
|---|---|
| Does it build a model of the world to plan? | **No — model-free.** The robot keeps a map, but it is only used to *restrict* the actions (Trémaux rules) and to store the best route; it is not used to plan with a transition model. |
| What does it learn? | **Values (value-based).** A value per cell and direction, Q(s, a), and the action is chosen with a softmax over the scores. There is no separate actor/critic. |
| On-policy or off-policy? | **Off-policy (Q-learning).** The bootstrap uses the *maximum* value of the free directions in the next cell: `v_tab_sig = max(q_sig[i] for i in libres)`. |
| Where is it stored? | **Three memories that vote together:** a tabular Q with eligibility traces, a small neural Q network, and a pattern memory. |

In one sentence: **model-free, value-based Q-learning with eligibility
traces (Q(λ)), combined with a neural Q network and a pattern memory, with
hard Trémaux-style prohibitions that shrink the action set before the
softmax.**

## The three memories

| Memory | What it is | Update rule | Code |
|---|---|---|---|
| **Q table** | One row of 4 values per cell of the relative map | TD error δ = r + γ·max Q(s′,·) − Q(s,a), spread backwards with *replacing* eligibility traces (λ = 0.85): Q ← Q + α·δ·e | `AgenteRL.aprender`, step 3 |
| **Q network** | MLP 8-32-32-4 with tanh, written by hand in numpy | Semi-gradient TD(0) with its own target r + γ·max Q_net(s′), gradient clipping | `RedQ.paso_sgd`, step 4 |
| **Pattern memory** | Value per *local signature* (walls seen from the heading + sector of the goal) and *relative* action | Exponential moving average towards r + γ·max Q_tab(s′) | `MemoriaPatrones.actualizar`, step 5 |

The Q network is not trained from zero: it starts from weights pre-trained
over 4 555 runs in simulation ([`q_network_weights.h`](../raspberry_pi/q_network_weights.h)),
and its weight in the vote (β) grows with the number of runs.

## Decision

```
score(d) = 1.0·Q_tab(c,d) + β·Q_net(d) + 0.6·V_pattern(rel(d)) + momentum(d, heading) + 0.4·g(d)
P(d)     = softmax(score / T)  over the candidate directions only
```

- `g(d)` is the gaussian activation of the bio-inspired neurocontroller that
  runs on the ESP32 (Naka–Rushton + gaussian layer), rotated to the absolute
  frame — the neural layer votes directly in the decision.
- `T` starts at 0.60 and drops by 0.06 per run down to 0.20; in exploitation
  mode (once an exit is known) it goes to the minimum and the **master route**
  (best loop-free route) is followed directly.

## Rewards

| Event | Reward |
|---|---|
| Every step | −0.05 |
| Entering a new cell | +0.60 |
| Revisiting a cell | −0.80 × (visits − 1) |
| Going back the way it came | −3.00 |
| Wall / rejected command | −0.50 |
| Stall | −1.00 |
| Exit found | +10 (+ up to 5 for beating the best time) |
| Turn of 90° / 180° | −0.08 / −0.20 |

## Prohibitions before scoring (`maze_map.py`, `navigator.py`)

A punishment never takes a probability to zero, so the candidate list is cut
**before** the softmax: no direction leading to one of the last 6 occupied
cells; if a never-seen cell is reachable, only those; the free front is
always a candidate; going back only if it is the only option; a dead end left
in reverse is walled off for the rest of the run.

## Connection with the basal ganglia

The same structure can be read in biological terms: the TD error δ plays the
role of the dopamine prediction-error signal, the softmax over candidates
plays the role of action selection by disinhibition, and the hard
prohibitions act like a gate that removes actions before they compete. What
this implementation does **not** have is an actor–critic split, separate
"go"/"no-go" (D1/D2) pathways, or learning inside the weights of the
neurocontroller itself (a three-factor, dopamine-modulated Hebbian rule).
Those are natural next steps to make the learning biologically plausible.

## Notes for whoever continues this work

- **Traces with an off-policy target.** The Q table uses `max` in the target
  but does not reset the traces after exploratory actions (as Watkins' Q(λ)
  does). It worked well in practice, but strictly it mixes on- and off-policy
  updates.
- **The goal coordinate is still a feature.** The progress reward is zero
  because the real exit is unknown, but the network input and the pattern
  signature still include the vector towards `META = (15, 15)`, a placeholder
  corner (`config.py`). That feature carries no real information about the
  exit.
- **Map drift limits memory across runs.** All three memories are indexed by
  the odometric map, so learning is reliable within a run but not across runs
  (see [report §IX-E](report.md#ix-e-memory-across-runs-is-limited-by-odometry)).
- **Vision is classical, not a CNN.** `perception.py` keeps the slot for a CNN,
  but the figures are recognized with OpenCV contour geometry
  (`test_camera.py`, `camera_master.py`).

## Reproducing the learning curve

```bash
cd raspberry_pi
python3 rl.py            # 12 runs on an 8x8 maze, then generalization to a new maze
python3 test_headless.py # the whole Pi stack against a fake ESP32
```
