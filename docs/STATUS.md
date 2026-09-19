# Where this stands

Everything below is measured, by a named tool, at the commit it says. No number here is
estimated, and where a number is bad it is written down as bad. Re-run the tool to check any
of it — that is the point of naming them.

Last measured: 19 September 2026. 168 TypeScript tests pass, 51 Java self-check assertions
pass, the site builds.

---

## 1. What the thing is

A physics simulator for the FTC 2026-27 BIOBUZZ game, written so a real OpMode can be driven
against it. Two implementations of the same robot brain are kept in step on purpose:

| | what it is | why |
|---|---|---|
| `packages/core/src/robot/builtinTeleOp.ts` | the brain the simulator runs | fast to iterate |
| `java/teamcode/` | **the deliverable** — real FTC code | what actually goes on the robot |

They mirror each other and tests grep both. When they disagree, the Java is usually right: it
has been the one without the bug more than once.

Balls fly on the project's own integrator (drag + Magnus, 3-D, 1/480 s). Rapier handles
contact and the rest of the field. Every physical constant lives in `config/*.json` with a
`_source` note saying whether it was measured, cited, derived or guessed, and every one is a
live slider in the **Variables** tab.

---

## 2. What it scores right now

### Standing still, on a square the map calls green

`tools/zoneaudit.ts` — 24 squares, 4 balls each

| balls | in | accuracy | time per ball |
|---|---|---|---|
| 88 | 84 | **95%** | **1.41 s** |

Grey squares fired 0 balls, correctly.

### By distance

`tools/landrate.ts` — 12 balls per range

| range | in | accuracy | time per ball |
|---|---|---|---|
| 40 in | 11/11 | 100% | 0.78 s |
| 55 in | 10/12 | 83% | 0.79 s |
| 70 in | 12/12 | 100% | 0.66 s |

### By angle round the side

`tools/obliquity.ts` — 10 balls each, from 55 in

| off the opening | accuracy |
|---|---|
| 40° | 80% |
| 50° | 90% |
| 60° | 100% |
| 70°+ | never fires — the camera cannot read the tag past 65° |

### Driving and shooting

`tools/fastfire.ts --untiltip` — whole patrol, 104–129 balls a row, before the HIVE tips

| stand-off | speed | balls | in | accuracy | time per ball | gate open |
|---|---|---|---|---|---|---|
| 40 in | 0.47 m/s | 129 | 96 | 74% | 1.01 s | 89% |
| **40 in** | **0.73 m/s** | 114 | 94 | **82%** | **1.15 s** | 84% |
| 40 in | 0.94 m/s | 116 | 87 | 75% | 1.34 s | 76% |
| 46 in | 0.57 m/s | 111 | 88 | 79% | 1.59 s | 77% |
| 52 in | 0.51 m/s | 109 | 88 | 81% | 1.59 s | 75% |
| 52 in | 0.65 m/s | 104 | 88 | 85% | 2.41 s | 67% |

**Best operating point: 40 in, 0.73 m/s.** Driving faster than that is worse on both counts.

### Autonomous

`tools/autocheck.ts` — 5 runs

4.2 balls fired, 3.6 in (**86%**), LEAVE 5/5, PARK 5/5, 8 points.

---

## 3. The ceilings, so it is clear what is left

`tools/ceiling.ts`

**Accuracy is nearly maxed.** A shot can miss with perfect aim, because the launch scatter
(1.5% of exit speed, 1° of elevation) is irreducible:

| range | best possible per shot |
|---|---|
| 30 in | 86.2% |
| 42 in | 87.7% |
| 54 in | 94.7% |
| 82 in | 99.7% |

Close in the ceiling is low because the only arc that fits is a steep lob, and a steep lob
arrives nearly vertically and bounces back out. The measured ceiling across everything, on the
move, is **93.7%** (`config/landcal.json`, 708 settled shots).

At 40–46 in the ceiling is 88% and the robot gets 82% — **93% of what is available.**

**Time is where the room is.** `transfer.cycleTime_s` is 0.60 s (mechanism floor: elevator
0.26 + gate 0.08 + margin), so 1.67 balls a second is the hard limit.

| gate open | time per ball at 90% landed |
|---|---|
| 100% | 0.67 s |
| 80% | 0.83 s |
| 51% | 1.31 s |
| 30% | 2.22 s |

The best row fires 1.05 balls/s against a possible 1.67 — **59% of the machine's capacity.**
That is the headroom worth chasing, not the accuracy.

---

## 4. What is wrong, in order of how much it costs

### The probability gate does nothing

`flywheel.minLandProb` at 0.5, 0.7, 0.8 and 0.9 fires the **identical** shots. At 0.92 it
fires none. It is a cliff, not a dial.

The cause is `config/landcal.json`: its observed rates are 0.911 for five of six bins and
0.937 for the sixth, so the calibrated P(land) only ever takes two values and no threshold can
separate two shots. **The robot computes a probability that does not vary, so it cannot tell a
good shot from a bad one.** This is the single biggest thing standing between the current
1.15 s per ball and the 0.68 s the mechanism allows.

### And the model is built on the wrong things

`tools/whatmisses.ts` — 385 settled shots, land rate by quartile of each feature:

| feature | worst → best | spread |
|---|---|---|
| **range** | 38 in 71% → 46 in 88% | **17 pts** |
| exit speed | 91% → 81% | 15 pts |
| aim error at launch | 79% → 86% | 6 pts |
| rpm error | 84% → 80% | 4 pts |

Aim error and rpm error are the two factors P(land) is **made of**, and they barely move the
outcome. Range, which it does not gate on at all, moves it most. The miss itself is unbiased
(2–3 cm downrange, 0 sideways, ±17–23 cm scatter), so there is no trim to find — only
selection can help, and the thing it selects on is the wrong thing.

`shot.minRange_in` exists for this (refuse shots closer than X). A 42 in floor measures **96%
landed** — and ships **off**, because at every stand-off the field allows the robot spends too
much time inside the floor and the cost is ruinous (14.9 s per ball).

### The HIVE tips, and then the goal faces away

After about 12 balls the rocker goes over. The up CELL becomes the other one, which opens the
other way, so from the same spot there is no shot and the tag is edge-on at 110–129°. At
1.57 m/s over 60 s the hive tips 4 times and **61–74% of the drive is post-tip**. The robot is
right to refuse; the driver has to reposition. Any measurement that does not account for this
is measuring a robot standing on the wrong side of a turned-over goal.

### Fast driving has almost nowhere to happen

- Straight out from the red mouth **the field ends at about 55 in** (mouth at z = 16, boards at 71).
- Of the 52 green squares, **7 are at 60 in or more** — the range the hood needs to lead a
  1.7 m/s shot — and all 7 sit 49–60° off the opening.
- At 1.7 m/s the robot **cannot shoot inside 50 in at all**: the ball's own horizontal speed
  there (1.4–2.1 m/s) is no greater than the chassis, so the launch would have to go nearly
  straight up, past the hood's 80° stop. It fires fine from 60 in out.

### Smaller, known, measured

- **First shot after arming costs 5.8 s** — the belt only runs once the flywheel is on, so the
  tube primes from cold. Arm early and it is free.
- **The camera cannot read the tag past 65° of incidence**, and the tag rides the rocker.
- **Receding** — the target rpm rises with range faster than the wheel follows. The gate
  refuses, correctly.

---

## 5. Settled, do not re-litigate

Each of these was measured and the answer was "leave it alone". Re-measure only if the thing
underneath changes.

| tried | result |
|---|---|
| turret error gate 3° → 5° | 1.37 → 1.32 s/ball but 75% → 70% landed. Kept 3°. |
| off-opening cap 60° → 65° | 1.37 → 1.56 s/ball and 75% → 66%. More shots, fewer scored. Kept 60°. |
| trusting odometry 1.5 → 3 → 6 s | 53 → 66 shots, the **same 43** landed. Blind shots do not score. |
| raising the incidence limit 65 → 75° | identical; the panel is at 85°. |
| wider lens 60 → 80° | removes the lens block, incidence takes over. No gain. |
| subtracting pocket depth from the aperture | deleted 20 green squares that measure 98%. Reverted. |

---

## 6. Where to look

| for | read |
|---|---|
| every constant and where it came from | `docs/VARIABLES.md` (generated) |
| why a thing is the way it is | `docs/DECISIONS.md` |
| the ballistics | `docs/PHYSICS.md` |
| pose and vision | `docs/LOCALIZATION.md` |
| driver controls | `docs/CONTROLS.md` |

**A measurement that beats the mechanism is a broken measurement, not a fast robot.** Four
tools in this project have printed times faster than the 0.60 s the feed physically takes, and
every one of them was wrong. If a number looks too good, check it against `cycleTime_s` first.
