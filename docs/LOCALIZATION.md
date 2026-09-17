# Knowing where you are, when the landmarks move

Run the numbers: `npm run tool -- tools/tagstudy.ts`

---

## The short answer

**No, the robot cannot know its exact position at all times.** Nothing in FTC can. But the
question is the wrong one to design against, because **the robot does not need a global
position for the thing that actually scores points.**

Shooting needs *range and bearing to the up CELL mouth*. There is an AprilTag bolted to that
mouth. That is a **direct relative measurement** — no global frame, no accumulated drift, no
heading term. The tag is best exactly where it matters most, and worst where it doesn't.

Global pose is still needed, for navigating to collection spots, for LEAVE and for PARK. But
those want inches, not tenths of a degree, and odometry supplies them.

---

## What the field actually gives you

Four `am-5888` panels, from `cad/parts.json`. **Two per hive, one on each CELL, all four
bolted to a rocker. There is not one static fiducial on this field** — no wall tags, no
perimeter tags, nothing on the A-frame.

| | radius from pivot | body angle | height, state A / B |
|---|---|---|---|
| red audience | 14.10 in | −6.1° | 49.7 / 35.6 in |
| red scoring | 14.10 in | −173.9° | 35.6 / 49.7 in |
| blue audience | 14.10 in | −6.1° | 49.7 / 35.6 in |
| blue scoring | 14.10 in | 186.1° | 35.6 / 49.7 in |

## Why the moving tag is a much smaller problem than it looks

**The rocker is bistable.** `params.json` gives `restAngles_deg: [−30, +30]` and it sits on a
hard stop at one end or the other. It is only in between during the ~1 s of an actual tip.

So a tag does not have a continuum of possible poses. **It has exactly two.** And because
each CELL carries its own tag with its own ID, *the ID you are looking at tells you which of
the two the rocker is in.* A tag you have identified has a pose you know exactly.

Getting the state wrong puts the tag **14.1 in** from where you think it is — so the ID
matters far more than the pose does. Three consequences:

1. **The tag ID is the rocker state sensor.** You do not need to infer the state from
   anything else; seeing which tag is up *is* the measurement. The sim already exposes
   `game.hiveTipping`; on the real robot that becomes "the tag I was tracking just moved".
2. **Reject fixes during a tip.** One second of transient, easily gated.
3. **Do not average across a tip.** Any pose filter must reset, not blend, when the ID
   changes.

## What a fix is worth

Tag 4 in, 640 px across 60°, 0.5 px corner noise:

| range | tag px | bearing err | range err | mouth half-angle |
|---|---|---|---|---|
| 36 in | 62 | 0.05° | 0.41 in | 15.5° |
| 54 in | 41 | 0.05° | 0.93 in | 10.5° |
| 78 in | 28 | 0.05° | 1.94 in | 7.3° |
| 110 in | 20 | 0.05° | 3.86 in | 5.2° |

**A tag is an excellent compass and a poor rangefinder.** Bearing error is flat with range —
it is a fixed pixel error on a fixed focal length. Range error grows as the *square* of
distance, because range is inferred from apparent size.

At the measured best shooting range of 54 in the mouth subtends 10.5° and a tag gives 0.05° —
**200× the margin needed.** Range is the weaker axis, and it is also the one the shot table is
indexed on, so that is where to spend effort: fuse tag range with odometry rather than trusting
it raw.

## The part everyone gets wrong: heading

A turret aimed from a *global pose* inherits the heading error directly. An FTC IMU drifts
1–3°/minute uncorrected.

| yaw drift | lateral miss at 54 in | of the mouth half-width |
|---|---|---|
| 2° | 1.9 in | 19% |
| 5° | 4.7 in | 47% |
| 7.5° | 7.1 in | 71% |

**Half the mouth is gone by 5° of drift**, which is squarely inside a match's worth of
uncorrected IMU. This is the strongest single argument for aiming off the tag: *a relative
bearing has no heading term in it at all.*

## The dead-reckoning budget

Odometry error scales with **distance travelled**, not elapsed time. A robot cruising 45 in/s
at 55% duty covers ~62 ft in AUTO and ~248 ft in TELEOP.

| method | rate | after AUTO | after TELEOP |
|---|---|---|---|
| drive encoders on omni wheels | 5% | 37 in | 148 in |
| two dead-wheel pods | 1% | 7.4 in | 30 in |
| fused pod + IMU | 0.5% | 3.7 in | 15 in |

Two conclusions that do not depend on the exact rates:

- **Drive-motor encoders on omni wheels are not a localizer.** The free rollers slip sideways
  by design and the encoder cannot see it. This robot's X-drive makes that worse, not better.
- **Good pods carry a 30 s autonomous alone. Nothing carries a full match alone.**

---

## The plan

### Phase 0 — confirm the three things this rests on
None is a physics guess; they are facts someone must look up. **Do this before building.**
1. **The tag IDs.** The whole scheme turns on each CELL carrying a *distinct* ID. Not
   recorded anywhere in this repository.
2. **The printed tag size.** The panel is 17 × 4.34 in; the tag inside it is smaller.
3. **The panel's normal.** `cad/parts.json` gives a bounding box, not an orientation. A tag
   past ~60–70° off its normal stops decoding, and the panel is on the pocket *underside* —
   so the far-range figures above are optimistic until this is measured off the STEP.

### Phase 1 — stop cheating in the sim
`robot.json` has `localizer: { source: "groundTruth", noise: { xy_in: 0, heading_deg: 0 } }`,
and PLAN.md line 30 admits it: *"the robot's pose is ground truth from the physics."* Every
autonomous result so far assumes perfect knowledge of position.

Add `source: "odometry"` — integrate the drivetrain's own motion with a slip term and an IMU
with drift and latency. **Expect AUTO scores to fall.** That drop is information, not a
regression: it is the size of the problem being measured for the first time.

### Phase 2 — a simulated tag camera
A `TagCamera` sensor producing detections, not poses: ID, relative bearing, relative range,
each with the noise model in `tagstudy.ts`. Gate on FOV, on the pixel floor, and on incidence
once Phase 0 gives the normal. Emit nothing while `hiveTipping`.

This is the piece PLAN.md §16 lists as future work ("AprilTag detection with a simulated
camera"). It stays a *sensor*, so the Java sees exactly what the SDK's
`AprilTagProcessor` would hand it.

### Phase 3 — aim off the tag, not off the pose
Change `BuiltinTeleOp` and `TurretTracker` so that when a tag of the up CELL is in view, the
turret uses the **measured relative bearing** directly and the shot table is indexed on the
**measured range**. Fall back to the global pose only when no tag is visible.

This is the highest-value change in the document and it is mostly deletion: the whole
pose → bearing → turret chain is bypassed when the measurement is available.

### Phase 4 — put the camera on the turret
The single best hardware decision available. A chassis-mounted camera with a 60° FOV covers
17% of headings, so "a tag is in view" is mostly luck. **The turret already tracks the CELL**,
so a camera bolted to it is pointing at the tag whenever the shooter is aimed — by
construction, not by chance. It also gives the turret a closed loop on its own encoder zero,
which is otherwise a calibration constant that drifts with every collision.

### Phase 5 — fuse, and only then
A small filter: odometry propagates, tag fixes correct. Weight bearing heavily and range
lightly, matching the error model above. Reset rather than blend on an ID change. Keep it a
complementary filter unless a measured need for more shows up — a full EKF here would be
complexity nobody asked for.

### Phase 6 — re-run the whole calibration loop
`shottable.ts` → `landrate.ts` → `autoplan.ts`, with honest localization in place. The current
AUTO conclusion ("28 points, stand at 54 in") was computed with a robot that knows its
position perfectly. **It is optimistic and should be expected to move.**

---

## What would change this answer

- **A static tag being found on the field** (on the A-frame, or a wall) would make
  conventional global localization viable and demote most of the above.
- **The two CELLs sharing one tag ID** would break the state-identification argument
  completely, and the 14.1 in ambiguity would become permanent and unresolvable by vision
  alone.
- **A grazing-incidence limit tighter than expected** would shrink the useful range enough
  that the tag becomes a short-range aid only, and odometry quality becomes the whole game.
