# What the simulator actually models, and where each rule lives

Every claim here names the file that implements it. If a claim and the code disagree, the
code is right and this file is stale — say so.

Nothing physical is written in code. Every constant is read from `config/params.json`,
`config/robot.json` or `config/motors.json`, each value carries a `_source` saying whether
it was measured, taken from the CAD, or guessed, and the **Variables** tab writes to those
same objects while the world is running.

---

## 1. Solver

| | |
|---|---|
| Engine | Rapier 3D 0.20 (`@dimforge/rapier3d-compat`), WASM |
| Step | `params.sim.dt` = 1/240 s, `substepsPerFrame` = 4 → one animation frame is 1/60 s |
| Solver iterations | `params.sim.solverIterations` |
| CCD | on balls only (`params.sim.ccdOnBalls`) — a ball at 14 m/s moves 58 mm per step, more than its own radius |
| Determinism | same seed + same command log ⇒ bit-identical snapshots (`tests/determinism.test.ts`) |

**Collision groups** — `packages/core/src/physics/groups.ts`. The rocker collides with balls
only. The real A-frame is open where the rocker swings, and any convex approximation of it
jams the pocket against a panel. The robot cannot reach the rocker anyway: R102 caps
expansion at 29 in and the pivot is at 44 in.

**Mass must go on the collider.** `RigidBodyDesc.setAdditionalMass` and
`setAdditionalMassProperties` are silently ignored in this Rapier build — the body comes out
with mass 0 and invMass 0, i.e. immovable, and no force does anything. Every mass in this
project is set with `ColliderDesc.setMassProperties`.

**Friction and restitution are combined with `Min`, not Rapier's default `Average`.** The
chassis shell against 0.85 tile friction averaged to 0.435, which dragged the robot at about
60 N — most of the drivetrain's output — and held it to half its top speed.

---

## 2. Drivetrain — `packages/core/src/physics/robot.ts`

Not a velocity you set. A torque that makes a force that accelerates a mass.

1. **Motor** (`motors.ts`): each DC motor is a first-order model — stall torque, free speed,
   winding resistance, rotor inertia — from `config/motors.json`. Torque falls linearly with
   speed and with the battery voltage it actually has.
2. **Hub loop** (`hub.ts`): `RUN_USING_ENCODER` and `RUN_TO_POSITION` run a PIDF exactly as
   the Control Hub does, at `hub.loopPeriodMs`, with encoder velocity measured over
   `encoderVelocityWindowMs` and command latency of `commandLatencyMs`. Anti-windup is
   conditional integration.
3. **Mecanum kinematics**: wheel speeds → chassis motion through the Jacobian transpose.
4. **Tyre model**: each wheel's force comes from the slip between its commanded surface
   speed and the ground speed under it, capped by `µ·N`. `N` includes weight transfer from
   the previous step's acceleration. This is why flooring it on low `env.tileMu` spins the
   wheels instead of accelerating, and why strafing is about 70% as efficient as driving
   straight.
5. **Battery** (`battery.ts`): total current sags the pack by `I·Rint`, and the sag feeds
   back into every motor's available torque next step.

Measured top speed 62.8 in/s against 62 in/s hand-computed from the motor curve
(`tests/drivetrain.test.ts`).

---

## 3. Intake, hopper and feed — all contact, no state machine

This was a scripted pipeline once: a ball that touched a trigger box vanished and reappeared
as a number in an array. It is now geometry and forces, end to end. `tools/mechcheck.ts`
drives each stage separately and reports where a ball actually got to.

**The chassis is built out of plates**, not one cuboid: floor, two sides, a back, a front
wall that stops *above* the intake mouth, and a vertical feed tube up the turret axis. A ball
is a rigid body the whole way through the robot.

**Intake** (`stepIntake`). The roller does not "capture" anything. Slip between the roller's
surface speed and the ball's own speed produces a friction force capped by `rollerMu · N`,
where `N` is the ball's weight plus `squeeze_N` from the roller being preloaded onto it —
the same slip-and-cap form as the tyres. Consequences that nobody wrote down: running the
roller backwards ejects a ball through the same code, chasing a ball at roller speed picks up
nothing, and a stalled roller has no surface speed and therefore no grip, which is what a jam
is. The roller also presses the ball *up* while it is still below the bin floor and *down*
once it is over the lip.

> There is no ramp. There was one; a plate reaching down to tile level to catch a ball also
> drags on the tile, and it cost three quarters of the robot's top speed. A real
> over-the-bumper intake does not use one either — the compliant roller carries the ball over
> the floor plate's edge. A 71 mm ball cannot escape under a 20 mm ground clearance.

**Hopper** (`censusBalls`). There is no add/remove bookkeeping. A ball is in the hopper
because its centre is inside the hopper, recomputed every step. Capacity is emergent: the bin
holds what fits. `hopper.capacity` is a rules figure and a preload guard, not a gate.

**Feed tube.** A square bore up the turret's rotation axis, because a turret has to be fed on
its own axis. Three details each cost an afternoon:

- **Open front *and* back.** With one opening, any ball that ended up behind the tube was
  unreachable and sat in the bin for the whole match.
- **Walls run past the nip to a stop.** Ending them at the nip left a ball-sized gap on all
  four sides exactly where the ball arrives at 0.8 m/s.
- **Bored for one POLLEN, not for NECTAR.** Sizing it to take the larger element made the
  bore 4.1 in, and two 2.8 in POLLEN wedge diagonally in 4.1 in. A single-file magazine is
  single file or it is not a magazine. NECTAR will not enter, which is correct.

**Indexer** (`stepTransfer`). Metered to one ball at a time, admitted only when the entry has
cleared by a ball's width — pushing every eligible ball at once sent two in through opposite
openings on the same step and jammed the bore solid. It steers to the nearer *opening*, not
to the tube's axis; aiming at the axis pressed balls into a solid side wall.

**Gate** (`gateCollider`). An actual plate across the bore, enabled and disabled by the servo.
The belt runs continuously and the gate is the release, so the magazine stays loaded between
shots. Gating the belt instead emptied the tube back into the bin every cycle.

**Nip** (`stepNip`). **This is the one place still modelled as an impulse rather than as
contact, and the reason is numerical.** The wheel turns at ~3500 rpm, so the nip opens and
closes in about 400 µs; resolving that as contact needs a timestep two orders of magnitude
below 1/240 s, and at 1/240 s the ball passes through the wheel between steps. Everything
that *decides* whether a shot happens is real: a ball has to physically be at the top of the
tube, the wheel has to be turning, and the energy comes out of the wheel's own inertia.

---

## 4. Shooter

**Exit speed** `v = k · ω · r_fly`, and the shot costs the wheel
`lossFactor · KE_ball / (I · ω)` — so the wheel visibly dips after each shot and the cycle
rate is limited by how fast it recovers (`tests/shoot.test.ts`).

**Scatter** (`flywheel.scatter`): Gaussian noise on elevation, yaw and speed at the moment of
release. This is the single largest cause of a wide group, and the Predictor says so.

**Flight** (`ballistics.ts`): explicit integration at 1/480 s with

- gravity,
- quadratic drag `½ρ C_d A v²`,
- Magnus lift `½ρ C_l A v²` along `ŵ × v̂`, with `C_l = clSlope · S` saturating at `clSMax`,
- spin decay per step.

`C_d` and `clSlope` are **guesses** and labelled as such — a 26-hole hollow ball is not a
smooth sphere.

**Aim** (`builtinTeleOp.ts` / `ShotLead.java`). The ball leaves with `v_exit·d̂ + v_robot`, so
the shot is solved for the velocity the **ball** must have in the field frame — which is the
table's answer as a *vector* — with the robot's own velocity subtracted from it. Three
components, three unknowns, so all three are solved:

```
horiz = S·cos(el) along the bearing        vert = S·sin(el)
mag   = |horiz·b̂ − v_robot|
                                azimuth = ∠(horiz·b̂ − v_robot)
                                el      = atan2(vert, mag)
                                speed   = hypot(mag, vert)
```

> **The vertical is part of the answer.** This solved the horizontal triangle only and left
> the hood at the table's angle, so the ball went out with a vertical of `mag·tan(el)` rather
> than `S·sin(el)`: the ground track exact, the hang time wrong. Closing at 0.4 m/s from 40 in
> that drops the exit speed from 5.28 to 4.09 m/s at a fixed 70° hood, the vertical from 4.97
> to 3.85 — and the ball **never reaches** the mouth's 1.46 m. Not a miss; a shot that cannot
> arrive. Retreating sailed over it the same way. `tools/leadcheck.ts` prints both, and the
> corrected lead is exact to the centimetre at every velocity and range it covers.

That also all but takes the flywheel out of it. Over ±0.8 m/s of closing speed at 40 in the
old lead swung the target 1283–3388 rpm against a wheel that slews 1102 rpm/s; this one asks
for 2241–2477, and gives the rest to a hood servo that is **commanded** rather than measured
and tracks 15 m/s² of radial acceleration against the flywheel's 1.2.

Only flight acceleration is left uncompensated: over a one-second flight the `a·t²` term is
small next to 1–2° of launch scatter. `transfer.leadLatency_s` is a different thing — it
predicts the velocity at *release*, and now that the wheel barely moves it is worth nothing
either way (`tools/movingfire.ts` measures 0.33 / 0.33 / 0.30 landed per second at τ = 0,
0.15, 0.3). It is kept because a real hood will lag in a way this one does not.

> The azimuth **must** be wrapped. `atan2` returns (−180, 180] and the heading is subtracted
> from it, so the result can land anywhere in (−540, 540). Unwrapped, a bearing of +90° came
> out as −270°, clamped to the turret's −120° limit, and the robot fired over the wall. Every
> azimuth crossing the ±180° seam did this. `tests/shotlead.test.ts` guards it.

**Turret axis**: trapezoidal motion profile with real velocity and acceleration limits, so a
137° swing takes most of a second. Firing on the *commanded* angle instead of the encoder
angle means firing at nothing.

**Shot table** (`tools/shottable.ts`): for each range, the hood angle whose feasible exit-speed
band is widest. The aperture is modelled as a near lip to clear and a far lip to stay under —
aiming at the mouth's centre is not enough, because a ball can pass through that point while
still climbing and clip the near lip.

**Calibration** (`robot.calibration`): `rangeTrim_in` and `turretTrim_deg`. The table is
looked up at `(range − rangeTrim)`, so a group landing 8 in long gets a trim of +8. The
Analysis tab computes both from a collected run and the **Apply calibration** button writes
them. Trims accumulate, because each run measures what is left after the last one.

---

## 5. HIVE — `packages/core/src/physics/hive.ts`

An over-centre see-saw on a revolute joint. At θ = 0 its CG is directly above the pivot, so
gravity's restoring torque is `m·g·r·sin θ` — about **0.62 N·m** at the ±30.04° stops. A ball
in the up CELL pushes back with `m·g·z`, where `z` is its horizontal distance from the pivot
axis. When the balls win, it goes over.

**Nothing scripts the tip.** There is no ball counter and no threshold. The Robot tab shows
the torque balance that produces it, per ball, with each ball's lever arm.

Measured onset: **12 POLLEN** or **8 NECTAR**, pooling at a 9–10 in lever arm.

A match starts with **empty CELLs**. The STEP stages six NECTAR inside them, which is the
CAD's display state and not a match start — left in, every match began with the HIVE 57% of
the way to tipping.

---

## 6. What the data tools measure

| Tool | Question it answers |
|---|---|
| `tools/mechcheck.ts` | does the mechanism work *as a mechanism* — does a ball get picked up, held, lifted and fired |
| `tools/collect.ts` | fire N shots across a range sweep; bias, spread, land rate, faults |
| `tools/collect.ts --calibrate N` | collect, apply the trim it implies, repeat — does the bias converge |
| `tools/audit.ts` | do balls gain energy, tunnel, jitter or go non-finite; does the robot drift |
| `tools/hivedrop.ts` | how many balls tip the HIVE |
| `tools/shottable.ts` | regenerate the shot table |
| Predictor tab | ∂(landing point)/∂(each variable) × that variable's 1σ — the error budget |

**The model is validated against itself.** A 30-shot run measures a downrange spread of
±11.7 in; the Predictor, from the same constants, predicts ±10.3 in. Agreement between a
measurement and a model derived independently of it is the reason the Predictor's advice is
worth acting on.

**Arrival, not resting place.** A shot's error is measured where the ball crossed the mouth's
height on the way down, not where it stopped rolling. A ball that drops an inch wide of the
lip and then bounces forty inches across the field was an inch out; scoring it as forty makes
the statistics measure the floor instead of the shooter. This alone moved the reported spread
from 40 in to 12 in.

---

## 7. Known approximations

Listed because they are the ceiling on how far this can be trusted, not because they are
fine.

1. **The nip is an impulse.** Section 3. Numerical, not laziness.
2. **`ball.Cd`, `ball.clSlope`, `ball.e_poly` are guesses.** They own about 40% of the
   predicted shot spread between them. These are the three numbers most worth measuring on a
   real ball.
3. **The CELL pocket is a reconstruction**, built from convex boxes around the CAD's CELL
   centroid. The CAD skin and the physics pocket agree to a few inches, not exactly. Deriving
   the pocket from the STEP's own faces instead of from summary bounding boxes is the fix, and
   it has not been done — an attempt using `up_floor_bbox_in` failed because that is the
   axis-aligned box of a plate tilted 30°, so its centre is not the plate's radial position.
4. **Chassis rotations are locked in roll and pitch.** The tyre model applies forces at the
   CG, so there is no roll moment to resolve, and leaving the axes free let solver noise tip
   the box over.
5. **Aero on balls inside the robot** is computed and negligible rather than special-cased.
