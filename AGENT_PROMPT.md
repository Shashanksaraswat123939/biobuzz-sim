# Working on the BIOBUZZ Simulator — a briefing for AI coding agents

Read this first. Then read `PLAN.md` — all of it, once, before touching code. This
briefing tells you what the project is, why it exists, how to work in it, and what
to do when the plan turns out to be wrong. It is not a set of orders; it is context
so that your judgement is good.

---

## 1. What FTC is

**FIRST Tech Challenge (FTC)** is a robotics competition for secondary-school students.
Teams design and build a robot that starts inside an 18-inch cube, then compete on a
roughly 12 × 12 ft field covered in soft foam tiles. Each match is 2½ minutes: a
30-second **autonomous** period where the robot runs code on its own, an 8-second
transition, and a 2-minute **tele-operated** period where two students drive it with
gamepads. Two robots form an **alliance** and play against another alliance.

The robot is controlled by a **REV Control Hub** — a small Android computer with
motor and servo ports, an IMU, and Wi-Fi — running an app built from the official
**FTC SDK** in **Java 8**. Teams write **OpModes** (classes extending `LinearOpMode`
or `OpMode`) that read sensors and gamepads and set motor powers in a loop. A second
Android device (or a phone) is the **Driver Station**, holding the gamepads.

A rulebook (the *Competition Manual*, ~170 pages) governs what you may build and how
scoring works. The parts of it that matter to this project are summarised in
`PLAN.md §16`; the manual itself is the final word.

## 2. What BIOBUZZ is

BIOBUZZ is the 2026–27 FTC game. In the middle of the field stands a frame carrying
two **HIVEs** (one per alliance), each a rocker on a pivot about 44 in above the floor.
Each rocker has two pockets (**CELLs**); one faces up at any time. Robots collect
balls from the floor and **launch** them into their up-facing CELL. When enough
balls land inside, the rocker **tips** over — the full CELL swings down, dumps its
balls, and the other CELL faces up. Every tip is worth 20 points, and two of the
three bonus ranking points are pure tip counts. There is no other legal way to tip
it, and you physically cannot reach it — the robot may not exceed 29 in tall.

Four **FLOWERs** (vertical tubes on the walls, 4 in opening at 21.5 in) take balls
dropped in from the top in the last minute; the alliance with the highest NECTAR in
a FLOWER "owns" it and scores everything in it. The balls are **POLLEN** (2.8 in,
25 g, 40 of them) and **NECTAR** (3.6 in, 41 g, 8 per alliance). They are hollow
holed plastic balls — light, drag-dominated, and not perfectly round.

The single most important unknown in the season is: **how many balls, placed where,
tip the hive?** The manual doesn't say. This simulator's first job is to answer it.

## 3. What we are building

A **3D physics simulator** of the BIOBUZZ field and a robot in it, with two halves:

- **The world** (TypeScript, Three.js, Rapier physics, browser): the field built from
  AndyMark's official CAD, 56 balls with aerodynamic drag and backspin lift, the two
  hive rockers on real hinge joints so they tip from ball contact, a box robot whose
  motion comes from motor torque through tyres, and the game's scoring and clock.
- **The brain** (Java 8): the robot's control code, written against the real FTC SDK
  interfaces so that the same files run on a Control Hub. In the sim they run on a
  laptop JVM against a fake hardware layer connected to the world by a WebSocket.

The robot is deliberately a **box**. It has: a mecanum drivetrain; a "line-in" intake
at the front that carries balls into a hopper; a transfer/feeder with a configurable
**cycle time** (for example one ball every 1.5 s); a **flywheel shooter** with an RPM
readiness gate ("cooldown"); a **turret** so the shooter can rotate to aim; an
optional hood for elevation; and the whole robot drives forward or backward to get
the range right. Motor torque and speed, wheel radius, gear ratios, masses, cycle
time, turret range and speed, flywheel inertia — all of it is configuration in
`config/robot.json`, read by both halves.

We are building it so the team can: see the hive tip and read the torque each ball
contributes; drive and practise against a live score and clock; develop and test the
Java autonomous and tele-op code months before the physical robot exists; sweep
unknown constants overnight; and then **copy the Java onto the real robot in an
afternoon**.

## 4. The one rule that matters most

**Everything under `java/teamcode/` must compile, unchanged, against the real FTC
SDK.** That folder is the deliverable that goes to the robot. Keep it Java 8, keep
it to the SDK surface listed in `PLAN.md` Appendix E, keep sim-only things behind the
small interfaces the plan names (`Localizer`, `TargetProvider`, `ConfigSource`). If
you need something from the SDK that the shim doesn't have, add it to the shim
first, write a line in `docs/DECISIONS.md`, and make sure the compile check against
the real SDK still passes. Never import from `simsdk`, `bridge` or `runner` inside
`teamcode`.

If you are ever unsure whether a piece of code belongs in the world or the brain,
ask: *does it decide what happens to matter, or what the motors are told?* The first
is the world; the second is the brain.

## 5. How to read `PLAN.md`

It is long. Read it in this order:

1. **§0** — orientation, non-negotiables, the two halves. Ten minutes.
2. **§1** — what the CAD file actually contains, with the numbers we extracted. Note
   the axis convention (Y is up) and the units trap (OCP imports STEP in millimetres).
3. **§3** — the architecture of the bridge, the SDK shim, the fake hardware, and the
   OpMode runner. This is the part that makes the port cheap; understand it before
   writing any Java.
4. **§7** — the robot: what each mechanism is, what is physics and what is Java, and
   the `robot.json` that describes it.
5. **§8–§10** — the world modules, the TeamCode design, rules/scoring/tools.
6. **§13** — phases and acceptance criteria. Find the phase you are working on.
7. Skim **§12** (tests), **§14** (port checklist), **§17** (known unknowns), and the
   appendices. Come back to them as needed.

`cad/cad-summary.json` has the geometry numbers in machine form; `cad/parts.json`
has every part. `docs/DECISIONS.md` is where deviations from the plan are recorded —
read it too, because the plan may already have been overtaken.

## 6. How to work here

**The plan is a starting point, not scripture.** It was written before any code
existed, from a CAD file, a rulebook, published datasheets and two other people's
simulators. Some of it will be wrong. When you find such a place:

- Don't force the plan. Check what's actually true (run the thing, print the number,
  read the library's current docs, look at the CAD part).
- Try the obvious alternative. If Rapier's joint motor won't hold as dry friction,
  try a small damping term, or a different iteration count, or the MuJoCo oracle to
  see what the "right" answer even looks like. If the AAR route for compiling against
  the real SDK is a mess, do the manual port and say so.
- Write down what you found and what you chose in `docs/DECISIONS.md` — a few lines:
  date, what the plan said, what happened, what you did instead, what it costs.
  Future agents will thank you.
- Keep the non-negotiables (`PLAN.md §0.3`). Everything else is yours to improve.

**Prefer working software over faithful fidelity.** A hinge that tips plausibly and
reports per-ball torque today beats a perfect damper model next month. Every physical
constant is a parameter with a `_source` tag; leave it as "guess" honestly rather
than pretending.

**Measure before you tune.** If a number matters, print it, plot it, or assert it.
The plan's phase table says what "done" means for each phase; use those checks.

**Small, checkable steps.** Get the field to load and pass its geometry assertions
before adding balls. Get one ball to fly and match the oracle before adding 56. Get
`AutoLeavePark` to score before building the shooter. The order in `PLAN.md §13` is
there because each step is the cheapest way to find out the next step's problems.

**When you're stuck for more than an hour on one thing**, write down the three things
you tried and what each one showed, then either pick a different approach or leave a
clear note and move to something independent. Don't sink a day into a detail the
plan marks as "calibrate later".

## 7. Things that are easy to get wrong

- **Units.** The world is SI internally; the CAD numbers are in inches in the plan;
  the FTC SDK speaks ticks, RPM and inches. Convert at boundaries only, and test the
  conversions (four field corners, both hive centres).
- **Axes.** CAD/world: Y up. FTC field coordinates: Z up, X toward the audience. The
  conversion lives in one file (`ftcFrame.ts`). Don't sprinkle sign flips elsewhere.
- **Concave colliders.** Physics engines turn a mesh into its convex hull unless told
  otherwise. A CELL pocket must be built from convex pieces or balls will float on an
  invisible lid. If balls hover, suspect this first.
- **Timestep.** Balls at 10 m/s need a small step (1/240 s) and continuous collision
  detection, or they tunnel through thin skins.
- **Determinism.** Lockstep runs must repeat exactly. Avoid wall-clock time inside the
  sim, seed every random draw, and be careful with `Math.sin/cos` in initialisation.
- **The bridge.** One frame of latency in lockstep is intended. A robot that "ignores"
  commands is usually a stale sequence number or a frame applied a step late.
- **Java 8 in teamcode.** No `var`, records, `List.of`, text blocks, or `switch`
  expressions. Newer Java is fine in `simsdk`, `bridge`, `runner`.
- **The hub's habits.** Sensor reads cost real time on a Control Hub; the fake hardware
  emulates bulk caching so code written here survives there. Don't defeat it.

## 8. Definition of done, per change

- Builds: `npm test` in the world, `./gradlew test` in `java/`, and the TeamCode
  compile check against the real SDK (or a note that the manual port was done).
- The phase's acceptance check in `PLAN.md §13` still passes, or you've moved it
  forward.
- New constants are in config with a `_source`; new SDK surface is in the shim and
  Appendix E; deviations are in `docs/DECISIONS.md`.
- Determinism test still green if you touched the world or the bridge.
- A short note of what you did and what you'd do next, so the next agent starts where
  you stopped.

## 9. Conventions

- TypeScript: strict; no DOM in `packages/core`; small pure functions; tests next to
  code or under `tests/`.
- Java: `teamcode` is Java 8 and SDK-only; subsystems expose `init(HardwareMap,
  RobotConfig)`, `update()`, `telemetry(Telemetry)`; no threads; no static mutable
  state. `simsdk`/`bridge`/`runner` may use modern Java.
- Config: JSON with schema; every physical key has a `_source` sibling.
- Commits: one topic each; message says what and why. Don't commit generated assets
  over a few MB — regenerate them.
- Don't "fix" physics by scripting outcomes. If the hive won't tip, find out why; a
  scripted flip defeats the purpose of the project.

## 10. Glossary

| Term | Meaning |
|---|---|
| Control Hub | REV's Android-based robot controller; runs the FTC SDK app |
| SDK | The FTC Robot Controller software; `com.qualcomm.robotcore.*` etc. |
| OpMode | A program the driver selects; `LinearOpMode` runs once, `OpMode` loops |
| TeamCode | The module of the SDK project where team code lives; here, `java/teamcode` |
| Shim | Our re-declaration of the SDK's classes/interfaces so TeamCode compiles in the sim |
| simsdk | Fake implementations of the SDK interfaces backed by the world over the bridge |
| Bridge | WebSocket exchange: world → SensorFrame, brain → ActuatorFrame, once per frame |
| Lockstep | World waits for the brain each frame; deterministic |
| HIVE / rocker / CELL | The tipping goal, its rotating body, and its two pockets |
| POLLEN / NECTAR | The two ball types |
| Tip | The rocker changing state; 20 points |
| Cycle time | Minimum time between two balls fed to the shooter (configurable) |
| Readiness gate | The flywheel is at target RPM for N loops; only then may a ball be fed |
| Turret | The rotating shooter mount used for azimuth aiming |
| Localizer / TargetProvider | Interfaces that hide where the robot's pose / the target bearing come from (sim now, sensors on the hub later) |
| Oracle | An independent implementation (MuJoCo / Python) used to cross-check the world's physics |

## 11. If you only remember three things

1. The world decides what happens; the brain decides what the motors are told; the
   bridge is the only thing between them.
2. `java/teamcode` must compile on the real hub, unchanged. Protect that.
3. When the plan and reality disagree, believe reality, try something sensible, and
   write it down.
