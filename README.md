# BIOBUZZ Simulator

A 3D physics simulator of the FTC 2026–27 BIOBUZZ field, with a robot whose control code is
real FTC Java — written against the Control Hub SDK so it ports by copying files.

**`docs/PHYSICS.md` is the one to read first**: what is modelled, which file implements it,
and what is still approximate. `docs/DECISIONS.md` is everywhere reality disagreed with the
plan (which is a lot, and the useful part). `docs/VARIABLES.md` is every tunable, generated
from `config/` so it cannot drift. The app's **Guide** tab documents every control.

---

## Run it

```bash
npm install
python tools/cad2assets.py    # once: turns the 35 MB STEP into assets/field.glb
npm run dev
```

Without that second step the app still runs — it falls back to procedural stand-in geometry
and says so in the console.

Open http://localhost:5173. Drive with **WASD**, turn with **Q/E**, `F` spins the flywheel,
`Space` fires, `H` hand-drops a POLLEN into your up CELL, `L` auto-loads the hopper, `R`
resets. Cameras on `1`–`5` (orbit / follow / top / first-person / muzzle). Full key list is in
the app's **Keys** tab. A gamepad works too.

**Does the HIVE actually rotate from the balls?** Yes — nothing scripts it. The Hive tab has a
meter showing how much of gravity's restoring torque the balls have overcome; at 100 % it goes
over. Measured onset: **12 POLLEN or 8 NECTAR**.

The five panels:

| Tab | What it shows |
|---|---|
| **Drive** | pose in FTC coordinates, range to the CELL, turret, hood, flywheel, hopper, cycle timer, battery |
| **Hive** | a meter for how close the rocker is to going over, its angle and rate, and **the per-ball torque breakdown with each ball's lever arm** — the thing this project exists for |
| **Robot** | mass, cycle time, last shot's exit speed and elevation |
| **Tune** | live sliders over the parameters that are still guesses (rocker mass, CG, pivot friction, damper, Cd, Magnus, restitution, tile friction, battery R) |
| **Keys** | controls |

## Run a real FTC OpMode against it

The Java is the deliverable. It runs on a laptop JVM against fake hardware, talking to the
world over a WebSocket.

```bash
bash java/build.sh
```

That compiles the SDK shim and `java/teamcode` with `--release 8` against **the shim only**,
then the sim-side code, then runs a self-check. If it builds there it builds on a Control Hub.

Three terminals:

```bash
npm run relay
```
```bash
npm run tool -- tools/headless.ts --match --lockstep --preload 6
```
```bash
java -cp "java/out;java/out-teamcode" sim.runner.Main --opmode "Auto One Tip"
```

(`--list` shows the OpModes, exactly as a Driver Station would.) To watch it instead of
reading numbers, skip the headless world, open the app and switch **Brain** to *Java*.

Current results, in lockstep:

| OpMode | Result |
|---|---|
| `Auto Leave + Park` | LEAVE + PARK, **8 pts** |
| `Auto One Tip` | LEAVE + PARK, **8 pts** — see below |

> `Auto One Tip` **does not currently score its shots**, and this table used to claim 28 pts
> for it. Run with `--echo` the brain is plainly working: it backs off to 31.8 in, reports
> `status: READY`, holds the hood at 75° and the wheel at 2293 rpm, and counts its hopper down
> from 6 to 2. The world records `shots 0` for the same run. So the Java decides to fire and
> the ball never leaves — the gap is in the feed across the bridge, not in the aim. The same
> aim code shooting the built-in brain's world scores normally (`tools/movingfire.ts`).

## The tools

```bash
npm run tool -- tools/hivedrop.ts          # how many balls tip the HIVE
npm run tool -- tools/landrate.ts          # land rate vs range
npm run tool -- tools/leadcheck.ts         # does the motion lead land the shot? (no scatter)
npm run tool -- tools/movingfire.ts        # shooting while moving, and while accelerating
npm run tool -- tools/shoterror.ts         # where a moving shot's error actually comes from
npm run tool -- tools/spincheck.ts --full  # single-wheel backspin vs a dual-wheel shooter
npm run tool -- tools/shottable.ts         # regenerate the shot table
npm run tool -- tools/hoodsweep.ts         # which hood range this robot needs
npm run tool -- tools/shootercheck.ts      # turret coverage, flywheel MOI, exit speed
npm run tool -- tools/flywheeltune.ts      # hub velocity-loop settling and ripple
python tools/cad2assets.py                 # tessellate the STEP -> assets/field.glb (~40 s)
node tools/cad2staging.mjs                 # regenerate ball staging from the CAD
node tools/genconstants.mjs                # robot.json + shot table -> Java constants
npm run tool -- tools/drivedemo.ts         # drive around and shoot from each stop
node tools/vars.mjs                        # regenerate docs/VARIABLES.md
npm test                                   # 84 tests
```

## What it currently says

- **12 POLLEN or 8 NECTAR tip the HIVE**, pooling at a 9–10 in lever arm, tip taking 0.88 s.
  The plan guessed 8–17; the community default was 8. See `docs/DECISIONS.md`.
- **The plan's default flywheel cannot reach the HIVE.** A 5203-1620 at 1:1 with k = 0.45 tops
  out at 3.7 m/s exit speed. `robot.json` now uses a bare 6000 RPM motor (13.6 m/s).
- **The plan's hood range [30°, 60°] is wrong for this game.** Every solution arrives still
  climbing and rebounds off the far wall. Extending to 85° lets the ball drop in at 85°:
  usable ranges go 19 → 25 and the best speed margin 8.0% → 14.8%.
- **Land rate is 38–50%** from a stationary robot, limited by launch scatter and the pocket's
  restitution (`e_poly`, still a guess) — not by aiming.
- **It shoots on the move, including while accelerating.** The lead solves the hood as well as
  the azimuth and the speed, so the ball leaves with the table's whole launch vector whatever
  the robot is doing, and the gate waits for the hood to get there. Measured through the gate
  the app actually enforces (`tools/movingfire.ts --gate`), landed per second and the hit rate
  of the shots taken:

  Error at the mouth, in cm, with the odometry noise modelled (`tools/shoterror.ts`):

  | | stopped | shuttling | closing | spinning | spinning + driving |
  |---|---|---|---|---|---|
  | downrange | 7 ±13 | 7 ±10 | 8 ±10 | 9 ±8 | 9 ±8 |
  | across | 2 ±14 | 2 ±22 | 1 ±4 | −2 ±9 | −2 ±4 |
  | wild shots | 1/79 | 1/68 | 0/21 | 0/37 | 0/50 |

  **Every moving case is now as accurate as standing still.** Spinning used to be −51 ±65 cm
  across with 13 shots in 78 going wildly wrong; shuttling was −18 ±49 downrange with 20 in
  108 wild. And through the gate the app enforces, 90–100% of the shots it takes now land.

  What it still will not do is shoot while **running away**: receding at 0.6 m/s the table's
  target rpm rises with the range faster than the wheel follows it, the readiness gate never
  latches, and with the gate open it manages two shots in 25 s with a 598 rpm error. That is a
  mechanical limit, not a policy one, and the gate is right to refuse.
- **`minLandProb` is set by what it costs, not by what it sounds like.**
  `tools/movingfire.ts --sweep` prices it in balls per second. At 0.70: 0.45 landed/s stopped
  with 90% of the shots taken landing, 0.55 closing at 100%, 0.35 shuttling at 100%. 0.80 and
  above takes the stopped case to zero. Re-measure it after any change to the shooter — it was
  0.85 against the old flywheel and a noiseless localizer, and both of those moved.
- **The flywheel is a real one now.** A bare 105 g grip wheel loses 210 rpm to every ball it
  throws, three and a half times the firing window, and no FTC shooter that works is built that
  way. With a 96 × 12 mm aluminium disc behind it (3.91e-4 kg·m², 0.34 kg) the dip is 70 rpm.
  Inertia rather than a second motor, because the robot is already at FTC's eight-motor limit.
- **The localizer is no longer an oracle.** It reports position, heading and velocity with real
  odometry error, and the brain filters the velocity before aiming on it — which is what a team
  does, and what the perfect estimate had been hiding.
- **Top speed matches the motor curve**: 62.8 in/s measured against 62 in/s hand-computed.

## Layout

```
config/          params.json, robot.json, motors.json  -- every physical constant, with _source
packages/core/   the world: physics, rules, no DOM. Runs in Node for tests and sweeps.
packages/render/ Three.js. Draws the tessellated CAD; "Colliders" shows what physics sees.
packages/ui/     app shell, input, panels, bridge client
packages/bridge/ relay.mjs -- forwards between the browser world and the Java brain
java/shim/       the FTC SDK surface, re-declared (PLAN.md Appendix E)
java/teamcode/   ← THE DELIVERABLE. Java 8, SDK-only, copies onto a Control Hub
java/simsdk/     fake DcMotorEx, Servo, IMU... backed by the bridge
java/bridge/     JSON + WebSocket, JDK-only
java/runner/     OpMode registry and the hub's lifecycle
tools/           experiments and generators
tests/           84 tests: geometry, hive, drivetrain, turret, shooting, lead, determinism
```

## Porting to the hub

1. Copy `java/teamcode/org/firstinspires/ftc/teamcode/**` into `TeamCode/src/main/java/...`.
2. Name the devices in the Robot Controller config as `config/robot.json → hardware`.
3. `RobotConstants.java` and `ShotTableData.java` are generated — no file parsing on the hub.
4. `Localizer`, `TargetProvider` and `BallCounter` all come from `hardwareMap.tryGet(...)`,
   which returns `null` on the hub. Each has a documented fallback; swapping in a Pinpoint or
   a Limelight is a one-line change.
5. `./gradlew :TeamCode:assembleDebug`.
