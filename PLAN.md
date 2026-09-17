# BIOBUZZ Physics Simulator — Build Plan (v2, Java-first)

A 3D, physics-first simulator of the FTC 2026-27 BIOBUZZ field, with a robot whose
control code is real FTC Java — written against the REV Control Hub SDK so it ports to
the hub by copying files, not by rewriting them.

This document is long on purpose. It is a plan, not a contract. Where reality
disagrees with a number or an approach here, trust reality, write down what you found
(`docs/DECISIONS.md`), and carry on. Words like **must** mark things the rules, the
physics, or the porting promise genuinely require. "Prefer", "suggested", "start with"
mark defaults you may change.

---

## 0. Orientation

### 0.1 What this is for

1. Watch the HIVE tip under real ball contact — how many balls, and **where they sit** in the CELL — and read the torque per ball.
2. Drive a box robot whose motion comes from motors, tyres and a battery, and read every state variable live.
3. Run **real FTC OpModes in Java** against the simulated robot: intake, transfer, turret, flywheel with an RPM readiness gate, drive-to-range aiming, autonomous routines.
4. Shoot balls with drag and Magnus and see whether they enter the up-facing CELL.
5. Score a full 2:30 match by the real rules (Table 10-2), with the timing gates.
6. Sweep unknowns overnight and hand the team numbers before the real robot exists.
7. **Port the Java to the Control Hub in an afternoon** when the robot exists.

### 0.2 What this is not

- Not connected to a physical Control Hub. The Java runs on a laptop JVM against a simulated hardware layer.
- No AprilTag detection yet. The robot's pose is ground truth from the physics (a `SimLocalizer` hands it to the Java as if it were an odometry computer).
- No opposing robots, no alliance partner. Leave room; don't build it.
- Not a replacement for measuring. Several constants are guesses until November — §15.

### 0.3 Non-negotiables

- **TeamCode compiles unchanged in both worlds.** Every `.java` under `java/teamcode/` must build against the sim's SDK shim *and* against the real FTC SDK in `FtcRobotController/TeamCode`. This is the porting promise; §3.3 and §14 say how it is kept.
- **TeamCode is Java 8.** The hub's SDK targets Java 8 (`minSdkVersion 24`, `targetSdkVersion 28`). No `var`, no records, no `List.of`, no streams-heavy code you wouldn't run on a phone-class CPU.
- **The physics core has no DOM and no rendering.** It runs in Node, in a Worker, or headless.
- **One robot description, two consumers.** `config/robot.json` defines the robot's specs; the physics builds the body from it and the Java reads its gear ratios, ticks-per-rev, and kinematics from the same file.
- **Every physical constant lives in config.** Nothing physical is hard-coded.
- **Determinism** in lockstep mode: same configs + same command log + same seed → identical run, checked by snapshot hash.
- **SI inside the physics core** (m, kg, s, rad). Inches appear only at the CAD boundary and in the UI. The Java side uses whatever units the FTC SDK idioms use (ticks, RPM, inches for field coordinates) — convert at the bridge.

### 0.4 The two halves

```
┌─────────────────────────────────────────┐        ┌──────────────────────────────────────┐
│  WORLD  — TypeScript, Three.js, Rapier  │  WS    │  BRAIN — Java 8                       │
│  field, balls, hive hinge, robot body,  │◄──────►│  teamcode/   your OpModes + subsystems │
│  mechanisms as physics, scoring, clock, │ 60 Hz  │  simsdk/     fake DcMotorEx, Servo…    │
│  rendering, gamepad, telemetry, tools   │        │  bridge/     WebSocket client           │
└─────────────────────────────────────────┘        │  runner/     OpMode lifecycle           │
                                                   └──────────────────────────────────────┘
                                                                    │ copy teamcode/ only
                                                                    ▼
                                                   ┌──────────────────────────────────────┐
                                                   │  REV Control Hub — real FTC SDK        │
                                                   │  same teamcode/, real motors           │
                                                   └──────────────────────────────────────┘
```

The physics side is the *world*: it decides what happens to matter. The Java side is the *brain*: it decides what the motors are told. The bridge carries actuator commands one way and sensor readings the other, and nothing else. If someone finds themselves putting game logic in the world or physics in the brain, stop and look at the boundary again.

### 0.5 Why this split and not one language

- The FTC hub runs Java. Writing the brain in anything else means a rewrite later, which is the thing this project exists to avoid.
- A browser 3D world with a deterministic WASM physics engine is far quicker to build and share than a Java 3D world, and the team already ships browser software.
- A clean sensor/actuator bridge is *also* how you'd later hang the sim off a real hub (hardware-in-the-loop). Designing it now costs nothing extra.

---

## 1. Ground truth from the CAD

The STEP file is AndyMark's full `am-5850 BIOBUZZ` field, exported from Onshape (AP242, 2026-09-09). It lives at `cad/field-cad-step.step` (35 MB, 993 leaf parts, 104 solids). `cad/analyse_step.py` walks it with OpenCASCADE (via `OCP`) and produces `cad/parts.json` (every part: path, bbox, volume, centroid) and `cad/report.txt`. `cad/cad-summary.json` has the headline numbers in machine form.

### 1.1 Units and axes — read before anything else

- The file declares metres, but **OCP imports STEP in millimetres**. `analyse_step.py` divides by 25.4. If you re-export with another tool, check a POLLEN's bbox: it must be 2.80 in / 71.1 mm.
- **Y is up.** X is the pivot-axis direction. Z points toward the audience/scoring sides.
- Origin is the field centre at tile-top level (tiles span Y = −0.589 … 0).
- Red hive is centred at X ≈ −12.74 in, blue at X ≈ +12.76 in. Both rockers share one pivot line.
- The FTC field-coordinate convention teams use in code (X toward the audience, Y to the left, Z up, origin at field centre) is different. The bridge converts; TeamCode only ever sees FTC coordinates in inches.

### 1.2 Field

| | Value | Note |
|---|---|---|
| Inside width | **141.35 in** (±70.674 to the glass) | The manual says "approximately 144". The CAD is 141.35. Use the CAD. |
| Tiles | **23.5 in**, 0.589 in thick, 36 of them | 60 cm mats are 23.62 in — closer to the CAD than real 24 in tiles are |
| Wall | glass 0.09 → 10.965 in; rail top 11.64 in | |
| Perimeter parts | `FTC Field Side Glass 11in` ×12, `am-2556a` rails, riveted panels, corner hinges | Static |

### 1.3 Pivot and frame

- **Pivot axis: the line Y = 43.95 in, Z = 0, parallel to X.** The aluminium stepped spacers (`am-5881`) sit exactly on it at X = ±12.76.
- Frame (`am-5854`): A-frame legs, top bar at Y = 41.4, ACM panels at Y 34–40, footprint X ±24.7 × Z ±19.5. Static.
- Goal Pivot Assemblies ×2 (`am-5873` brackets at X = ±12.2 / ±13.3, axle holders `am-5863`).
- **Dampers: four `am-5865 Blumotion 970A`** hydraulic soft-close pistons, two per hive, on `am-5864` holders at roughly (Y 41.0, Z ±1.97) and (Y 44.18, Z ±3.54). End-stop dampers; Blum publishes no force curve. Treat as a calibrated velocity-dependent damper that engages near each rest angle.

### 1.4 The rocker (one per hive)

Each rocker = 2 CELLs + ribs + skins + basket base tubes + fasteners, 173 parts, 103 in³ of material.

| | Red rocker (from CAD) |
|---|---|
| Bbox (in) | X −23.36…−2.13, Y 30.65…66.13, Z −25.19…20.06 |
| Volume centroid | (−12.74, 46.22, −1.31) in |
| **Mass, guessed by material** | **≈ 2.4 kg** (ribs as HDPE/PC 1.05, skins as polycarbonate 1.20, tubes/churro aluminium, fasteners steel) |
| **CG, guessed** | (−12.73, 45.78, −1.05) in → **1.83 in above the pivot, 1.05 in toward the down-CELL side** |
| Inertia about pivot (parallel-axis approx.) | ≈ 0.36 kg·m² |
| Gravity restoring torque at rest | ≈ 0.6 N·m |

**Important:** mass and CG depend on densities guessed from part names. Ribs and skins dominate and could be ACM, HDPE or polycarbonate. Treat mass as a range (roughly 1.5–3.5 kg) and CG offset as a range until someone weighs a real rocker or reads the Onshape materials. The sim must take these as parameters.

In the CAD's saved state the **audience CELL is up** (centroid Y 53.8, Z +11.3) and the scoring CELL is down (Y 39.1, Z −14.2). A CG slightly on the down-CELL side is consistent with gravity holding that state.

### 1.5 The CELL (pocket)

Nominal mouth 20 × 14 in, 12.04 in deep, pentagonal section, on arms tilted 30° (two independent sims derived the same numbers from this CAD; the manual gives 20 × 14 × 12). Up-CELL bbox in CAD: Y 43.95…66.13, Z 1.57…20.06. Mouth lip around Y 53.5, top around Y 65.6.

The pocket is built from: two `Red Goal Rib` (pentagon side plates), `Hive Goal Bottom Skin` (floor), `Hive Goal Top Skin`, `Hive Goal Back Skin`, a `Basket Base Tube`, two `10.5in Churro Lite` bars, and the AprilTag skin on the underside.

**Where balls settle** — the up-CELL floor spans Z 5.2…19.3 in and Y 47.5…60.1 in: tilted, low end nearest the pivot, closed by the back skin at Z 1.6…9.1. Balls roll **toward the pivot** and pool at roughly **Z ≈ 5–8 in** from the axis, not out at the mouth. That is a short lever arm.

### 1.6 Tip threshold: what the CAD suggests, and why it's a hypothesis

Gravity-only, using the guessed 0.6 N·m restoring torque and ignoring friction and dampers:

| Ball lever arm from pivot | POLLEN needed | NECTAR needed |
|---|---|---|
| 4 in | 25 | 15 |
| 6 in | 17 | 10 |
| 8 in | 13 | 8 |
| 10 in | 10 | 6 |
| 12 in | 8 | 5 |

The community default (ftc_demo) is 8 POLLEN-equivalents. The CAD floor geometry suggests balls pool at 5–8 in, which would mean more like 12–17 POLLEN by gravity alone. **Neither number is trusted.** Phase 1 exists to replace this table with a simulated one; November's measurement replaces that. Keep the sweep.

### 1.7 FLOWERs (×4)

Rings at Y ≈ 0.2 (lower, `Flower Layer X`), 4.34 (middle, `Layer B`), 21.4 (top, `Layer C`); backstop 22.4–22.65; four HIPS pipes; 4 in opening. Positions (X, Z) in inches: (−23.39, −69.77), (−69.77, 23.39), (69.77, −23.39), (23.39, 69.77) — one per wall, 23.4 in off the centreline. Scoring volume per the manual is between the top and middle rings (≈ 4.3 → 21.4 in).

### 1.8 Balls

| | Diameter | Mass | Count |
|---|---|---|---|
| POLLEN `am-5851` | 2.80 in / 71.1 mm | **0.055 lb = 24.9 g** | 40 |
| NECTAR `am-5852` | 3.62 in / 91.9 mm | **0.091 lb = 41.3 g** | 8 red + 8 blue |

Masses from AndyMark's spec tab; two independent sims use the same. Pickleball-style, 26 holes. The STEP places them at their **staged positions** — use those transforms for match staging.

### 1.9 What the STEP does not give you

Densities (1.4), damper force curves, tile friction, ball restitution, the rocker's exact rest angles (derive from the damper contact geometry, or measure).

---

## 2. Physical constants and where they came from

| Constant | Value | Source | Status |
|---|---|---|---|
| g | 9.81 | — | fixed |
| Air density ρ | 1.15 kg/m³ (Dubai hall) | estimate | config |
| Ball Cd | 0.45 | shot-sim (pickleball-derived, uncited) | **calibrate** |
| Ball Cl | 0.20 × spin ratio, cap at S = 1.0 | shot-sim | **calibrate** |
| Ball restitution on foam / polycarbonate | 0.45 / 0.7 | guess | **calibrate** |
| Ball rolling friction on tile | 0.02 | guess | calibrate |
| Tile μ (rubber wheel on foam) | 0.8–0.9 | typical | calibrate |
| Motor (goBILDA 5203) | free 312 / 435 / 1620 RPM variants; stall torque per datasheet; stall current ~9.2 A; 537.7 / 384.5 / 103.8 ticks/rev | goBILDA | fixed |
| Battery | 12 V NiMH 3000 mAh, R_int ≈ 0.08 Ω, ±1.2 V swing over a match | shot-sim + typical | calibrate |
| Flywheel inertia | Gecko 96 mm 1.21e-4; 72 mm 3.37e-5; steel 82 mm 1.65e-4; steel 60 mm 5.51e-5 kg·m² | shot-sim | fixed |
| Exit efficiency k | 0.45 single-wheel hood (0.30–0.50), 0.90 dual | shot-sim | **calibrate** |
| Energy loss per shot | 2.0 × ball KE (2.0–3.5) | shot-sim | **calibrate** |
| Scatter presets (σ angle, σ yaw, σ speed) | 0.5°/0.5°/1 %, 1°/1°/1.5 %, 2°/2°/3 % | shot-sim | config |
| Rocker mass / CG / I | §1.4 | CAD + density guess | **sweep** |
| Pivot friction torque | unknown, start 0.05 N·m | — | **sweep** |
| Damper coefficient | unknown | — | **sweep** |
| Hub loop period | 10–20 ms typical; hub commands ~2–3 ms each over UART | gm0 / SDK docs | config |

---

## 3. System architecture — the two halves in detail

### 3.1 Data flow, one frame

```
WORLD (physics, clock master)                          BRAIN (Java)
──────────────────────────────                         ────────────────────────
step ×4 at dt=1/240
build SensorFrame  ──────────── ws ─────────────────►  simsdk caches it (bulk read)
                                                        OpMode loop runs once
                                                        motors/servos set …
                                                        ◄──── ActuatorFrame ────
apply ActuatorFrame on next step
```

Two timing modes, both supported from day one:

- **Lockstep** (default for tests and autos): the world sends a `SensorFrame`, waits for the `ActuatorFrame`, then steps. Deterministic. Sim time is decoupled from wall time.
- **Free-run** (driver training): the world steps on its own 60 Hz schedule; the last `ActuatorFrame` persists; the brain runs as fast as it can. Realistic latency, not deterministic.

### 3.2 Coordinate and unit conventions at the bridge

- World → Brain: pose in **FTC field coordinates, inches and degrees** (X toward audience wall, Y toward the left wall from the audience, heading CCW from +X). Encoders in **ticks**, velocities in **ticks/s** (as `DcMotorEx.getVelocity()` returns), battery in **volts**, distance sensors in **inches or mm as configured**, IMU yaw in degrees.
- Brain → World: motor **power −1…1** or **velocity ticks/s** with the run mode, servo **position 0…1**, CR-servo power. Nothing else.
- The world's mapping from FTC coordinates to its own (Y-up, X pivot axis) lives in one file: `packages/core/src/field/ftcFrame.ts`. Test it with the four field corners and the two hive centres.

### 3.3 The FTC SDK shim (`java/ftc-sdk-shim`)

TeamCode must use the real SDK's package names, class names and method signatures, so that on the hub the real `RobotCore` satisfies them. In the sim, a **shim** module provides the same names with sim behaviour behind them.

**Allowed SDK surface** (Appendix E lists it precisely). Start with:

```
com.qualcomm.robotcore.eventloop.opmode.{LinearOpMode, OpMode, Autonomous, TeleOp, Disabled}
com.qualcomm.robotcore.hardware.{HardwareMap, DcMotor, DcMotorEx, DcMotorSimple, Servo, CRServo,
                                  IMU, VoltageSensor, DistanceSensor, ColorSensor, TouchSensor,
                                  Gamepad, PIDFCoefficients, DcMotor.RunMode, DcMotor.ZeroPowerBehavior}
com.qualcomm.robotcore.util.{ElapsedTime, Range}
com.qualcomm.hardware.lynx.LynxModule  (bulk caching mode only)
com.qualcomm.hardware.rev.RevHubOrientationOnRobot
org.firstinspires.ftc.robotcore.external.Telemetry
org.firstinspires.ftc.robotcore.external.navigation.{AngleUnit, DistanceUnit, YawPitchRollAngles}
```

Anything outside that list is a deliberate decision, recorded in `docs/DECISIONS.md`, and it must be added to the shim before TeamCode may use it.

**Two ways to keep the shim honest** — pick one, prefer the first, keep the second as a backstop:

1. **CI compile check against the real SDK.** Extract `classes.jar` from the season's `RobotCore`, `Hardware`, `FtcCommon` AARs (Maven Central, `org.firstinspires.ftc:*`, version matching the season's FtcRobotController release — v12.0 at the time of writing) plus `android.jar` (API 24) and compile `java/teamcode` against them with `compileOnly`. If it compiles there and against the shim, the shim is faithful enough. This runs on every PR.
2. **Periodic manual port.** Copy `java/teamcode` into a clone of `FtcRobotController/TeamCode` and run `./gradlew :TeamCode:assembleDebug`. Do this at every phase boundary regardless.

If someone finds the AAR route impractical, don't fight it for days — the manual port is acceptable as long as it is actually done.

### 3.4 simsdk — the fake hardware (`java/simsdk`)

Implementations of the shim interfaces backed by the bridge. Behaviours to get right, because TeamCode will rely on them exactly as it would on the hub:

- **`SimHardwareMap`**: `get(DcMotorEx.class, "fl")` resolves by the names in `config/robot.json → hardware`. Unknown name → throws, like the hub does.
- **`SimDcMotorEx`**:
  - `setPower(p)` in `RUN_WITHOUT_ENCODER` → power command.
  - `RUN_USING_ENCODER` → the world emulates the hub's velocity PID (a simple PIDF in the world, tuned to feel like a Control Hub: not instant, ~50–100 ms settle). `setVelocity(ticks/s)` and `setPower` both route through it.
  - `RUN_TO_POSITION` with `setTargetPosition` → position P-control emulated in simsdk (the hub does it on the hub firmware; here it's fine to do it Java-side), `isBusy()` semantics as on the hub.
  - `STOP_AND_RESET_ENCODER` → zeroes the offset.
  - `getCurrentPosition()` / `getVelocity()` return the **cached** values from the last `SensorFrame` (see bulk caching below). Velocity is quantised the way the hub's is (ticks per 20 ms window → scaled), with `getVelocity(AngleUnit)` derived from `ticksPerRev`.
  - `setDirection`, `setZeroPowerBehavior` (BRAKE → strong damping; FLOAT → coast), `getCurrent(CurrentUnit)` from the world's motor model.
  - `setPIDFCoefficients` stored and forwarded so the world's velocity loop uses them (or ignored with a warning if the team runs its own loop — both are common).
- **`SimServo` / `SimCRServo`**: position/power forwarded; the world models servo travel speed and range from `robot.json`.
- **`SimIMU`**: `getRobotYawPitchRollAngles()` from the world's chassis orientation, with a configurable latency (Beta8397 uses 175 ms; start lower, ~40 ms, make it a knob).
- **`SimVoltageSensor`**: battery volts from the world.
- **`SimDistanceSensor`**: ray-cast in the world from a named mount on the robot.
- **`SimGamepad`**: fields filled from the browser gamepad through the bridge, or from a scripted command log.
- **`SimTelemetry`**: `addData`/`update` go to the console and to the UI panel via the bridge.
- **`LynxModule` bulk caching**: `AUTO` → every sensor read within one loop returns the same cached frame; `MANUAL` → reads refresh only on `clearBulkCache()`; `OFF` → each read waits for a fresh frame (slow, as on the hub). Emulating this makes teams write code that survives the hub's real I/O costs.

### 3.5 The OpMode runner (`java/runner`)

`Main --opmode AutoTwoTip --robot config/robot.json --mode lockstep --ws ws://localhost:8765`

- Scans `org.firstinspires.ftc.teamcode` for `@Autonomous`/`@TeleOp` classes (reflection), like the hub's registry.
- Lifecycle exactly as on the hub: construct → `init()` / `runOpMode()` up to `waitForStart()` → (INIT loop) → START → run → STOP (`opModeIsActive()` false) → `stop()`. The world's match clock drives START and STOP (30 s AUTO, 8 s transition, 2:00 TELEOP) so an OpMode experiences the same lifecycle it will at an event.
- `sleep()`, `ElapsedTime` and `opModeIsActive()` use the **sim clock**, not wall time, so lockstep runs are repeatable.
- **Loop period emulation**: in lockstep, the runner lets the OpMode loop body run once per world frame (16.7 ms). Optional knob `loopPeriodMs` to emulate a slower hub loop (e.g. 25 ms with vision running) by skipping frames. In free-run, the loop runs as fast as it can, like a real hub loop.
- A watchdog: if an OpMode loop exceeds `maxLoopMs` (default 100), log it loudly. Hub OpModes that stall get killed by the SDK; the sim should at least shout.

### 3.6 Bridge protocol (`java/bridge`, `packages/worker/bridge.ts`)

WebSocket, JSON text frames, one message per world frame each way. Keep the schema in `schemas/bridge.schema.json` and validate in tests.

**SensorFrame** (world → brain):

```jsonc
{
  "type": "sensor", "seq": 1234, "t": 20.55,          // sim time, s
  "match": { "period": "TELEOP", "remaining": 99.45, "started": true, "stopped": false },
  "motors": { "fl": { "pos": 12034, "vel": 1180.5, "amps": 2.1 }, "fr": {...}, "bl": {...}, "br": {...},
              "intake": {...}, "transfer": {...}, "flywheel": {...}, "turret": {...} },
  "servos": { "hood": { "pos": 0.42 }, "gate": { "pos": 0.0 } },
  "imu":   { "yaw": 12.3, "pitch": 0.1, "roll": -0.2, "yawRate": 3.4 },
  "battery": { "volts": 12.31 },
  "distance": { "front": 42.0 },                       // inches
  "localizer": { "x": 12.4, "y": -30.1, "heading": 12.3, "vx": 5.0, "vy": 0.2, "omega": 1.1 },  // FTC frame, in & deg
  "gamepad1": { "left_stick_x": 0.0, "left_stick_y": -0.8, "right_stick_x": 0.1, "a": false, "b": false, /* … all fields */ },
  "gamepad2": { /* … */ },
  "game": { "upCellAzimuthDeg": 31.2, "upCellRangeIn": 88.0, "hiveTipping": false, "hopper": 3 }   // convenience, from ground truth
}
```

`game` exists so early OpModes can aim without a vision stack. On the hub it would come from Limelight/odometry; keeping it in a separate block makes that swap obvious.

**ActuatorFrame** (brain → world):

```jsonc
{
  "type": "actuator", "seq": 1234,
  "motors": { "fl": { "mode": "RUN_USING_ENCODER", "power": 0.8 },
              "flywheel": { "mode": "RUN_USING_ENCODER", "velocity": 2400.0, "pidf": [10, 0.5, 0, 12] },
              "turret": { "mode": "RUN_TO_POSITION", "target": 812, "power": 0.6 },
              "transfer": { "mode": "RUN_WITHOUT_ENCODER", "power": 1.0 } },
  "servos": { "hood": 0.55, "gate": 1.0 },
  "telemetry": [["flywheel rpm", "2380"], ["ready", "true"]],
  "log": ["…"]
}
```

**Control messages**: `{"type":"hello","robotConfigHash":…}`, `{"type":"reset","scenario":…}`, `{"type":"mode","lockstep":true}`, `{"type":"opmode","name":"…","state":"INIT|START|STOP"}`. The world may refuse `hello` if the robot config hash differs from the one it loaded — both sides must be building the same robot.

Latency: in lockstep, exactly one frame. In free-run, whatever the network gives (local: ~1 ms) plus a configurable artificial delay to imitate the hub's command latency (2–3 ms per command, ~10 ms per loop of commands).

### 3.7 Hub-realism knobs (all in `config/robot.json → hub`)

`loopPeriodMs`, `bulkCacheMode`, `imuLatencyMs`, `encoderVelocityWindowMs` (20), `commandLatencyMs`, `velocityPid` (the hub's built-in loop gains and settle time), `voltageNoise`. None of them change the physics; they change how the brain *sees* the physics. Keep them honest and the port will be boring, which is the goal.

---

## 4. Stack

| Piece | Choice | Version (2026-09-16) | Notes |
|---|---|---|---|
| Physics | **Rapier 3D (WASM)** `@dimforge/rapier3d` | 0.20.0 | deterministic, joints with motors, per-step forces, CCD; runs in a Worker and in Node |
| Rendering | **Three.js** | 0.186.0 | glTF, instancing, orbit cameras |
| World language / build | TypeScript + Vite | TS 7.0.2, Vite 8.3.0 | |
| Brain language | **Java 8** (TeamCode), Java 17 allowed in `simsdk`/`runner`/`bridge` | Gradle | Keep TeamCode on 8; the others may use newer features |
| Java WebSocket | `org.java-websocket:Java-WebSocket` (or JDK 11 `HttpClient` WS in the runner) | | small, dependency-free |
| Java JSON | Gson (`com.google.code.gson`) — the FTC SDK already ships Gson, so it's also fine to use in TeamCode for reading `robot.json` | | |
| Oracle | **MuJoCo (Python)** | latest | independent engine for the hive and one-ball flight; hinge `frictionloss` is real dry friction |
| CAD pipeline | Python + `OCP`/`cadquery` (installed), Blender 4.4, `gmsh` 4.15 | | STEP → per-part meshes, mass props, glTF |

**If Rapier turns out wrong** (joint dry friction unstable, ball stacks jitter), the fallbacks are Godot 4 + Jolt, then Unity + PhysX. The world's `Sim` interface (§5.2) is written so the engine is swappable; keep it that way. The bridge does not care which engine is behind it.

**If the shim route turns out wrong** (SDK drift is a constant fight), the fallback is compiling TeamCode directly against the extracted SDK `classes.jar` + `android.jar` and providing sim implementations of the SDK *interfaces* only — that is close to what Beta8397's simulator does. Record the decision.

**Why not fork an existing sim:** ftc_demo (MIT) is 2D and tips the hive on a mass threshold; biobuzz-shot-sim (MIT) is a single-ball solver; Beta8397's 3D sim is JavaFX + ODE4J. None has a hinged hive with contact and a hub-portable Java brain. Borrow constants, rules, run-mode emulation ideas and test patterns (see §18).

---

## 5. Repository layout

```
biobuzz-sim/
  PLAN.md  AGENT_PROMPT.md
  docs/DECISIONS.md            append-only: what changed vs the plan, why, and what was tried
  cad/                         STEP, analyse_step.py, parts.json, report.txt, cad-summary.json
  assets/                      field.glb, hive-red.glb, hive-blue.glb, flower.glb, robot-box.glb,
                               colliders/*.json, frames.json
  config/
    params.json                the world: physics, balls, hive, environment, match
    robot.json                 the robot: geometry, drivetrain, mechanisms, hardware names, hub knobs
    *.schema.json
  schemas/bridge.schema.json
  packages/                    ── WORLD (TypeScript) ──
    core/                      NO DOM. physics, rules, robot body + mechanisms, telemetry
      src/field/  geometry.ts staging.ts zones.ts loader.ts ftcFrame.ts
      src/physics/ world.ts aero.ts balls.ts hive.ts tyre.ts motor.ts battery.ts chassis.ts
                   intakeLine.ts hopper.ts transfer.ts turret.ts hood.ts flywheel.ts launcher.ts
      src/rules/  scoring.ts match.ts gates.ts
      src/robot/  robotBuilder.ts telemetry.ts hubEmulation.ts (velocity PID, encoder quantisation)
      src/io/     recorder.ts snapshot.ts rng.ts
    worker/                    hosts core; SAB bus to ui; bridge.ts (WebSocket server side)
    render/                    Three.js only
    ui/                        app shell, gamepad, panels, graphs, scenario picker
  java/                        ── BRAIN (Java) ──
    settings.gradle  build.gradle
    ftc-sdk-shim/              same package/class/method names as the FTC SDK (Java 8)
    simsdk/                    Sim* implementations of the shim interfaces
    bridge/                    WebSocket client, SensorFrame/ActuatorFrame POJOs, clock
    runner/                    Main: OpMode registry + lifecycle + CLI
    teamcode/                  ← THE PORTABLE PART. org.firstinspires.ftc.teamcode.*  (Java 8 only)
      opmodes/                 AutoLeavePark, AutoTwoTip, TeleOpMain, TuningFlywheel, TuningDrive…
      subsystems/              Drivetrain, IntakeLine, Transfer, Turret, Hood, Flywheel, Hopper
      control/                 MecanumKinematics, VelocityPIDF, FlywheelGate, ShotTable, DriveToRange,
                               TurretTracker, TrajectoryFollower, StateMachine
      config/                  RobotConfig (loads robot.json; falls back to constants on the hub)
      util/                    Units, Clock, Log
    hubport/                   scripts + checklist for copying teamcode/ into FtcRobotController
  oracle/                      MuJoCo: hive.xml, ball_flight.py, compare.py
  tools/                       cad2assets.py, sweep.ts, shotmap.ts, shottable.ts, report.ts
  scenarios/                   *.json
  tests/                       unit/ determinism/ oracle/ parity/ e2e/
```

### 5.2 World core API (engine-agnostic surface)

```ts
interface Sim {
  load(params: Params, robot: RobotSpec, scene: SceneAssets, seed: number): void
  step(actuators: ActuatorFrame, gamepads: GamepadFrame): void   // exactly one physics step
  sensors(): SensorFrame                                           // for the bridge
  snapshot(): Snapshot                                             // for render / telemetry / tests
  createSnapshotHash(): string
  reset(staging?: StagingOverride): void
}
```

Engine handles never leave `packages/core/src/physics`.

---

## 6. Asset pipeline (`tools/cad2assets.py`)

Goal: from the STEP, produce render meshes, collision pieces and named frames, all in metres, Y-up.

1. Load with OCP XCAF (as `analyse_step.py` does) to keep names and transforms.
2. Group leaves by role using Appendix A: `frame`, `rocker-red`, `rocker-blue`, `flower-1…4`, `perimeter`, `tiles`, `pollen[i]`, `nectar-red[i]`, `nectar-blue[i]`. Drop fasteners under a few mm³ from collision.
3. Render: tessellate at ~1.0–1.5 mm deflection, export one glTF per group. Balls: export one POLLEN and one NECTAR mesh, instance them.
   Measured on this STEP: the red-rocker region tessellates to **~373 k triangles at 1.5 mm** (628 k at 0.5 mm) — most of it fasteners and bearing fillets. Drop fasteners and decimate in Blender; aim for ≲ 80 k per rocker and ≲ 300 k for the whole static field. Collision meshes are separate and far coarser.
4. Collision:
   - Static field, frame, flowers: trimesh colliders (Rapier `trimesh`) or convex hulls per part.
   - **Rocker: convex pieces, not one hull.** The pocket is concave. Suggested per CELL: floor skin, two rib plates, back skin, top skin, base tube, two churro bars — each its own convex hull. Verify by dropping a ball: it must rest on the floor, not on an invisible hull across the mouth. If a hull bridges the mouth, split further (V-HACD / CoACD via `trimesh`, or manual boxes).
   - Balls: spheres. Robot: a box plus mechanism volumes (§7).
5. Frames → `assets/frames.json`: pivot (point + axis), per-rocker rest angle and mass properties (with the density table used), per-CELL mouth polygon/centre/normal in rocker-local coordinates, per-FLOWER ring heights and scoring box, staging transforms of every ball.
6. Sanity: reload and assert POLLEN 0.0711 m ± 1 %, pivot Y 1.1163 m ± 1 mm, inside width 3.590 m ± 5 mm, four flower openings at 0.546 m ± 5 mm.

If Blender is easier for the glTF half, use it (STEP via CAD Assistant/FreeCAD first); keep frames/mass-props in Python so it's reproducible.

---

## 7. The robot

A **box** with mechanisms, fully described by `config/robot.json`. The world builds the physics body and mechanism volumes from it; the Java reads the same file for kinematics, gear ratios and hardware names. Nothing about the robot is hard-coded on either side.

### 7.1 Geometry and drivetrain

- Chassis: box `length × width × height`, mass, `Izz` (or computed from the box), CG offset, ground clearance. Must fit the 18 in starting cube by default; the sim flags violations rather than preventing them.
- Four mecanum wheels at configurable positions, radius, roller angle (±45°), traction μ, mecanum efficiency η.
- Each wheel driven by a motor spec: `variant` (5203-312 / 435 / 1620, or explicit `freeRpm`, `stallTorque`, `stallA`, `ticksPerRev`), external `gearRatio` (1.0 default), direction.
- The world computes per-wheel force from motor torque through the tyre model (§8.4) and applies it to the box. Acceleration, velocity, slip and pull-under-acceleration are outputs.

### 7.2 Intake line ("line in")

A straight path from a front mouth to the hopper, like a typical FTC roller intake:

- `mouth`: a trigger volume at the front face (width, height, offset). A free ball inside it while `intake` motor power > `captureThreshold` for `captureDelay_s` is captured.
- Captured balls become kinematic, parented to the chassis, and travel along a line of length `intake.length_m` at `intake.speed_mps` (or proportional to motor speed) to the hopper. `intake.transit_s` is the derived time; expose it in telemetry.
- Reverse intake (negative power) ejects the front-most ball back onto the field as a free ball.
- Roller contact is **not** simulated; capture is a trigger. That is a deliberate fidelity cut.

### 7.3 Hopper

Queue with `capacity` (default 6). Balls in the hopper are rendered in a stack inside the box. Telemetry: count, colours, order.

### 7.4 Transfer (feeder) and cycle time

The transfer moves one ball from the hopper into the shooter when commanded. It has:

- `cycleTime_s` (**configurable, e.g. 1.5**): the minimum time between two successive feeds. The Transfer subsystem enforces it on the Java side; the world also enforces a physical `feedTransit_s` (ball leaves hopper → reaches wheel), default 0.15 s.
- A `gate` servo (or the transfer motor itself) — with `gate` closed nothing can reach the wheel even if the transfer motor runs; use whichever the team's real design has. Both are in `robot.json` as optional.
- Command semantics from the Java: `Transfer.feedOne()` → runs the transfer motor for `feedPulse_s` and honours `cycleTime_s`; `Transfer.hold()` stops it. The world moves the ball accordingly.

The overall per-ball cycle is therefore `max(cycleTime_s, flywheelRecovery) + feedTransit_s + intake supply`. Telemetry shows each term.

### 7.5 Turret (shooter yaw)

- A yaw joint on the chassis with `range_deg` (e.g. ±90 from forward), `speed_dps`, and actuator `type: "servo" | "motor"`. Servo → position 0…1 mapped to the range with travel speed; motor → encoder ticks with `RUN_TO_POSITION` semantics and `ticksPerDeg`.
- Muzzle position is defined relative to the turret pivot (`muzzleOffset_m`, `muzzleHeight_m`). The launch point rotates with the turret.
- The FTC rules cap actuators at 8 motors + 8 servos (R503); the sim doesn't enforce it, but the robot.json validator warns when the total exceeds it.

### 7.6 Hood (elevation) — optional

`hood.enabled`, `angleRange_deg`, servo mapping. If disabled, `fixedAngle_deg` is used and the shot solver only varies RPM and range.

### 7.7 Flywheel and shot

- `type: single | dual`, `I_fly`, `r_fly`, motor variant, external `gearRatio`, `k` (exit efficiency), `lossFactor`, `minRpmFrac` (readiness), `readySteps`.
- Physics: `I ω̇ = τ_motor(ω, V)·gearRatio − τ_friction`; on launch `v_exit = k·ω·r_fly + noise`, `ω −= lossFactor·KE_ball/(I·ω)`, backspin for single-wheel, chassis velocity added. Ball placed at the muzzle, released to physics with aero on.
- The world reports flywheel `vel` in ticks/s through the motor's encoder like any other motor; the Java computes RPM and readiness. **The readiness gate lives in Java** (`FlywheelGate`), because that is what ships to the hub.

### 7.8 Aiming: turret for azimuth, chassis for range, hood for elevation

The Java `ShotTable` (generated by `tools/shottable.ts`, §10.3) maps `range_in → (rpm, hoodPos, marginScore)`. The aiming behaviour in TeamCode:

1. `TurretTracker`: turn the turret so the muzzle bearing equals the bearing to the up-CELL mouth (from `game.upCellAzimuthDeg` now; from a localizer + known hive position later).
2. `DriveToRange`: if the current range is outside the table's best-margin band, drive forward/back (and strafe if needed) until it is; hold heading.
3. `FlywheelGate`: spin to the table's RPM (voltage-compensated feedforward + P), fire only when at speed for `readySteps` loops **and** the hive isn't tipping.
4. `Transfer.feedOne()` when everything says ready; honour `cycleTime_s`.

All four are plain Java classes with no sim dependencies, so they run on the hub as-is.

### 7.9 What is physics-side and what is Java-side

| Concern | World (TS) | Brain (Java) |
|---|---|---|
| Wheel force, slip, chassis motion | ✔ | |
| Motor torque curve, battery sag | ✔ | |
| Hub velocity PID emulation, encoder quantisation | ✔ (`hubEmulation.ts`) | |
| Inverse kinematics, drive controllers | | ✔ |
| Intake capture, transit, hopper | ✔ | commands only |
| `cycleTime_s` enforcement | mirrors as a physical minimum | ✔ authoritative |
| Flywheel dynamics | ✔ | readiness gate ✔ |
| Turret/hood joint motion | ✔ | targets ✔ |
| Shot solver (table) | generated here | consumed here |
| Scoring, clock, gates | ✔ | reads `match` only |

### 7.10 `config/robot.json` — initial values

```jsonc
{
  "name": "box-v1", "version": 1,
  "chassis": { "length_m": 0.43, "width_m": 0.43, "height_m": 0.30, "mass_kg": 14.0, "Izz_kgm2": null,
               "cgOffset_m": [0, -0.05, 0], "clearance_m": 0.02 },
  "drivetrain": {
    "type": "mecanum", "wheelRadius_m": 0.048, "wheelbase_m": 0.33, "track_m": 0.33, "rollerAngle_deg": 45,
    "mu": 0.85, "eta": 0.80, "rollingRes_N": 3.0,
    "motors": { "fl": { "variant": "5203-312", "gearRatio": 1.0, "reversed": false }, "fr": {...}, "bl": {...}, "br": {...} }
  },
  "intake":   { "motor": { "variant": "5203-1150", "gearRatio": 1.0 }, "mouth": { "width_m": 0.30, "height_m": 0.12, "depth_m": 0.06 },
                "captureThreshold": 0.3, "captureDelay_s": 0.25, "length_m": 0.35, "speed_mps": 1.2 },
  "hopper":   { "capacity": 6 },
  "transfer": { "motor": { "variant": "5203-312", "gearRatio": 1.0 }, "cycleTime_s": 1.5, "feedPulse_s": 0.25, "feedTransit_s": 0.15,
                "gate": { "enabled": true, "servo": "gate", "open": 1.0, "closed": 0.0 } },
  "turret":   { "enabled": true, "type": "motor", "motor": { "variant": "5203-117", "gearRatio": 2.0 }, "range_deg": [-90, 90],
                "speed_dps": 180, "muzzleOffset_m": 0.12, "muzzleHeight_m": 0.28 },
  "hood":     { "enabled": true, "servo": "hood", "angleRange_deg": [30, 60], "fixedAngle_deg": 45 },
  "flywheel": { "type": "single", "motor": { "variant": "5203-1620", "gearRatio": 1.0 }, "I_fly_kgm2": 0.000121, "r_fly_m": 0.048,
                "k": 0.45, "lossFactor": 2.0, "minRpmFrac": 0.97, "readySteps": 3 },
  "sensors":  { "imu": { "latencyMs": 40 }, "distance": [ { "name": "front", "mount_m": [0.2, 0, 0.1], "dir": [1, 0, 0], "unit": "in" } ],
                "localizer": { "source": "groundTruth", "noise": { "xy_in": 0.0, "heading_deg": 0.0 } } },
  "hardware": { "fl": "motor0", "fr": "motor1", "bl": "motor2", "br": "motor3", "intake": "motor4", "transfer": "motor5",
                "flywheel": "motor6", "turret": "motor7", "hood": "servo0", "gate": "servo1", "imu": "imu", "battery": "Control Hub" },
  "hub":      { "loopPeriodMs": 16, "bulkCacheMode": "AUTO", "imuLatencyMs": 40, "encoderVelocityWindowMs": 20,
                "commandLatencyMs": 2, "velocityPid": { "p": 10, "i": 0.5, "d": 0, "f": 12, "settleMs": 80 }, "voltageNoise_V": 0.05 },
  "limits":   { "startingCube_in": 18, "expansion_in": [18, 24, 29], "maxMotors": 8, "maxServos": 8 }
}
```

The Java `RobotConfig` reads exactly this file (Gson). On the hub, `robot.json` goes into the app's assets or is transcribed into `RobotConstants.java` by `hubport/gen_constants.py` — either is fine; the important thing is that the numbers have one origin.

---

## 8. World physics modules

Each: purpose, model, config keys, tests, and what to do if it misbehaves.

### 8.1 `field/` — geometry, zones, staging

Load `assets/*.glb` + `frames.json`; static colliders; zones as axis-aligned boxes (two LOADING ZONEs 23 × 11 in against the wall, two GARDENs 23 × 2 in, four FLOWER scoring volumes, two CELL interior volumes per hive attached to the rocker). Staging from the STEP transforms, with an override file. `ftcFrame.ts` converts to and from FTC coordinates. Tests: §6 sanity; zone corners vs manual figures; four field corners and both hive centres through `ftcFrame` and back.

### 8.2 `balls.ts` + `aero.ts`

56 dynamic spheres; per-ball `d`, `m` sampled at spawn (`ball.dVar`, `ball.mVar`). Per step, for airborne balls:
```
Fd = −½ ρ Cd A |v| v
S  = |ω| r / |v|;  Cl = clSlope · min(S, clSMax)
Fm =  ½ ρ Cl A |v| (ω̂ × v)
ω̇ = −spinDecay · ω
```
Material pairs with their own restitution/friction. CCD on. Tests: one-ball flight vs the oracle (landing within 0.3 %); drop test vs `e`. If balls tunnel through the CELL floor, halve `dt` for airborne balls or thicken the floor collider.

### 8.3 `hive.ts` — the hinge

- Two rockers, rigid bodies with mass properties from `frames.json` (overridable: `hive.massKg`, `hive.cgOffset_m`, `hive.Ipivot_kgm2`).
- Rapier revolute joint on the pivot line, axis X, anchored to the fixed frame. Limits at the two damper-contact angles (start ±30° about the CAD rest angle; adjust so the dampers just touch at rest).
- **Dry friction**: Rapier joints have no `frictionloss`; emulate with a joint motor at target velocity 0 and `maxForce = hive.frictionTorque_Nm`. If it chatters at 1/240, add small damping or reduce iterations. MuJoCo does it natively — that's the reference.
- **Damper**: within `damperEngage_deg` of a limit, moving toward it, `τ = −damperC · ω`.
- **Tip detection** (manual §10.5.1): rocker crosses mid-angle **and** far limit contact event. Latch, count, scoring event, NECTAR release.
- **Per-ball torque telemetry**: for every ball–rocker contact, `((p − p_pivot) × F) · x̂`, summed per ball, exposed in the snapshot. This is the feature the project exists for.
- Tests: empty rocker stays at rest; monotonic depth → fewer balls; onset count and ω(t) vs `oracle/hive.xml` over a small (mass, CG, friction) grid.

### 8.4 Drivetrain physics: `motor.ts`, `battery.ts`, `tyre.ts`, `chassis.ts`, `hubEmulation.ts`

- Motor: `τ = τ_stall·(1 − ω/ω_free(V))·(V/12)`, current, encoder ticks; optional thermal integrator.
- Battery: `V = Voc(SoC) − Rint·ΣI`; `SoC −= ΣI·dt/C`.
- Tyre (mecanum): per wheel `F = clamp(τ·gearRatio/r, ±μN_i)·η` along the roller-permitted direction; weight transfer from last step's acceleration; rolling resistance; small body drag. References: Matsinos (arXiv 1211.2323); 2023 orthotropic-friction model.
- Chassis: one Rapier box body from `robot.json`; collides with walls, frame, flowers.
- `hubEmulation.ts`: the hub's built-in velocity PIDF for `RUN_USING_ENCODER` (gains from `robot.json → hub.velocityPid`), encoder velocity quantisation over `encoderVelocityWindowMs`, BRAKE/FLOAT behaviour, command latency. This is where "feels like a Control Hub" lives; keep it separate from the true physics so it can be turned off for pure-physics experiments.
- Tests: straight-line acceleration vs hand-computed curve; strafing pull under weight transfer; top speed drops with battery; velocity PID settles in ~`settleMs`.

### 8.5 Mechanism physics: `intakeLine.ts`, `hopper.ts`, `transfer.ts`, `turret.ts`, `hood.ts`, `flywheel.ts`, `launcher.ts`

As specified in §7. Each reads its sub-block of `robot.json`, exposes its state in the snapshot, and reports through the corresponding motor/servo entries in the `SensorFrame`. Tests: intake captures a ball rolled into the mouth and not one beside it; transfer honours `feedTransit_s`; turret slews at `speed_dps`; flywheel dip/recovery vs the first-order estimate; launch places the ball at the rotated muzzle.

---

## 9. TeamCode design (`java/teamcode`, Java 8, hub-portable)

### 9.1 Structure

```
org.firstinspires.ftc.teamcode
  config/     RobotConfig.java        loads robot.json (Gson) → typed fields; RobotConstants.java fallback
  util/       Units.java, Clock.java (wraps ElapsedTime), Log.java (Telemetry + optional logcat-safe logging)
  control/    MecanumKinematics, VelocityPIDF (voltage-compensated), FlywheelGate, ShotTable (CSV in assets),
              DriveToRange, TurretTracker, TrajectoryFollower (simple pure-pursuit or straight-line + turn), StateMachine
  subsystems/ Drivetrain, IntakeLine, Transfer, Turret, Hood, Flywheel, Hopper  (each: init(HardwareMap), update(), telemetry())
  opmodes/    AutoLeavePark, AutoOneTip, AutoTwoTip, TeleOpMain, TuningFlywheel, TuningDrive, TuningTurret
```

Every subsystem takes a `HardwareMap` and a `RobotConfig` in `init`, holds SDK interfaces only, and has an `update()` called once per loop. OpModes compose subsystems and call `update()` in their loop. No threads. No static mutable state. `LynxModule` bulk caching `AUTO` set in `init`.

### 9.2 Key classes

- **`FlywheelGate`**: `setTargetRpm(r)`; `update(rpmMeasured, volts)` computes `power = (kS + kV·target)·(12/V) + kP·(target − rpm)`; `isReady()` true after `readySteps` consecutive loops within `tolRpm`; `fired()` resets readiness so the next feed waits for recovery. Exposes `predictedReadyIn()` for the predictive feed (feed `feedTransit_s` early).
- **`Transfer`**: `feedOne()` honours `cycleTime_s` (from config) and only acts if `FlywheelGate.isReady()` when `requireReady` is set; `gate` open/close; telemetry of time-since-last-feed.
- **`TurretTracker`**: target bearing → turret position (servo pos or motor ticks); clamps to range; slew-rate limited by `speed_dps`.
- **`DriveToRange`**: given range and heading error, outputs chassis (vx, vy, ω) to bring range into `[rMin, rMax]` from the `ShotTable`'s best band; stops when inside; integrates with `Drivetrain.driveFieldCentric`.
- **`ShotTable`**: loads `shottable.csv` (`range_in, hoodPos, rpm, margin`), interpolates, returns best-margin band. Generated by the world (§10.3). On the hub the same CSV lives in assets.
- **`Drivetrain`**: field/robot-centric mecanum drive, `VelocityPIDF` per wheel (or `RUN_USING_ENCODER` with the hub's loop — both supported by config), pose from a `Localizer` interface with two implementations: `GroundTruthLocalizer` (sim, from `SensorFrame.localizer`) and later `PinpointLocalizer`/`LimelightLocalizer` (hub). Only the constructor changes at port time.

### 9.3 OpModes (initial set)

| OpMode | What it does | Why it exists |
|---|---|---|
| `AutoLeavePark` | drive off the wall, return to LOADING ZONE | 16 pts + SWARM RP for the alliance; the first thing that must work |
| `AutoOneTip` | leave, spin up, aim, fire the 4 preloaded POLLEN, park | the second thing |
| `AutoTwoTip` | above plus collect from the GARDEN and shoot again | stretch |
| `TeleOpMain` | field-centric drive; intake on triggers; auto-aim toggle (turret track + drive-to-range); fire button honours the gate; NECTAR endgame helpers | driver training |
| `TuningFlywheel` | sweep RPM, log dip/recovery, print `kS/kV` fit | calibration on the sim and later on the hub |
| `TuningDrive` | step responses, log encoder velocity, print PIDF suggestions | same |

### 9.4 Portability rules for TeamCode (checked by a lint script in CI)

- Java 8 syntax only. No `java.nio.file`, `java.awt`, `javax.*`, `java.net`, threads/executors, `System.exit`. File access only through the SDK's `AppUtil` (add to the shim if needed) or not at all — prefer reading `robot.json` once in `init` via an `InputStream` supplied by a tiny `ConfigSource` interface with sim and hub implementations.
- No imports from `simsdk`, `bridge`, `runner`, or anything not in Appendix E.
- No reliance on `SensorFrame.game.*` in code meant for the hub *without* an interface boundary: `TargetProvider` with `SimTargetProvider` (uses `game`) and later `LimelightTargetProvider`.
- Loop bodies must return within a few ms of work; heavy maths goes into tables generated offline.

---

## 10. Rules, scoring, tools

### 10.1 `rules/` — scoring, match, gates (world side)

Encode Table 10-2 verbatim:

| Event | AUTO | TELEOP | When assessed |
|---|---|---|---|
| HIVE TIP | 20 | 20 | on the tip event; tips completed before TELEOP count as AUTO |
| LEAVE | 3 | — | end of AUTO |
| PARK | 5 | 5 | end of AUTO / end of match |
| element in up-CELL | — | 2 each | **end of match only** |
| Bottom NECTAR bonus | — | 5 | end of match; lowest scoring NECTAR |
| element in owned FLOWER | — | 2 each | end of match; owner = highest NECTAR |
| element in GARDEN | — | 1 each | end of match |
| RPs | SWARM (LEAVE+PARK ≥ 16), POLLINATOR 1 (≥ 4 tips), POLLINATOR 2 (≥ 7 tips), WIN 3, TIE 1 | | |

Gates: FLOWER scoring locked until 60 s remain (G410 — flag a MAJOR FOUL, still count); NECTAR release one per own-colour tip, all at 60 s (G426); shooting at the down-CELL/frame flagged (G417), not blocked. Match: staging → 30 s AUTO → 8 s transition → 120 s TELEOP → freeze → assess → reset; sim clock.

**Open rules questions to keep visible in the UI** (ask in the Q&A on 28 Sept): bottom-NECTAR bonus one-per-flower or one-per-colour; whether NECTAR entitlements bank across tips.

### 10.2 Telemetry, recording, render, UI

Snapshot: pose/vel/accel (field + robot frames); per wheel command/ω/τ/F/N/slip; per motor current; battery V/SoC; flywheel ω/ready/shots; hopper; intake line contents and transit; transfer time-since-feed and cycle limit; turret angle/target; hood; last shot (v_exit, θ, spin, result); hive angle/ω/α/net torque/per-ball torques; match period/clock/scores/tips/RPs; zone occupancy; the Java's telemetry lines.
Recorder: CSV at 60 Hz + JSON event log; replay from the command log. Render: instanced balls, rocker driven by joint transform, robot box with turret and hood as child groups, cameras (orbit, top-down ortho, follow, muzzle POV). UI: HUD, telemetry table, graph strip, hive torque breakdown, shot-map overlay, params/robot.json editors ("apply on reset"), OpMode picker that talks to the Java runner.

### 10.3 Tools

- `sweep.ts`: headless world + optional headless Java runner over scenario × parameter grid, N seeds → CSV.
- `shotmap.ts`: field heat map of shot margin for the current robot (muzzle height, hood set) — for drivers.
- `shottable.ts`: for the current robot, `range_in → (hoodPos, rpm, margin)` at a fine range step, using the same integrator as the oracle → `java/teamcode/assets/shottable.csv`. **This is the artefact that makes the hub port cheap**: the solver is precomputed; the hub only interpolates.
- `report.ts`: summaries.

---

## 11. Config (`config/params.json`) — the world

```jsonc
{
  "sim":     { "dt": 0.0041667, "substepsPerFrame": 4, "solverIterations": 8, "ccdOnBalls": true, "seed": 1, "lockstep": true },
  "env":     { "rho": 1.15, "g": 9.81, "tileSize_m": 0.5969, "fieldInside_m": 3.590, "tileMu": 0.85 },
  "ball": {
    "pollen": { "d_m": 0.0711, "m_kg": 0.0249, "dVar": 0.0015, "mVar": 0.002 },
    "nectar": { "d_m": 0.0919, "m_kg": 0.0413, "dVar": 0.002,  "mVar": 0.003 },
    "Cd": 0.45, "clSlope": 0.20, "clSMax": 1.0, "spinDecay": 0.6,
    "e_foam": 0.45, "e_poly": 0.70, "e_ball": 0.80, "rollMu": 0.02, "mu": 0.5
  },
  "hive": {
    "massKg": 2.4, "cgOffset_m": [0.0, 0.0465, -0.0267], "Ipivot_kgm2": 0.36,
    "restAngles_deg": [-30, 30], "frictionTorque_Nm": 0.05,
    "damperC_Nms": 2.0, "damperEngage_deg": 6, "limitRestitution": 0.05,
    "ballsToTipOverride": null
  },
  "battery": { "capacity_Ah": 3.0, "Rint_ohm": 0.08, "VocBySoC": [[1.0,13.0],[0.8,12.6],[0.5,12.2],[0.2,11.6],[0.0,10.8]] },
  "match":   { "auto_s": 30, "transition_s": 8, "teleop_s": 120, "flowerUnlock_s": 60 }
}
```

Every key gets a `"_source"` sibling in the schema (measured / cited / guessed). `null` on `ballsToTipOverride` means "let the physics decide".

---

## 12. Testing, determinism, parity

- **Determinism** (`tests/determinism`): same scenario, same seed, 3 runs → identical `createSnapshotHash()` at steps 100/1000/10000, in lockstep with the Java runner attached.
- **Parity** (`tests/parity`): the same OpMode run (a) with the shim's velocity PID and (b) with the team's own `VelocityPIDF` must produce the same *behaviour class* (reaches the same target within tolerance) — this catches TeamCode that secretly depends on sim-only behaviour.
- **Shim faithfulness**: CI compiles `java/teamcode` against the extracted real SDK jars (§3.3) — or the manual port at phase boundaries.
- **Oracle**: `oracle/ball_flight.py` and `oracle/hive.xml` (MuJoCo, `frictionloss`); `compare.py` runs the same cases through the world via a small Node CLI. Targets: flight landing within 0.3 %; hive onset ball-count identical; tip duration within 15 %.
- **Geometry**: every number in §1 asserted against loaded assets.
- **Scoring**: scripted event lists → totals and RPs; each gate positive and negative.
- **Bridge**: schema validation of every frame in tests; a replayed command log must reproduce a recorded telemetry CSV bit-for-bit in lockstep.
- **E2E**: headless Chromium + headless Java runner play a full match from `AutoTwoTip` then a scripted TeleOp; assert final score and no NaN in any snapshot.
- **Performance**: 56 balls + 2 rockers + robot at 240 Hz in a Worker should leave 60 fps on a mid laptop with the Java attached. If not: sleep resting balls, fewer iterations for static contacts, or 1/120 for the drive and 1/240 only while a ball is airborne.

---

## 13. Phases and acceptance

| Phase | Build | Done when |
|---|---|---|
| **0** | `cad2assets.py`, `frames.json`, `ftcFrame.ts`, sanity tests | assets load; §6 sanity passes; corners round-trip through `ftcFrame` |
| **1 — the hinge** | rockers on joints, balls with aero, per-ball torque telemetry, MuJoCo hive oracle | drop POLLEN by hand at chosen depths; onset table produced; Rapier and MuJoCo agree on onset count; ω(t) plotted |
| **2 — the box robot (world side)** | chassis, motors, battery, tyres, `hubEmulation`, gamepad drive from the browser, telemetry panel | drive it; slip flags; battery sag; straight-line test matches the hand curve |
| **3 — the brain** | shim, simsdk, bridge, runner; `RobotConfig`; `Drivetrain`; `AutoLeavePark`; `TeleOpMain` (drive only) | `AutoLeavePark` scores LEAVE + PARK in lockstep, deterministically; CI compile-against-SDK green |
| **4 — mechanisms** | intake line, hopper, transfer with `cycleTime_s`, turret, hood, flywheel, launcher (world); `IntakeLine`, `Transfer`, `Turret`, `Hood`, `Flywheel`, `FlywheelGate` (Java) | from six marked spots, land-rate vs RPM curve; RPM dip/recovery visible; feeding honours cycle time and the gate |
| **5 — aiming and match** | `shottable.ts` → `ShotTable`; `TurretTracker`, `DriveToRange`; scoring, clock, staging, gates, NECTAR release, HUD; `AutoOneTip` | `AutoOneTip` tips the hive in a lockstep run; a full 2:30 scores correctly vs a hand count |
| **6 — tooling** | sweeps, oracle CI, e2e, recorder/replay, shot map overlay, `AutoTwoTip` | `ballsToTip` 3–17 sweep → tips per match CSV; `k` sweep → land rate; replay reproduces CSV |
| **7 — port rehearsal** | `hubport/` scripts; manual port into a clean FtcRobotController checkout; `TuningFlywheel`/`TuningDrive` run there (no robot needed to compile) | APK builds with the copied TeamCode and no edits |
| **8 — calibration** | November: plug measured numbers in (§15); swap `GroundTruthLocalizer` for the real one | sim predictions vs bench logs within stated tolerances |

Phase 1 before any robot. Phase 3 before Phase 4 — the brain skeleton and the porting check should exist before there is much brain to port.

---

## 14. Hub port checklist (`java/hubport/`)

What "port in an afternoon" concretely means:

1. Clone the season's `FtcRobotController`. Copy `java/teamcode/org/firstinspires/ftc/teamcode/**` into `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Copy `teamcode/assets/*` (`robot.json`, `shottable.csv`) into `TeamCode/src/main/assets/`.
2. Provide the hub `ConfigSource` (reads assets via `AppUtil`/`Context`) — a 20-line class that lives in `hubport/` and is copied alongside; or run `hubport/gen_constants.py` to bake `robot.json` into `RobotConstants.java` and skip file reading entirely.
3. In the Robot Controller app's hardware configuration, name devices exactly as `robot.json → hardware` (`motor0…`, `servo0…`, `imu`). Set motor types to the matching goBILDA variants so the hub's built-in velocity PID has the right ticks/rev.
4. Replace `GroundTruthLocalizer` with the real one (Pinpoint/odometry/Limelight) and `SimTargetProvider` with the real target source. Both are single-constructor swaps by design.
5. Build: `./gradlew :TeamCode:assembleDebug`. Deploy over Wi-Fi or USB.
6. Run `TuningDrive` and `TuningFlywheel` first; feed the numbers back into `robot.json`; re-generate `shottable.csv` with `tools/shottable.ts`; redeploy.
7. Keep a `docs/PORT_LOG.md` of anything that needed a change. Anything that needed a change is a bug in the shim or a portability-rule miss — fix it upstream so the next port is boring.

---

## 15. Calibration protocol (November, with real parts)

| Measure | How | Feeds |
|---|---|---|
| Ball mass, 10 each | kitchen scale | `ball.m_kg`, `mVar` |
| Ball diameter, 10 each | calipers | `d_m`, `dVar` |
| Restitution on foam / polycarb | drop 1 m, film 60 fps | `e_foam`, `e_poly` |
| Rolling stop distance | roll at known speed | `rollMu` |
| Exit velocity vs RPM | fixed hood, sweep RPM, film landings | `k`, then fit `Cd` |
| Second hood angle | repeat | confirms `Cd`/`Cl` |
| Flywheel dip and recovery | `TuningFlywheel` on the hub, 100 Hz encoder log, 5 shots | `lossFactor`, `I_fly`, `kS/kV` |
| Drivetrain accel/decel | `TuningDrive` on the hub | `mu`, `eta`, `rollingRes`, PIDF |
| Battery sag | log V through a 2:30 practice match | `Rint`, `VocBySoC` |
| Hub loop time and IMU latency | log `ElapsedTime` per loop and a step response | `hub.*` |
| **Rocker mass and CG** | weigh one; balance-point test | `hive.massKg`, `cgOffset_m` |
| **Tip onset by placement** | count balls into the real hive: pooled at back vs pushed to the mouth | validates §8.3 |

Each measurement replaces one `"_source": "guess"` with `"measured"`.

---

## 16. Rules that shape the sim

- §9.1: the CAD is the official field; tolerance ±1 in. Use CAD numbers over the manual's rounded ones.
- §10.5.1: TIP = state change + far damper contacts the frame. Shooting at a tipping hive may disrupt it.
- G410: no NECTAR into a FLOWER until 60 s remain (MAJOR FOUL each).
- G417: only launching into the up-CELL may move the hive; deliberate strikes on the down-CELL/frame are fouls; misses are excused.
- G426/G427: NECTAR enters via the human player into the LOADING ZONE, one per own-colour tip, all at 60 s; must touch the tile first.
- R102/R105: 18 in cube start; 18 × 24 × 29 in expansion, mechanically limited. Sim checks and flags.
- R503: 8 motors + 8 servos total. `robot.json` validator warns.
- R801: no pneumatics; flywheels/rollers fine.
- R704.D: no extra streaming tools on the robot network at events. The sim never touches the robot network; if a hardware-in-the-loop mode is added later it is home-only.

---

## 17. Known unknowns and how to treat them

| Unknown | Treatment |
|---|---|
| Rocker mass/CG/inertia | sweep a range; UI slider; measure in November |
| Pivot friction, damper curve | sweep; MuJoCo cross-check; measure onset/duration on the real hive |
| Cd, Cl for a 26-hole ball | sweep ±30 %; calibrate from landing points |
| Where balls settle in the CELL | let the physics show it; check by eye against the CAD floor tilt |
| Exact rest angles | derive from damper geometry; adjust so dampers just touch at rest |
| Hub velocity-PID feel | start with the gains above; compare against `TuningDrive` logs from the hub in November |
| Bounce-in scoring, bottom-bonus interpretation | count both ways; ask the Q&A |
| Whether the shim keeps up with SDK changes | CI compile check; manual port at phase boundaries |

If a result looks surprising: first units (mm vs m vs in), then collider decomposition (a hull bridging the mouth explains most "balls float" bugs), then the timestep, then the bridge frame ordering (a stale `ActuatorFrame` explains most "robot ignores me" bugs).

---

## 18. Things to borrow, and from where

- `jsherman999/ftc_demo` (MIT): field/rules constants to cross-check, staging, **motor run-mode emulation** (`src/sim/motor-model.js` — RUN_USING_ENCODER as P+FF, RUN_TO_POSITION as P on ticks, τ 0.12 s), headless e2e test shape. Its Java→JS translator is an alternative "run real code" path if the JVM bridge ever becomes a burden.
- `LILRINO71/biobuzz-shot-sim` (MIT): `data/field.json` CELL geometry, `data/shooter.json` flywheel inertias, `tests/crosscheck/oracle.py` pattern.
- `Beta8397/virtual_robot` and `vr_physics`: the "abridged SDK" idea and its Road Runner integration — evidence that OpModes can run unchanged against a fake `HardwareMap`.
- WPILib sim (FRC): the "controller writes voltages → sim updates → sim writes sensors" loop shape; `BatterySim`.
- Road Runner / Pedro Pathing: how teams structure localizers and followers on the hub — copy the *shape* of `Localizer` and follower interfaces so the real ones drop in.

---

## 19. Later (explicitly out of scope now)

- **Hardware-in-the-loop**: the same bridge with the real hub on one end (a hub-side OpMode that speaks the bridge over the RC Wi-Fi). Home-only per R704.D.
- **Hub-side "tier A/B"**: the ballistic solver and planar robot model are small enough to run on the Control Hub (quad Cortex-A53, 1 GB RAM, Java 8, API 24) — the `ShotTable` already is the tier-A form.
- Opponent robots, defence, alliance partner.
- AprilTag detection with a simulated camera; Limelight emulation.
- Replacing the box with the real robot's CAD.

---

## Appendix A — Part-name → role map

| Name contains | Role |
|---|---|
| `am-5853-Red Hive`, `am-5853-Blue Hive` (subassembly path) | rocker-red / rocker-blue (everything under it) |
| `Red Cell (Audience)` / `(Scoring)`, `Blue Cell …` | CELL sub-groups inside a rocker |
| `Goal Rib`, `Bottom Skin`, `Top Skin`, `Back Skin`, `Basket Base Tube`, `Churro Lite`, `April Tag` | rocker collision pieces |
| `Goal Pivot Bracket`, `Axle Holder`, `Stepped Spacer`, `Pivot Damper Holder`, `Blumotion 970A Damper` | pivot hardware (frame side) |
| `am-5854: Frame`, `A-Frame Leg/Top Bar/Top Corner`, `ACM Panel`, `Sheet Metal Foot Bar`, `Frame Foot`, `Under Tile Bar` | frame (static) |
| `am-5855: Flower Assembly <n>`, `Flower Layer X/B/C`, `Flower HIPS Pipe`, `Flower Backstop`, `Flower Field Bracket`, `Peanut Support` | flower-n (static) |
| `FTC Field Side Glass 11in`, `am-2556a`, `Riveted FTC Panel`, `Corner Hinge`, `am-0481b` | perimeter (static) |
| `am-2499*` | tiles (static floor) |
| `am-5851: Pollen`, `am-5852: Red/Blue Nectar` | balls (dynamic; STEP transforms = staging) |
| screws, nuts, washers, rivnuts, cable ties, plugs | fasteners — render optional, no collision |

## Appendix B — Density table used for the first mass estimate (g/cm³)

Goal Rib 1.05 (HDPE/PC — **unknown, biggest lever on the result**), skins 1.20 (polycarbonate — unknown), tubes/churro/spacers 2.70 (aluminium), steel fasteners 7.85, vinyl skins 1.30, nylon 1.15. Change in `tools/cad2assets.py`; regenerate `frames.json`.

## Appendix C — Formulas in one place

```
Drag        Fd = −½ ρ Cd A |v| v
Magnus      Fm =  ½ ρ Cl A |v| (ω̂ × v),   Cl = clSlope·min(S, clSMax),  S = |ω| r / |v|
Motor       τ = τ_stall (1 − ω/ω_free(V)) (V/12),   I = I_stall τ/τ_stall,   ω_free(V) = ω_free·V/12
Battery     V = Voc(SoC) − R_int ΣI,   SoC −= ΣI dt / C
Tyre        F_i = clamp(τ_i·G/r, ±μ N_i) η   along the roller-permitted direction (±45°)
Flywheel    I ω̇ = τ_motor·G − τ_f;   per shot ω −= lossFactor·KE_ball/(I ω);   v_exit = k ω r
Hinge       I α = τ_balls + τ_gravity(θ) − τ_friction sign(ω) − τ_damper(θ, ω) − τ_limit
Ball torque τ_ball = ((p_contact − p_pivot) × F_contact) · x̂
Mecanum IK  ω_FL = (vx − vy − ω(L+W))/r,  ω_FR = (vx + vy + ω(L+W))/r,
            ω_BL = (vx + vy − ω(L+W))/r,  ω_BR = (vx − vy + ω(L+W))/r
Ticks       rpm = vel_ticks_per_s · 60 / ticksPerRev ;  ticksPerRev(5203-312) = 537.7
Feedforward power = (kS + kV·targetRpm)·(12/V) + kP·(targetRpm − rpm)
```

## Appendix D — Reproducing the CAD numbers

```
pip install gmsh            # OCP/cadquery are already present
cd cad && python analyse_step.py field-cad-step.step parts.json > report.txt
```

## Appendix E — SDK surface allowed in TeamCode (the shim must provide exactly these)

```
LinearOpMode:   runOpMode, waitForStart, opModeIsActive, opModeInInit, isStarted, isStopRequested, idle, sleep,
                hardwareMap, telemetry, gamepad1, gamepad2, getRuntime, resetRuntime
OpMode:         init, init_loop, start, loop, stop (+ the same fields)
Annotations:    @Autonomous(name, group, preselectTeleOp), @TeleOp(name, group), @Disabled
HardwareMap:    get(Class<T>, String), tryGet, voltageSensor (iterable), getAll(Class<T>)
DcMotorEx:      setPower/getPower, setMode/getMode (RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER),
                setZeroPowerBehavior (BRAKE, FLOAT), setDirection (FORWARD, REVERSE), getCurrentPosition,
                setTargetPosition/getTargetPosition, isBusy, setVelocity(double), setVelocity(double, AngleUnit),
                getVelocity(), getVelocity(AngleUnit), setPIDFCoefficients(RunMode, PIDFCoefficients), getPIDFCoefficients,
                getCurrent(CurrentUnit), setTargetPositionTolerance, setMotorEnable/Disable
Servo:          setPosition/getPosition, setDirection, scaleRange
CRServo:        setPower/getPower, setDirection
IMU:            initialize(IMU.Parameters), getRobotYawPitchRollAngles, getRobotAngularVelocity, resetYaw
                IMU.Parameters(RevHubOrientationOnRobot), RevHubOrientationOnRobot(LogoFacingDirection, UsbFacingDirection)
VoltageSensor:  getVoltage
DistanceSensor: getDistance(DistanceUnit)
ColorSensor:    red, green, blue, alpha, argb
TouchSensor:    isPressed
Gamepad:        left_stick_x/y, right_stick_x/y, left_trigger, right_trigger, a/b/x/y, dpad_*, left_bumper/right_bumper,
                start/back/guide, left_stick_button/right_stick_button, rumble(ms)
Telemetry:      addData(String, Object), addData(String, String, Object...), addLine(String), update, clearAll, setMsTransmissionInterval
ElapsedTime:    ElapsedTime(), reset, seconds, milliseconds, time
Range:          clip, scale
LynxModule:     getAll via hardwareMap.getAll(LynxModule.class), setBulkCachingMode(AUTO|MANUAL|OFF), clearBulkCache
AngleUnit, DistanceUnit, CurrentUnit, YawPitchRollAngles (getYaw/getPitch/getRoll(AngleUnit))
```

Adding anything else: add it to the shim first, note it in `docs/DECISIONS.md`, and make sure the CI compile against the real SDK still passes.

## Appendix F — Bridge message schema (summary)

`SensorFrame` and `ActuatorFrame` as in §3.6; control messages `hello`, `reset`, `mode`, `opmode`, `ack`, `error`. All numbers are IEEE doubles; times in seconds; sequence numbers increase by one per world frame; a frame with a stale `seq` is ignored and logged. Full JSON Schema in `schemas/bridge.schema.json`, validated in `tests/bridge`.
