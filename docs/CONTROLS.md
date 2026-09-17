# Controls

**Everything is a button.** The deck down the left side of the app drives the same state the
keys do, so you never have to touch the keyboard. Toggles light up when they are on; Fire
lights red, because it keeps firing until you press it again.

Keyboard and gamepad both drive the built-in TeleOp. When **Brain** is switched to *Java*,
the gamepad goes over the bridge to the OpMode instead and these bindings become whatever
that OpMode does with `gamepad1`.

## Driving

| Key | Gamepad | Does |
|---|---|---|
| `W` `S` | left stick Y | forward / back |
| `A` `D` | left stick X | strafe left / right |
| `Q` `E` | right stick X | turn left / right |
| `Shift` | LB | slow mode (35 %) |
| `C` | L3 | robot-centric ↔ field-centric — **Field-centric** |
| `Z` | Back | re-zero what "forward" means (field-centric only) |

**Robot-centric is the default** — `W` drives the way the robot is pointing. Field-centric
drives relative to a fixed heading instead, which is how most FTC drivers run once they are
used to it; `Z` sets that heading to whichever way the robot currently faces.

## Game pieces

| Key | Gamepad | Button | Does |
|---|---|---|---|
| `I` | D-pad ↑ | **Intake** | intake on / off — **it starts on**, like a real one |
| `J` | RT | — | hold to run the intake while it is switched off |
| `K` | LT | **Spit one out** | hold to spit (the button ejects one ball) |
| `F` | A | **Flywheel** | flywheel on / off |
| `Space` | RB | **Fire** | fire **latch** — keeps firing at the cycle time until pressed again |
| — | B | — | hold to fire without latching |
| `G` | Y | **Gate** | gate open / closed |
| `T` | X | **Auto-aim** | auto-aim on / off |
| `,` `.` | D-pad ← → | — | turret left / right (manual aim only) |
| `L` | — | **Auto-load hopper** | keep the hopper topped up from balls on the floor |
| — | — | **Fill hopper now** | one-shot top-up |
| `H` | — | **Drop POLLEN in CELL** | hand-drop a ball into your up CELL |

Fire is a latch on purpose: the transfer's `cycleTime_s` paces the shots, not your thumb.
It still refuses to feed unless the readiness gate is open.

`Space` will not fire while the flywheel is below speed, the turret is still slewing, the
range is outside the shot table, or the HIVE is mid-tip (G417). The pill in the HUD says
which one is stopping it.

## View and match

| Key | Does |
|---|---|
| `1` | orbit — **drag** to rotate, **shift-drag** (or middle-drag) to pan, scroll to zoom |
| `2` | follow |
| `3` | top-down |
| **`4`** | **first person** — looks where the **chassis** points, not the turret |
| `5` | muzzle — looks down the barrel |
| `Enter` | start the match (30 s AUTO → 8 s transition → 2:00 TELEOP) — **Start match** |
| `P` | pause / resume — **Pause** |
| `R` | reset everything — **Reset** |

The bottom bar has alliance, camera, the shot-arc toggle, a **Colliders** toggle (swaps the
CAD skin for the convex boxes the physics actually uses), the brain selector, and the frame
time plus the determinism hash.
