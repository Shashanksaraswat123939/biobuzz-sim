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
| `Shift`+`↑` `↓` | R2 / L2 | forward / back on the triggers, added to the stick |
| `Q` `E` | X / B | turn left / right |
| `R` `F` | Y / A | speed gear up / down |
| arrows | right stick | the VIEW. The right stick never turns the robot |
| `C` | R3 | hold for field-centric |
| `Backspace` | — | re-zero what "forward" means (keyboard only) |

**Robot-centric is the default** — `W` drives the way the robot is pointing, and the front
of the box is the intake. Field-centric is a HOLD on `C`, not a toggle: a modifier's keyup is
the one you reliably miss, and a missed keyup leaves the drive frame stuck in the mode you
are not in.

Turning is on `Q` / `E` (X / B) because the right stick moves the camera. They are buttons,
so they are synthesised into the yaw command *before* the ramp: a short tap is a precise
correction you can still shoot through, a long hold is not. Measured with
`tools/turretcheck.ts`: 100 ms gives 15 °/s and fires, 200 ms gives 49 °/s and fires, 300 ms
gives 90 °/s and the fire gate refuses — its cap is 70 °/s.

## Game pieces

| Key | Gamepad | Button | Does |
|---|---|---|---|
| `Space` | **R1** | **Auto-fire** | the **latch** — a ball every time the gate clears, at the cycle time, until pressed again |
| `G` | L3 | — | hold to fire by hand, one gate-cleared ball at a time |
| `T` | **L1** | **Auto-aim** | auto-aim on / off |
| `,` `.` | M1 / M2 | — | nudge the turret anticlockwise / clockwise (manual aim only) |
| `V` | D-pad ↑ | — | pre-spin the flywheel |
| `Z` | D-pad ↓ | — | reverse the intake to spit |
| `L` | — | **Auto-fill hopper** | keep the hopper topped up from balls on the floor |
| `H` | — | **Drop POLLEN in CELL** | hand-drop a ball into your up CELL |

M1 and M2 are the paddles; on a pad without them they fall back to D-pad left and right.
The intake always runs, like a real one, and rejects the opponent's NECTAR by reversing.

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
