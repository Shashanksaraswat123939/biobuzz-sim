/**
 * A tag camera, not an oracle.
 *
 * `game.upCellAzimuthDeg` / `upCellRangeIn` / `upCellOpenDeg` / `hiveTipping` used to come
 * straight off the world's truth: no noise, no latency, no field of view, never invalid. The
 * robot always knew exactly where the goal was and always knew the instant the HIVE went
 * over. That is the one thing a real FTC robot cannot do, and every aiming result in this
 * repo was measured on top of it (docs/PHYSICS.md 6, docs/LOCALIZATION.md).
 *
 * This is the sensor that replaces it. It emits what an `AprilTagProcessor` emits -- a
 * DETECTION, not a pose: which tag, its bearing, its range, how square its face is to us --
 * and it emits nothing at all when a camera would not have seen the tag:
 *
 *   - while the rocker is swinging (the tag is moving and motion blurred),
 *   - past `maxRange_in` (the tag stops resolving),
 *   - past `maxIncidence_deg` (grazing: the face has almost no projected area),
 *   - outside the lens (the camera rides the TURRET, so the gate is against turret angle).
 *
 * It runs at the camera's own frame rate, not the control loop's, and what it returns is
 * `latencyMs` old. Both matter to a moving shot: the pipeline is the slowest thing in the
 * aim chain and the only one the robot cannot make faster by trying harder.
 *
 * ON THE TURRET, not the chassis (docs/LOCALIZATION.md phase 4). A chassis camera with a
 * 60 deg lens covers 17% of headings and "a tag is in view" is mostly luck; the turret
 * already tracks the CELL, so a camera bolted to it points at the tag whenever the shooter
 * is aimed -- by construction rather than by chance.
 */
import { Rng } from '../io/rng.js';
import type { TagCameraSpec, TagSighting } from '../types.js';

export type { TagSighting } from '../types.js';

/** The truth the camera is pointed at, sampled every step so latency can be served from it. */
export interface TagTruth {
  id: number;
  bearingDeg: number;
  rangeIn: number;
  openDeg: number;
  /** Where the turret -- and so the lens -- is pointing, relative to the robot's heading. */
  turretDeg: number;
  tipping: boolean;
}

const wrapDeg = (d: number): number => {
  let x = (d + 180) % 360;
  if (x < 0) x += 360;
  return x - 180;
};

export class TagCamera {
  private buf: (TagTruth & { t: number })[] = [];
  private nextFrameT = 0;
  private latest: TagSighting | null = null;
  /** Detections attempted and detections made, so a run can report how blind it was. */
  frames = 0;
  hits = 0;
  /**
   * World time of the last SUCCESSFUL decode. `read()` keeps handing back the last detection
   * forever, which answers "has it ever seen the tag" and not "is it seeing it now" -- and it
   * is the second question that tells you whether the geometry currently works.
   */
  lastHitT = -1e9;

  constructor(private readonly spec: TagCameraSpec, private readonly rng: Rng) {}

  /**
   * Feed one step of truth, and on a camera frame turn a LATENCY-OLD sample into a
   * detection. A failed gate leaves the previous detection alone rather than clearing it:
   * that is what a pipeline does, and ageing the last fix is the consumer's job.
   */
  step(t: number, truth: TagTruth): void {
    this.buf.push({ ...truth, t });
    const lag = this.spec.latencyMs / 1000;
    while (this.buf.length > 1 && t - this.buf[0].t > lag) this.buf.shift();
    if (!this.spec.enabled) return;
    if (t < this.nextFrameT) return;
    this.nextFrameT = t + 1 / Math.max(this.spec.frameRateHz, 1e-6);
    this.frames++;

    const s = this.buf[0]; // what the lens saw `lag` ago, which is all it can report now
    if (s.tipping) return;
    if (s.rangeIn > this.spec.maxRange_in) return;
    if (s.openDeg > this.spec.maxIncidence_deg) return;
    if (Math.abs(wrapDeg(s.bearingDeg - s.turretDeg)) > this.spec.fov_deg / 2) return;

    const n = this.spec.noise;
    this.hits++;
    this.lastHitT = t;
    this.latest = {
      id: s.id,
      bearingDeg: s.bearingDeg + this.rng.gauss(0, n.bearing_deg),
      // Range off a tag comes from its apparent SIZE, so the error is a fraction of the
      // range rather than a fixed distance -- which is why docs/LOCALIZATION.md says to
      // weight bearing heavily and range lightly when the two are ever fused.
      rangeIn: s.rangeIn * (1 + this.rng.gauss(0, n.rangeFrac)),
      // Tag YAW is the worst-conditioned axis of an AprilTag pose by a wide margin, and this
      // is the axis the "is the mouth still open towards me" gate is built on. Its sigma is
      // an order above the bearing's on purpose.
      openDeg: Math.max(0, Math.min(180, s.openDeg + this.rng.gauss(0, n.open_deg))),
      sampleT: s.t,
    };
  }

  /** Is the tag decodable right now -- within the last couple of camera frames? */
  isSeeing(t: number): boolean {
    return t - this.lastHitT <= 2 / Math.max(this.spec.frameRateHz, 1e-6);
  }

  /** The most recent detection, however old. Null until the camera has ever seen the tag. */
  read(): TagSighting | null {
    return this.latest;
  }

  reset(): void {
    this.buf = [];
    this.nextFrameT = 0;
    this.latest = null;
    this.frames = 0;
    this.hits = 0;
    this.lastHitT = -1e9;
  }
}
