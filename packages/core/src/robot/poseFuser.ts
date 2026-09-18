/**
 * Odometry drifts. The tag says where you are. This is the half that closes the loop.
 *
 * THE MIRROR of java/teamcode/.../control/FusedLocalizer.java.
 *
 * A fiducial at a SURVEYED position is, first and last, a measurement of the observer. The
 * robot sees the tag at a bearing and a range; the tag's field position is a known constant;
 * so the robot's position follows by subtraction. That constrains the dead-reckoned pose and
 * bounds an error that otherwise grows with every inch driven and never comes back.
 *
 * WHICH WAY THE INFORMATION FLOWS IS A CHOICE, AND YOU ONLY GET ONE.
 *
 * A single sighting is one relative measurement between two unknowns -- where the robot is,
 * and where the HIVE is. It cannot pin both. Estimating the HIVE from the pose AND the pose
 * from the HIVE is circular: the pair drifts off together, perfectly consistent with each
 * other and wrong about the field, and nothing in the measurements ever notices. With one
 * landmark and no loop closure there is no second opinion to break the tie.
 *
 * So the field is the reference and the robot is what gets estimated, because that is the one
 * of the two that is actually surveyed: the HIVE is bolted to a field built to a drawing, and
 * `tools/tagoffsets.ts` reads its geometry out of the CAD. That makes
 * `sensors.tag.target.anchorAlpha` -- the robot's own estimate of the pivot -- redundant while
 * this is running, and it defaults to 0 for exactly that reason. Turn it on only if you
 * distrust the survey more than the odometry, and understand you are choosing which of the two
 * to believe, not getting both.
 *
 * THREE THINGS HAD TO BE RIGHT, and each was wrong first. They are worth keeping written down,
 * because every one of them looked like it worked and measurably made the robot worse:
 *
 *   1. HEADING HAS TO BE CORRECTED TOO. A bearing-and-range fix is measured along a line drawn
 *      at the heading the robot believes, so correcting position under a heading that is 4 deg
 *      out drives the pose to a place consistent with that 4 deg. Bearing and range give two
 *      constraints for three unknowns; the panel's YAW supplies the third and is what makes the
 *      pose observable from a single landmark at all. It is the worst-conditioned axis of an
 *      AprilTag pose, 6 deg against 0.5 on bearing, hence the much smaller gain.
 *
 *   2. THE FIX HAS TO BE COMPARED AGAINST THE POSE THE ROBOT WAS IN WHEN IT LOOKED. A detection
 *      arrives 75-108 ms old; a robot turning at 100 deg/s has moved 10 deg by then. Because a
 *      driving robot turns in a correlated way, that lands as a BIAS, not as noise.
 *
 *   3. THIS HAS TO BE AN ESTIMATOR, NOT AN OFFSET, and that is the one that really mattered.
 *      Keeping the corrections as a translation added to whatever odometry reported looks
 *      equivalent and is not: odometry keeps INTEGRATING in its own uncorrected heading, so a
 *      heading error goes on turning every inch driven into new position error at exactly the
 *      old rate, and a constant offset cannot chase it. Correcting at the reporting layer
 *      leaves the leak open and mops the floor. So the fuser carries its own pose and
 *      integrates odometry DELTAS into it -- a heading correction then changes the frame every
 *      subsequent step is added in, which is the entire point.
 *
 * THE CORRECTION IS A GAIN, NOT A JUMP. A tag fix carries the camera's own error -- 4% of
 * range, 2 in at 50 -- so snapping onto every detection at 30 Hz injects that straight into the
 * aim. A one-pole pulls toward each fix instead, so the noise averages down while the drift,
 * which is one-signed, still gets removed.
 */

export interface PoseFuserSpec {
  /** Fraction of each fix's position disagreement to absorb. 0 disables correction. */
  gain: number;
  /**
   * The same, for heading, off the tag's yaw. Much smaller: yaw carries an order more noise
   * than bearing, so it is averaged over many fixes instead of believed once.
   */
  headingGain: number;
  /**
   * Refuse a fix that disagrees by more than this, inches. A detection that says the robot is
   * four feet from where it thinks it is, is a misread or the wrong tag -- not four feet of
   * drift that appeared between two frames.
   */
  rejectOver_in: number;
}

const wrapDeg = (d: number): number => {
  let x = (d + 180) % 360;
  if (x < 0) x += 360;
  return x - 180;
};

const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;

export interface Pose {
  x: number;
  y: number;
  heading: number;
}

export class PoseFuser {
  private x = 0;
  private y = 0;
  private heading = 0;
  private started = false;
  private last: Pose = { x: 0, y: 0, heading: 0 };
  /** A short history of the FUSED pose, so a detection can be back-dated onto it. */
  private hist: { t: number; x: number; y: number; heading: number }[] = [];

  applied = 0;
  rejected = 0;
  lastCorrectionIn = 0;
  headingCorrectionDeg = 0;

  constructor(private readonly spec: PoseFuserSpec) {}

  reset(): void {
    this.started = false;
    this.hist = [];
    this.applied = 0;
    this.rejected = 0;
    this.lastCorrectionIn = 0;
    this.headingCorrectionDeg = 0;
  }

  /** The robot's best estimate of where it is. Call propagate() first, every loop. */
  pose(): Pose {
    return { x: this.x, y: this.y, heading: this.heading };
  }

  /**
   * Fold in this loop's odometry. DELTAS, not the absolute reading: the raw pose carries all
   * of its accumulated drift, and taking it whole would throw away every correction so far.
   */
  propagate(t: number, raw: Pose): Pose {
    if (!this.started) {
      this.x = raw.x;
      this.y = raw.y;
      this.heading = raw.heading;
      this.last = { x: raw.x, y: raw.y, heading: raw.heading };
      this.started = true;
    } else {
      const dx = raw.x - this.last.x;
      const dy = raw.y - this.last.y;
      const dHead = wrapDeg(raw.heading - this.last.heading);
      // The step, rotated out of odometry's heading frame and into the corrected one. This is
      // where a heading fix earns its keep: every inch after it is integrated the right way.
      const rot = wrapDeg(this.heading - this.last.heading) * DEG;
      const c = Math.cos(rot);
      const sn = Math.sin(rot);
      this.x += dx * c - dy * sn;
      this.y += dx * sn + dy * c;
      this.heading = wrapDeg(this.heading + dHead);
      this.last = { x: raw.x, y: raw.y, heading: raw.heading };
    }
    this.hist.push({ t, x: this.x, y: this.y, heading: this.heading });
    while (this.hist.length > 2 && t - this.hist[0].t > 0.5) this.hist.shift();
    return this.pose();
  }

  /** The fused pose at `t`, for anything else that has to back-date a measurement. */
  poseAt(t: number): Pose { return this.at(t); }

  /** The fused pose at `t`, or the nearest thing remembered. */
  private at(t: number): Pose {
    if (!this.hist.length) return this.pose();
    let best = this.hist[0];
    for (const h of this.hist) if (Math.abs(h.t - t) < Math.abs(best.t - t)) best = h;
    return { x: best.x, y: best.y, heading: best.heading };
  }

  /**
   * Fold in one sighting of a tag whose field position is surveyed.
   *
   * @param tagX/tagY  the tag's surveyed field position, inches
   * @param bearingDeg the tag's bearing relative to the robot's heading
   * @param rangeIn    the tag's range
   * @param facingX    which way the panel faces along FTC +X, +1 or -1. Surveyed.
   * @param openDeg    the tag's measured yaw: how far off square the robot is to its face
   * @param sampleT    when the photons left, on the same clock as propagate()
   */
  observe(
    tagX: number,
    tagY: number,
    bearingDeg: number,
    rangeIn: number,
    facingX: number,
    openDeg: number,
    sampleT: number,
  ): Pose {
    const then = this.at(sampleT);

    // HEADING FIRST, because the position fix is measured along a line drawn at it.
    let headingThen = then.heading;
    if (this.spec.headingGain > 0 && facingX !== 0) {
      const psi = facingX > 0 ? 0 : 180;                 // the panel normal, as a field angle
      const phiEst = Math.atan2(then.y - tagY, then.x - tagX) * RAD;
      const cand = [psi + openDeg, psi - openDeg];       // yaw loses the left/right sign
      const phi = Math.abs(wrapDeg(cand[0] - phiEst)) <= Math.abs(wrapDeg(cand[1] - phiEst))
        ? cand[0]
        : cand[1];
      const implied = wrapDeg(phi + 180 - bearingDeg);
      const hg = Math.max(0, Math.min(1, this.spec.headingGain));
      const dH = wrapDeg(implied - headingThen) * hg;
      headingThen = wrapDeg(headingThen + dH);
      this.heading = wrapDeg(this.heading + dH);
      // REBASE THE HISTORY TOO. A correction is a rigid transform of the whole trajectory
      // estimate, not just of its latest point -- and the history is what the NEXT detection
      // gets back-dated onto. Leaving it stale means every fix is compared against a pose
      // that has not seen any of the corrections before it, so the same correction is applied
      // over and over: a test that fed 60 identical sightings to a robot 8 deg out drove it
      // to 16 deg out, away from the truth, at a perfectly steady rate.
      for (const h of this.hist) h.heading = wrapDeg(h.heading + dH);
      this.headingCorrectionDeg = dH;
    }

    // Where the robot must have been for this tag, at this bearing and range, to be where the
    // field drawing says it is.
    const b = (headingThen + bearingDeg) * DEG;
    const dx = tagX - rangeIn * Math.cos(b) - then.x;
    const dy = tagY - rangeIn * Math.sin(b) - then.y;
    const miss = Math.hypot(dx, dy);
    this.lastCorrectionIn = 0;
    if (!(miss <= this.spec.rejectOver_in)) {
      this.rejected++;
      return this.pose();
    }
    const g = Math.max(0, Math.min(1, this.spec.gain));
    // A rigid translation, so solving for it back then and applying it now is exact -- and
    // the history moves with it, for the same reason the heading history does.
    this.x += dx * g;
    this.y += dy * g;
    for (const h of this.hist) {
      h.x += dx * g;
      h.y += dy * g;
    }
    this.applied++;
    this.lastCorrectionIn = miss * g;
    return this.pose();
  }
}
