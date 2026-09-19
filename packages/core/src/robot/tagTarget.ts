/**
 * Where to aim, from a camera that is usually not looking at the goal.
 *
 * THE MIRROR of java/teamcode/.../control/TagTargetProvider.java. Both brains -- the
 * built-in one and the Java deliverable -- have to answer "where is the up CELL" the same
 * way, or the simulator measures one robot and the hub flies another. tests/tag.test.ts
 * asserts the two agree on the cases that matter.
 *
 * Three states, and the whole point is that only one of them may shoot:
 *
 *   FRESH   a detection this loop or last: aim on it, fire on it.
 *   HELD    no detection for a moment, but there was one: aim on the fix carried forward by
 *           odometry, DO NOT FIRE. A shot solved off a fix that is a third of a second old
 *           is solved off where the goal was, and this is exactly the window a TIP lives in.
 *   LOST    nothing recent enough to carry: sweep the turret and look for the tag.
 *
 * The fix is carried forward the cheap way: the detection is turned into a FIELD-FRAME
 * POINT once, and the bearing and range are re-derived from the localizer's current pose
 * every loop after that. That costs two trig calls, needs no target model, and degrades
 * exactly the way the odometry does -- which is the honest failure mode.
 *
 * ponytail: the fix is built against the pose NOW, not the pose when the photons left, so a
 * robot moving at v carries a v*latency error into the field point (0.075 s at 1.5 m/s is
 * 4.4 in). Back-date it with a short pose ring buffer if the moving-shot error budget ever
 * needs the inches back; the staleness gate below is what stops it compounding.
 */
import type { TagSighting } from '../types.js';

export interface TagTargetSpec {
  /**
   * Tag -> CELL mouth, FTC +X inches, keyed by tag id. The panel is about 10 in from the
   * mouth on the same rocker, and the rocker is bistable, so this has exactly two values and
   * the ID says which (tools/tagoffsets.ts, docs/LOCALIZATION.md).
   */
  mouthDx: Record<number, number>;
  /** Which way the up CELL opens along FTC +X, by tag id. A TIP flips it. */
  mouthFacingX: Record<number, number>;
  /**
   * WHERE THE HIVE IS, TRACKED RATHER THAN ASSUMED.
   *
   * `anchorPrior` is the pivot's surveyed position -- a coordinate to start from, nothing
   * more. The moment a tag is decoded the robot computes the pivot for itself from its own
   * pose and the detection, and aims off THAT from then on. `mouthFromAnchor` is the rigid
   * offset from the pivot to each CELL's mouth, which is a property of the HIVE and not of
   * the field: a TIP swings the CELLs about the pivot and leaves the pivot alone.
   */
  anchorPrior: { x: number; y: number };
  mouthFromAnchor: Record<number, { x: number; y: number }>;
  /**
   * One-pole on the tracked pivot. The HIVE does not translate, so averaging across sightings
   * is legitimate and buys down the camera's range noise.
   *
   * ponytail: with REAL (drifting) odometry this has to be short, or the anchor wanders with
   * the pose that produced it. The proper fix is to invert the problem -- once the anchor is
   * trusted, a sighting is a measurement of the ROBOT, and should correct the pose instead.
   * That is docs/LOCALIZATION.md phase 5 and it is not built.
   */
  anchorAlpha: number;
  /** May a shot be taken on odometry alone? Never before a tag has been seen at least once. */
  fireOnOdometry: boolean;
  /**
   * How long a DECODED ROCKER STATE may be trusted without seeing a tag again, seconds.
   *
   * `fireOnOdometry` says the robot may keep shooting on the surveyed geometry once it knows
   * which CELL is up. It may -- for a while. A TIP changes which CELL is up and hides itself
   * at the same time, because the new tag faces away from where the robot is standing, so
   * "no detection" and "nothing has changed" look identical from here. Past this age they
   * are not the same claim any more and the robot goes and looks.
   */
  maxStateAgeS: number;
  /** Which CELL is up at the start of a match. The assumption odometry begins from. */
  startId: number;
  /** How long a fix may be carried on odometry before the target counts as lost, s. */
  holdS: number;
  /** How old a fix may be and still be shot on, s. */
  maxFireAgeS: number;
  /**
   * How much of each new sighting to take. 1 replaces the estimate outright, which is what
   * this used to do -- and it means every fix jumps the target by that detection's own noise,
   * about 2 in at 50. Below 1 the sightings average, and the noise comes down as their root.
   */
  measAlpha: number;
  /** How fast the turret sweeps while searching, deg/s. */
  scanRateDps: number;
  /** Turret travel, degrees, so the sweep stays inside it. */
  turretMinDeg: number;
  turretMaxDeg: number;
}

export interface TagTargetState {
  /** Always a bearing worth pointing the turret at: the fix, the carried fix, or the sweep. */
  azimuthDeg: number;
  rangeIn: number;
  /** 180 when the camera has never seen the tag: unknown, and no shot may be taken on it. */
  openDeg: number;
  /** A fix exists and is recent enough to aim by. False means the turret is searching. */
  valid: boolean;
  /** Fresh enough to FIRE on. Always implies `valid`. */
  fresh: boolean;
  ageS: number;
  /**
   * The lock was lost after having had one. A TIP looks exactly like this from a camera --
   * the rocker swings, the tag blurs and the other CELL's tag comes up in its place -- and
   * so does driving behind a wall. The camera cannot tell them apart and neither can this;
   * both mean hold fire, which is the only decision that hangs off it.
   */
  lostLock: boolean;
  scanning: boolean;
  /** Which CELL's tag the fix belongs to, 0 if none. */
  id: number;
  /**
   * The bearing and range came from the localizer and the baked HIVE position, not from a
   * detection. The aim is as good as the odometry; the ROCKER STATE is as good as the last
   * tag seen, which is the part that can silently go wrong.
   */
  fromOdometry: boolean;
  /** A tag has been seen at least once, so the rocker state is observed rather than assumed. */
  stateFromTag: boolean;
  /** The HIVE's tracked pivot, FTC inches: measured once a tag has been seen, else the prior. */
  anchor: { x: number; y: number };
  /** True once the anchor is the robot's own estimate rather than the surveyed prior. */
  anchorFromTag: boolean;
}

const wrapDeg = (d: number): number => {
  let x = (d + 180) % 360;
  if (x < 0) x += 360;
  return x - 180;
};

const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;

export class TagTarget {
  private fixX = 0;
  private fixY = 0;
  private fixFacingX = 0;
  private fixId = 0;
  private fixT = -1e9;
  /** Has a tag EVER been decoded? Until it has, the rocker state is an assumption. */
  private seenTag = false;
  /** The HIVE's pivot as the robot currently believes it, FTC inches. */
  private anchorX = 0;
  private anchorY = 0;
  private anchorSeen = false;
  private scanDeg = 0;
  private scanDir = 1;

  constructor(private readonly spec: TagTargetSpec) {
    this.scanDeg = Math.max(spec.turretMinDeg, Math.min(spec.turretMaxDeg, 0));
    this.anchorX = spec.anchorPrior.x;
    this.anchorY = spec.anchorPrior.y;
  }

  reset(): void {
    this.fixT = -1e9;
    this.fixId = 0;
    this.fixFacingX = 0;
    this.seenTag = false;
    this.anchorX = this.spec.anchorPrior.x;
    this.anchorY = this.spec.anchorPrior.y;
    this.anchorSeen = false;
    this.scanDeg = Math.max(this.spec.turretMinDeg, Math.min(this.spec.turretMaxDeg, 0));
    this.scanDir = 1;
  }

  /**
   * How far off the mouth's opening the robot is, degrees. 0 is square on, past 90 is behind
   * the goal and no launch can enter.
   *
   * COMPUTED, not measured, and that is the better answer here. The camera does return a tag
   * yaw and it is the worst-conditioned axis of an AprilTag pose by an order -- 6 deg of
   * sigma against 0.5 on bearing. The ID plus the baked geometry gives the same fact exactly:
   * knowing which CELL is up IS knowing which way its mouth opens.
   */
  private mouthOpenDeg(loc: { x: number; y: number }): number {
    if (this.fixFacingX === 0) return 180;
    const dx = loc.x - this.fixX;
    const dy = loc.y - this.fixY;
    const d = Math.hypot(dx, dy);
    if (d < 1e-6) return 0;
    return Math.acos(Math.max(-1, Math.min(1, (dx * this.fixFacingX) / d))) * RAD;
  }

  update(
    t: number,
    loc: { x: number; y: number; heading: number },
    sight: TagSighting | null,
    dt: number,
    /** The pose when the detection was taken. Omit and the current pose is used instead. */
    poseThen?: { x: number; y: number; heading: number },
  ): TagTargetState {
    if (sight && sight.sampleT > this.fixT) {
      // REPLACED, NEVER BLENDED, which is what docs/LOCALIZATION.md phase 5 asks for on an
      // ID change and is the reason there is no ID check here to forget: the two CELLs sit
      // 14 in apart on opposite sides of the pivot, and averaging a fix on one with a fix on
      // the other aims at the rocker between them. The newest detection simply wins.
      // AGAINST THE POSE WHEN THE PHOTONS LEFT, not the pose now. The detection is 75-108 ms
      // old; at 1.5 m/s that is 4-6 in of travel, and a robot driving hard turns that into a
      // target that sits behind where it really is. `poseThen` is the fuser's own back-dated
      // estimate, which is the same history the pose correction uses.
      const at = poseThen ?? loc;
      const b = (at.heading + sight.bearingDeg) * DEG;
      // THE TAG, then the MOUTH. What the camera measured is the panel; what the shot needs
      // is the mouth, and the two are a rigid offset apart on the rocker. The ID picks which
      // of its two field-frame values applies -- the tag is the rocker state sensor.
      const mx = at.x + sight.rangeIn * Math.cos(b) + (this.spec.mouthDx[sight.id] ?? 0);
      const my = at.y + sight.rangeIn * Math.sin(b);
      // AVERAGE THE SIGHTINGS, except across a TIP. Each detection carries the camera's own
      // error -- 4% of range, 2 in at 50 -- so taking each one whole makes the target jump by
      // that much 30 times a second. Blending pulls the noise down as the root of the count.
      // A change of ID is the one case that must NOT blend: the two CELLs are 14 in apart on
      // opposite sides of the pivot, and an average of them aims at the rocker in between.
      const a = this.fixId === sight.id && this.seenTag
        ? Math.max(0, Math.min(1, this.spec.measAlpha))
        : 1;
      this.fixX += a * (mx - this.fixX);
      this.fixY += a * (my - this.fixY);
      this.fixFacingX = this.spec.mouthFacingX[sight.id] ?? 0;
      this.fixId = sight.id;
      this.fixT = sight.sampleT;
      this.seenTag = true;

      // AND TRACK THE HIVE ITSELF, not just this one shot.
      //
      // The measured mouth minus the rigid pivot->mouth offset IS the pivot, in field
      // coordinates, measured by the robot. That is the landmark: it does not move, a TIP
      // swings the CELLs about it, and knowing it means the OTHER CELL's mouth is known too
      // the moment the rocker goes over. The surveyed prior is only ever a starting guess.
      const off = this.spec.mouthFromAnchor[sight.id];
      if (off) {
        const ax = this.fixX - off.x;
        const ay = this.fixY - off.y;
        const a = this.anchorSeen ? Math.max(0, Math.min(1, this.spec.anchorAlpha)) : 1;
        this.anchorX += a * (ax - this.anchorX);
        this.anchorY += a * (ay - this.anchorY);
        this.anchorSeen = true;
      }
    }

    const have = this.fixT > -1e8;
    const ageS = have ? Math.max(0, t - this.fixT) : Infinity;
    // MAY IT FIRE? Two ways to earn it, and they are different claims.
    //
    // A fresh detection is the camera saying so. Anything older leans on the surveyed
    // geometry, which is a policy call -- `fireOnOdometry` -- and is refused outright until a
    // tag has been decoded at least once, because until then WHICH CELL IS UP is an
    // assumption and shooting into the wrong one moves the HIVE for the other alliance.
    // A DECODED STATE GOES OFF. The second arm used to have no clock in it: decode a tag
    // once and the robot would fire on the surveyed geometry for the rest of the match. Fine
    // for a fix that blinks; wrong across a TIP, which is the one event that changes which
    // CELL is up and hides the evidence in the same motion. tools/tipcheck.ts: 60 balls fired
    // after a tip, none credited, CLEAR TO FIRE the whole way.
    const stateAgeOk = ageS <= Math.max(this.spec.maxStateAgeS, this.spec.maxFireAgeS);
    const fresh = (have && ageS <= this.spec.maxFireAgeS)
      || (this.spec.fireOnOdometry && this.seenTag && stateAgeOk);

    // BOTH, CROSSFADED BY AGE -- not one or the other at a threshold.
    //
    // The two estimates fail in opposite directions and neither is right on its own. The
    // camera's is the more accurate the instant it arrives and gets worse with every
    // millisecond it is carried; the geometry's is as good as the pose and no better, but it
    // does not decay. A hard cutover at `holdS` took the better one and threw it away whole
    // the moment the clock ran out, which is a step change in the aim for no physical reason.
    //
    // So the weight slides. At age 0 the measurement carries it; by `holdS` the surveyed
    // geometry does; in between they are mixed in the proportion their errors deserve. That
    // is also the honest answer to driving fast, standing off to the side, or having the tag
    // covered for a second: nothing switches, the aim just leans on whichever source is
    // currently worth more.
    const wMeas = have ? Math.max(0, Math.min(1, 1 - ageS / Math.max(this.spec.holdS, 1e-6))) : 0;

    const stateNow = this.seenTag ? this.fixId : this.spec.startId;
    const offNow = this.spec.mouthFromAnchor[stateNow];
    const geo = offNow ? { x: this.anchorX + offNow.x, y: this.anchorY + offNow.y } : null;

    if (have && geo) {
      const mx = wMeas * this.fixX + (1 - wMeas) * geo.x;
      const my = wMeas * this.fixY + (1 - wMeas) * geo.y;
      const dx = mx - loc.x;
      const dy = my - loc.y;
      const rangeIn = Math.hypot(dx, dy);
      const azimuthDeg = wrapDeg(Math.atan2(dy, dx) * RAD - loc.heading);
      this.scanDeg = Math.max(this.spec.turretMinDeg, Math.min(this.spec.turretMaxDeg, azimuthDeg));
      const facing = this.spec.mouthFacingX[stateNow] ?? 0;
      const d = Math.hypot(loc.x - mx, loc.y - my);
      return {
        azimuthDeg,
        rangeIn,
        openDeg: facing === 0 || d < 1e-6
          ? 180
          : Math.acos(Math.max(-1, Math.min(1, ((loc.x - mx) * facing) / d))) * RAD,
        valid: true,
        fresh,
        ageS,
        // AN AGE FACT, not a policy one. `fresh` now folds in whether the robot is ALLOWED to
        // shoot on odometry; this says only whether the camera is still confirming the fix,
        // which is what a TIP, an occlusion or looking away all look like.
        lostLock: have && ageS > this.spec.maxFireAgeS,
        scanning: false,
        id: stateNow,
        // "From odometry" now means "mostly from odometry": past the halfway point the
        // surveyed geometry is carrying more of the answer than the camera is.
        fromOdometry: wMeas < 0.5,
        stateFromTag: this.seenTag,
        anchor: { x: this.anchorX, y: this.anchorY },
        anchorFromTag: this.anchorSeen,
      };
    }

    const valid = have && ageS <= this.spec.holdS;
    if (!valid) {
      // ODOMETRY. The tag is gone, but the HIVE is not: it is bolted to the floor at a
      // position that is a field constant, and only which of its two CELLs is up changes. So
      // with a pose there is still a bearing and a range to aim at, and the robot keeps
      // working through a vision dropout instead of standing still hunting for a lens flare.
      //
      // WHAT ODOMETRY CANNOT DO is see the rocker. Until a tag has been decoded at least once
      // the state below is the one the match started on -- true at the buzzer, and quietly
      // wrong from the first TIP nobody watched. That is the whole reason `stateFromTag`
      // gates the shot rather than the aim: pointing at the wrong CELL costs nothing, firing
      // at it costs a ball and moves the HIVE for the other alliance.
      const stateId = this.seenTag ? this.fixId : this.spec.startId;
      // FROM THE TRACKED PIVOT, not from a baked field position. Before the first sighting
      // the pivot is the surveyed prior and this is the old behaviour; after it, the robot is
      // aiming at a HIVE it has measured for itself, and a TIP just swaps which offset it
      // adds. The only fixed coordinate left in the chain is where the pivot STARTS.
      const off = this.spec.mouthFromAnchor[stateId];
      const m = off && { x: this.anchorX + off.x, y: this.anchorY + off.y };
      if (m) {
        const dx = m.x - loc.x;
        const dy = m.y - loc.y;
        const azimuthDeg = wrapDeg(Math.atan2(dy, dx) * RAD - loc.heading);
        this.scanDeg = Math.max(this.spec.turretMinDeg, Math.min(this.spec.turretMaxDeg, azimuthDeg));
        const facing = this.spec.mouthFacingX[stateId] ?? 0;
        const d = Math.hypot(loc.x - m.x, loc.y - m.y);
        return {
          azimuthDeg,
          rangeIn: Math.hypot(dx, dy),
          openDeg: facing === 0 || d < 1e-6
            ? 180
            : Math.acos(Math.max(-1, Math.min(1, ((loc.x - m.x) * facing) / d))) * RAD,
          valid: true,
          fresh: this.spec.fireOnOdometry && this.seenTag && stateAgeOk,
          ageS,
          lostLock: have,
          scanning: false,
          id: stateId,
          fromOdometry: true,
          stateFromTag: this.seenTag,
          anchor: { x: this.anchorX, y: this.anchorY },
          anchorFromTag: this.anchorSeen,
        };
      }

      // SEARCH, for a robot with no pose at all -- `tryGet` returns no localizer on a bare
      // hub. The camera is on the turret, so with nothing to point at nothing is ever found.
      // Sweep the travel until a tag lands in the lens.
      this.scanDeg += this.scanDir * this.spec.scanRateDps * Math.max(dt, 1e-3);
      if (this.scanDeg >= this.spec.turretMaxDeg) {
        this.scanDeg = this.spec.turretMaxDeg;
        this.scanDir = -1;
      } else if (this.scanDeg <= this.spec.turretMinDeg) {
        this.scanDeg = this.spec.turretMinDeg;
        this.scanDir = 1;
      }
      return {
        azimuthDeg: this.scanDeg,
        rangeIn: 0,
        openDeg: 180,
        valid: false,
        fresh: false,
        ageS,
        lostLock: have,
        scanning: true,
        id: 0,
        fromOdometry: false,
        stateFromTag: this.seenTag,
        anchor: { x: this.anchorX, y: this.anchorY },
        anchorFromTag: this.anchorSeen,
      };
    }

    const dx = this.fixX - loc.x;
    const dy = this.fixY - loc.y;
    const rangeIn = Math.hypot(dx, dy);
    const azimuthDeg = wrapDeg(Math.atan2(dy, dx) * RAD - loc.heading);
    // Park the sweep where the target is, so losing the lock resumes the search from the last
    // place the tag was rather than from wherever the sweep had wandered to before.
    this.scanDeg = Math.max(this.spec.turretMinDeg, Math.min(this.spec.turretMaxDeg, azimuthDeg));
    return {
      azimuthDeg,
      rangeIn,
      // Carried, not propagated: the mouth's facing changes as the robot drives round it, but
      // over the few tenths a fix is allowed to live that change is small next to the sigma
      // the camera puts on tag yaw in the first place.
      openDeg: this.mouthOpenDeg(loc),
      valid: true,
      fresh,
      ageS,
      lostLock: !fresh,
      scanning: false,
      id: this.fixId,
      fromOdometry: false,
      stateFromTag: true,
      anchor: { x: this.anchorX, y: this.anchorY },
      anchorFromTag: this.anchorSeen,
    };
  }
}
