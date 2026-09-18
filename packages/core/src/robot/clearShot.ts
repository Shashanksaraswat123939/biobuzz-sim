/**
 * Is the OTHER HIVE in the way?
 *
 * THE MIRROR of the same test in java/teamcode/.../control/AimController.java.
 *
 * The two HIVEs sit 25.5 in apart along the pivot axis and each CELL is 20 in wide, so they
 * very nearly touch. A shot at your own CELL taken from the far side of theirs has to cross
 * their structure, and nothing in the shot solver ever knew that: `bestShot` checks the near
 * and far lip of the TARGET mouth and integrates a ball through empty air. The physics knows
 * -- both rockers carry colliders -- so the ball really does hit, and the aim really does
 * choose to throw it there. Measured over the shot zone, 5% of the positions the table is
 * happy with are blocked.
 *
 * THE OPPONENT'S ROCKER STATE DOES NOT MATTER, which is the piece of luck that makes this
 * cheap. Both of their CELLs swing about their pivot at a fixed radius, so the volume the
 * structure can occupy is a DISC, and a disc is the same in both states. There is no need to
 * see their tag, track their tips, or take the worst of two cases: one shape covers it.
 *
 * ponytail: a ground-plane test, refusing any shot whose path crosses the footprint, rather
 * than asking whether the arc clears the top. That is CONSERVATIVE and knowingly so -- the
 * table's apex is 59-61 in and the obstacle's top runs 60 in at the edges of the disc to 66
 * at its centre, so a shot over the shallow end would sometimes get through. Buying those
 * back means carrying the arc's height profile onto the hub, which is a shot table with a
 * second axis; refusing them costs a few positions on one side of the field and needs four
 * baked numbers. Revisit if that side of the field ever matters.
 */

export interface ObstacleSpec {
  /** The opposing HIVE's pivot, FTC inches. Its CELLs sweep a disc about this. */
  x: number;
  y: number;
  /** Radius of that disc: the CELL mouth's distance from the pivot. */
  radius_in: number;
  /** Half the CELL's width along the pivot axis, which is the FTC Y direction. */
  halfWidth_in: number;
}

/**
 * Does the ground path from the robot to the mouth cross the opposing HIVE's footprint?
 *
 * The footprint is a box: `radius_in` either side of their pivot along FTC X, `halfWidth_in`
 * along FTC Y. A standard segment-versus-box test, in the one plane that matters.
 */
export function shotIsBlocked(
  fromX: number,
  fromY: number,
  toX: number,
  toY: number,
  o: ObstacleSpec,
): boolean {
  const minX = o.x - o.radius_in;
  const maxX = o.x + o.radius_in;
  const minY = o.y - o.halfWidth_in;
  const maxY = o.y + o.halfWidth_in;

  // Trivial accept: the muzzle is already inside it, which means something has gone wrong
  // elsewhere, but a shot from inside the other hive is certainly blocked.
  const inside = (x: number, y: number) => x >= minX && x <= maxX && y >= minY && y <= maxY;
  if (inside(fromX, fromY) || inside(toX, toY)) return true;

  // Slab clipping. `t` runs 0 at the muzzle to 1 at the mouth; the segment crosses the box
  // when the X and Y intervals it is inside overlap anywhere in [0, 1].
  const dx = toX - fromX;
  const dy = toY - fromY;
  let t0 = 0;
  let t1 = 1;
  const slab = (p: number, d: number, lo: number, hi: number): boolean => {
    if (Math.abs(d) < 1e-9) return p >= lo && p <= hi;   // parallel: in or out for good
    const a = (lo - p) / d;
    const b = (hi - p) / d;
    t0 = Math.max(t0, Math.min(a, b));
    t1 = Math.min(t1, Math.max(a, b));
    return t1 >= t0;
  };
  if (!slab(fromX, dx, minX, maxX)) return false;
  if (!slab(fromY, dy, minY, maxY)) return false;
  return t1 >= t0;
}
