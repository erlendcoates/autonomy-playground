// A path of straight legs between 3D points: the nearest point on it, a point a given distance along it, and the
// part of it ahead of a point.

export class Path3 {
  /**
   * @param {ArrayLike<number>[]} points  the corners, each [x, y, z] (z may be left out: 0)
   * @param {{closed?: boolean}} [options]  closed: the last corner joins the first
   */
  constructor(points, {closed = false} = {}) {
    if (!points || points.length < 2) throw new RangeError("Path3: needs at least two points");
    this.points = points.map((p) => [p[0], p[1], p[2] ?? 0]);
    this.closed = closed;
    /** @type {{ax: number, ay: number, az: number, tx: number, ty: number, tz: number, L: number, s0: number}[]} */
    this.segs = [];
    const P = this.points, n = closed ? P.length : P.length - 1;
    let s = 0;
    for (let i = 0; i < n; i++) {
      const a = P[i], b = P[(i + 1) % P.length], dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
      const L = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (L <= 1e-9) continue;                                   // a repeated corner adds no leg
      this.segs.push({ax: a[0], ay: a[1], az: a[2], tx: dx / L, ty: dy / L, tz: dz / L, L, s0: s});
      s += L;
    }
    if (!this.segs.length) {                                     // all one point: a path of length 0
      const a = P[0];
      this.segs.push({ax: a[0], ay: a[1], az: a[2], tx: 1, ty: 0, tz: 0, L: 0, s0: 0});
    }
    this.length = s;
  }

  /**
   * The point on the path nearest (x, y, z). Ties go to the earlier leg.
   * @returns {{s: number, i: number, u: number, d2: number}} distance along the path, the leg, the distance along
   *   the leg, and the squared distance to the point
   */
  project(x, y, z) {
    let best = Infinity, bi = 0, bu = 0;
    const segs = this.segs;
    for (let i = 0; i < segs.length; i++) {
      const g = segs[i], wx = x - g.ax, wy = y - g.ay, wz = z - g.az;
      let u = wx * g.tx + wy * g.ty + wz * g.tz;
      u = u < 0 ? 0 : u > g.L ? g.L : u;
      const dx = wx - u * g.tx, dy = wy - u * g.ty, dz = wz - u * g.tz, d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < best) { best = d2; bi = i; bu = u; }
    }
    return {s: segs[bi].s0 + bu, i: bi, u: bu, d2: best};
  }

  /**
   * The leg nearest (x, y, z), as stored in `segs` (allocates nothing): its start (ax, ay, az), unit direction
   * (tx, ty, tz), length L and distance along the path s0.
   */
  nearestLeg(x, y, z) {
    let best = Infinity, bi = 0;
    const segs = this.segs;
    for (let i = 0; i < segs.length; i++) {
      const g = segs[i], wx = x - g.ax, wy = y - g.ay, wz = z - g.az;
      let u = wx * g.tx + wy * g.ty + wz * g.tz;
      u = u < 0 ? 0 : u > g.L ? g.L : u;
      const dx = wx - u * g.tx, dy = wy - u * g.ty, dz = wz - u * g.tz, d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < best) { best = d2; bi = i; }
    }
    return segs[bi];
  }

  /** The squared distance from (x, y, z) to the path (allocates nothing: fit for a cost's inner loop). */
  distance2(x, y, z) {
    let best = Infinity;
    const segs = this.segs;
    for (let i = 0; i < segs.length; i++) {
      const g = segs[i], wx = x - g.ax, wy = y - g.ay, wz = z - g.az;
      let u = wx * g.tx + wy * g.ty + wz * g.tz;
      u = u < 0 ? 0 : u > g.L ? g.L : u;
      const dx = wx - u * g.tx, dy = wy - u * g.ty, dz = wz - u * g.tz, d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < best) best = d2;
    }
    return best;
  }

  _leg(s) {                       // the leg holding distance s (s already wrapped or clamped)
    const segs = this.segs;
    let lo = 0, hi = segs.length - 1;
    while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (segs[mid].s0 <= s) lo = mid; else hi = mid - 1; }
    return segs[lo];
  }

  _wrap(s) {
    const L = this.length;
    if (L === 0) return 0;
    return this.closed ? ((s % L) + L) % L : s < 0 ? 0 : s > L ? L : s;
  }

  /**
   * The point `s` along the path (wrapped round a closed path, held at the ends of an open one), written to
   * `out[o..o+2]`.
   */
  pointAt(s, out = [0, 0, 0], o = 0) {
    s = this._wrap(s);
    const g = this._leg(s), u = Math.min(s - g.s0, g.L);
    out[o] = g.ax + u * g.tx; out[o + 1] = g.ay + u * g.ty; out[o + 2] = g.az + u * g.tz;
    return out;
  }

  /** The unit direction of the path `s` along it, written to `out[o..o+2]`. */
  tangentAt(s, out = [0, 0, 0], o = 0) {
    const g = this._leg(this._wrap(s));
    out[o] = g.tx; out[o + 1] = g.ty; out[o + 2] = g.tz;
    return out;
  }

  /**
   * The path ahead of (x, y, z): from the point on it nearest that, `len` on (round the loop of a closed path, up to
   * the end of an open one), as an open path. A cost measured from this cannot be lowered by going the wrong way.
   * @returns {Path3}
   */
  ahead(x, y, z, len) {
    const {i, u} = this.project(x, y, z), segs = this.segs, pts = [];
    let si = i, su = u, left = len;
    const g0 = segs[si];
    pts.push([g0.ax + su * g0.tx, g0.ay + su * g0.ty, g0.az + su * g0.tz]);
    while (left > 0) {
      const g = segs[si], L = Math.min(g.L - su, left), v = su + L;
      if (L > 1e-9) pts.push([g.ax + v * g.tx, g.ay + v * g.ty, g.az + v * g.tz]);
      left -= L;
      if (left <= 0) break;
      if (!this.closed && si === segs.length - 1) break;
      si = (si + 1) % segs.length; su = 0;
    }
    if (pts.length < 2) pts.push(pts[0].slice());                // at the end of an open path: a single point
    return new Path3(pts);
  }
}
