// The scene and the simulation of the 3D app, with no drawing (it runs in Node as well): a quadrotor follows a loop of
// straight legs by simulation-based MPC from the sbmpc kernel, past another drone and obstacles.
//
// Coordinates are north-east-down (NED), as in the 2D app and the videos: p = [north, east, down] in metres, so the
// altitude is h = -p[2]. The kernel does not mind which frame it is given.
import {Planner, Path3, lattice, zeros, models, costs, constraints, shapes} from "../sbmpc/src/index.js";

export const DT = 0.25;                 // s, one step of the model and of the replanning
export const H = 0.02;                  // s, the simulation's step
export const H_MIN = 1;                 // m, the lowest altitude a plan may reach
export const AHEAD = 15;                // m, where "… ahead" puts an obstacle, along the path from the drone

// the path's corners: north, east and altitude (m)
export const LOOP = [[0, 0, 8], [70, 0, 8], [100, 35, 14], [85, 90, 20], [30, 100, 14], [-15, 55, 8]];
export const PATH = new Path3(LOOP.map(([n, e, h]) => [n, e, -h]), {closed: true});
export const START = {p: [5, -8, -4], v: [3, 0, 0]};          // 8 m to the left of the path and 4 m under it
// the other drone flies back and forth between a and b, across the first leg, climbing as it goes east
export const INTRUDER = {a: [42, -25, -6], b: [42, 25, -11], phase0: 0};

export const DEFAULTS = {
  T: 5, S: 2, Tseg: 1.5, nh: 3, nv: 3, amax: 2.5,             // the plans
  lam: 0.2, U: 4,                                             // the cost: what accelerating costs, the speed on the path
  vmax: 8, intruder: true, dsafe: 5, vi: 2.5, dclear: 1.5,    // the constraints
  tau: 0.25, wind: 0,                                         // the drone, beyond the model
};
export const MAX_PLANS = 20000;

/**
 * The segments: S of Tseg each (shortened to fit the horizon), then a coast (no acceleration) to the horizon.
 */
export function schedule(P) {
  const N = Math.max(1, Math.round(P.T / DT)), S = Math.min(P.S, N);
  const ns = Math.max(1, Math.min(Math.round(P.Tseg / DT), Math.floor(N / S)));
  const segs = Array(S).fill(ns), rest = N - S * ns;
  if (rest > 0) segs.push(rest);
  return {N, S, ns, segs, coast: rest > 0, perSeg: P.nh * P.nh * P.nv, count: (P.nh * P.nh * P.nv) ** S};
}

/** Signed offsets from the path: across (horizontal, > 0 to the right of the way it is flown) and above (> 0 higher). */
export function offPath(x, y, z) {
  const q = PATH.project(x, y, z), g = PATH.segs[q.i];
  const wx = x - (g.ax + q.u * g.tx), wy = y - (g.ay + q.u * g.ty), wz = z - (g.az + q.u * g.tz);
  const th = Math.hypot(g.tx, g.ty);
  const across = th > 1e-9 ? (g.tx * wy - g.ty * wx) / th : Math.hypot(wx, wy);
  return {across, above: -wz};
}

// obstacles: the shape, and the constraint that keeps the plans dclear from it
let nextId = 1;
function obstacle(kind, shape) {
  return {id: nextId++, kind, shape, c: constraints.keepOut({shape, margin: DEFAULTS.dclear, name: `the ${kind}`})};
}
export const make = {
  balloon: (n, e, h, r = 2.5) => obstacle("balloon", shapes.sphere([n, e, -h], r)),
  building: (n, e, height, half = [5, 5]) => obstacle("building", shapes.box([n, e, -height / 2], [half[0], half[1], height / 2])),
  pillar: (n, e, height, r = 1.2) => obstacle("pillar", shapes.cylinder([n, e, -height / 2], r, height / 2, 2)),
};
export function defaultObstacles() {
  return [make.building(93, 62, 16, [6, 5]), make.balloon(8, 78, 11), make.pillar(-4, 26, 24, 1.5)];
}

export class Sim {
  /** @param {typeof DEFAULTS} P  the settings, read live */
  constructor(P) {
    this.P = P;
    this.obstacles = defaultObstacles();
    this.ia = Float64Array.from(INTRUDER.a); this.ib = Float64Array.from(INTRUDER.b);
    this.model = models.pointMass3();
    this.track = costs.trackPath({path: PATH, speed: P.U, name: "the path"});
    this.effort = costs.effort({weight: P.lam, name: "accelerating"});
    this.intr = constraints.keepOut({shape: shapes.sphere([0, 0, 0], 0), margin: P.dsafe, velocity: [0, 0, 0], name: "the other drone"});
    this.ground = constraints.halfSpace({normal: [0, 0, 1], offset: -H_MIN, name: "the ground"});
    this.speed = constraints.maxNorm({index: 3, max: P.vmax, name: "the speed limit"});
    this.version = 0;
    this.rebuild(false);
    this.reset();
  }

  /** Make the plans again from the settings (horizon, segments, the lattice). */
  rebuild(replan = true) {
    const P = this.P, sc = schedule(P);
    this.sched = sc;
    this.lat = lattice([P.nh, P.nh, P.nv], [P.amax, P.amax, P.amax]);
    const controls = sc.segs.map((_, s) => (sc.coast && s === sc.segs.length - 1 ? zeros(3) : this.lat));
    const opts = {model: this.model, dt: DT, segments: sc.segs, controls, costs: [this.track, this.effort],
                  constraints: this.constraintList(), maxPlans: MAX_PLANS};
    if (this.planner) this.planner.configure(opts); else this.planner = new Planner(opts);
    this.plans = [];
    if (replan) this.replan();
  }

  constraintList() {
    return [...(this.P.intruder ? [this.intr] : []), this.ground, this.speed, ...this.obstacles.map((o) => o.c)];
  }

  reset() {
    Object.assign(this, {
      t: 0, p: Float64Array.from(START.p), v: Float64Array.from(START.v), aAct: new Float64Array(3),
      aCmd: new Float64Array(3), tNext: 0, phase: INTRUDER.phase0, none: false, chosen: -1, hist: [], trail: [],
      plans: [], closest: Infinity, clearest: Infinity, tLog: -1, tTrail: -1, grounded: false,
    });
    this.replan();
  }

  // ── the other drone: back and forth between ia and ib at P.vi ─────────────────────────────────────────────────────
  intruderAt(t) {
    const a = this.ia, b = this.ib, d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], L = Math.hypot(...d);
    const u = d.map((x) => x / L), vi = this.P.vi, cyc = 2 * L;
    const s = (((this.phase + vi * t) % cyc) + cyc) % cyc;
    const along = s < L ? s : cyc - s, dir = s < L ? 1 : -1;
    return {p: u.map((x, i) => a[i] + along * x), v: u.map((x) => dir * vi * x), along, L, u};
  }
  /** Put the other drone `along` m from a, going the way it is going now. */
  setIntruderAlong(along) {
    const it = this.intruderAt(this.t), L = it.L, x = Math.min(Math.max(along, 0), L);
    const s = it.v.some((c, i) => c * it.u[i] < 0) ? 2 * L - x : x;
    this.phase = s - this.P.vi * this.t;
  }
  /** Keep the other drone where it is when its speed changes. */
  keepIntruder(viOld) {
    const s = this.phase + viOld * this.t;
    this.phase = s - this.P.vi * this.t;
  }

  // ── planning ────────────────────────────────────────────────────────────────────────────────────────────────────────
  replan() {
    const P = this.P, pl = this.planner;
    this.track.speed = P.U; this.effort.weight = P.lam; this.speed.max = P.vmax;
    if (P.intruder) {
      const it = this.intruderAt(this.t);
      this.intr.shape.center.set(it.p); this.intr.velocity = it.v; this.intr.margin = P.dsafe;
      this.seen = it;                                   // where it was when the plans were made, and its velocity
    } else this.seen = null;
    for (const o of this.obstacles) o.c.margin = P.dclear;
    pl.constraints = this.constraintList();
    pl.plan([...this.p, ...this.v], this.t);
    this.none = pl.best < 0;
    this.chosen = this.none ? pl.fallback : pl.best;
    this.aCmd.set(pl.control(this.chosen, 0));
    this.chosenX = pl.rollout(this.chosen);
    const X = this.chosenX, rec = {t0: this.t, across: [], above: [], speed: [], acc: []};
    for (let k = 0; k <= pl.N; k++) {
      const o = offPath(X[6 * k], X[6 * k + 1], X[6 * k + 2]);
      rec.across.push(o.across); rec.above.push(o.above);
      rec.speed.push(Math.hypot(X[6 * k + 3], X[6 * k + 4], X[6 * k + 5]));
      if (k < pl.N) { const u = pl.control(this.chosen, k); rec.acc.push([u[0], u[1], -u[2]]); }
    }
    this.plans.push(rec);
    if (this.plans.length > 4) this.plans.shift();
    this.tNext = this.t + DT;
    this.version++;
  }

  // ── the drone: the model, plus a lag on the acceleration and a push from the wind, neither in the model ───────────
  integrate(h) {
    const P = this.P, a = this.aAct, c = this.aCmd, p = this.p, v = this.v;
    for (let i = 0; i < 3; i++) a[i] = P.tau > 0 ? a[i] + h * (c[i] - a[i]) / P.tau : c[i];
    const w = [0, P.wind, 0];
    for (let i = 0; i < 3; i++) { const acc = a[i] + w[i]; p[i] += h * v[i] + 0.5 * h * h * acc; v[i] += h * acc; }
    if (p[2] > 0) { p[2] = 0; if (v[2] > 0) v[2] = 0; this.grounded = true; }
    this.t += h;
    if (P.intruder) {
      const q = this.intruderAt(this.t).p;
      this.closest = Math.min(this.closest, Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]));
    }
    for (const o of this.obstacles) this.clearest = Math.min(this.clearest, o.shape.sdf(p[0], p[1], p[2]));
    if (this.t - this.tLog >= 0.1 - 1e-9) {
      this.tLog = this.t;
      const o = offPath(p[0], p[1], p[2]);
      this.hist.push({t: this.t, across: o.across, above: o.above, speed: Math.hypot(...v),
                      cmd: [c[0], c[1], -c[2]], act: [a[0], a[1], -a[2]]});
      while (this.hist.length && this.hist[0].t < this.t - 40) this.hist.shift();
    }
    if (this.t - this.tTrail >= 0.2 - 1e-9) {
      this.tTrail = this.t;
      this.trail.push([p[0], p[1], p[2]]);
      if (this.trail.length > 1500) this.trail.shift();
    }
  }

  advance(dt) {
    for (let left = dt; left > 1e-9; left -= H) {
      if (this.t >= this.tNext - 1e-9) this.replan();
      this.integrate(Math.min(H, left));
    }
  }

  /** Carry out one step of the plan, then plan again. */
  stepOnce() {
    while (this.t < this.tNext - 1e-9) this.integrate(Math.min(H, this.tNext - this.t));
    this.replan();
  }

  /** Where "… ahead" puts something: on the path, AHEAD m on from the point nearest the drone. */
  ahead() {
    const q = PATH.project(this.p[0], this.p[1], this.p[2]);
    return PATH.pointAt(q.s + AHEAD);
  }
  addAhead(kind) {
    const [n, e, d] = this.ahead(), h = -d;
    const o = kind === "pillar" ? make.pillar(n, e, h + 12) : kind === "building" ? make.building(n, e, h + 4, [4, 4])
      : make.balloon(n, e, h, 2);
    this.obstacles.push(o);
    this.replan();
    return o;
  }
  removeObstacle(id) {
    this.obstacles = this.obstacles.filter((o) => o.id !== id);
    this.replan();
  }
}
