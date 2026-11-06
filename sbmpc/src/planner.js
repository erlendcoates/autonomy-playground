// Simulation-based MPC: predict every plan in a finite set with a model, score each by a cost, drop the ones that
// break a constraint, and keep the cheapest. Its first control is carried out for one step, then the set is predicted
// again from where the vehicle really is (the receding horizon).
//
// A plan is a control held constant over each of S segments of the horizon, segment s picking one control of its set
// (so there are |set 0| × |set 1| × … plans). Plan p's choice in segment s is the digit `Math.floor(p / span[s]) %
// sizes[s]`, segment 0 the most significant. Plans that share their first segments share the prediction of them, so
// the planner walks the plans as a tree: each segment is predicted once per prefix, and a prefix that breaks a
// constraint drops all the plans under it at once.

const now = typeof performance !== "undefined" ? () => performance.now() : () => Date.now();

/**
 * @typedef {import("./models.js").Model} Model
 * @typedef {{x0: Float64Array, t0: number, dt: number, N: number, model: Model, pos: number}} Context
 * @typedef {{name: string, prepare?: (ctx: Context) => void, stage: (k: number, x: Float64Array, u: Float64Array) => number,
 *            terminal?: (x: Float64Array) => number}} CostTerm
 * @typedef {{name: string, prepare?: (ctx: Context) => void,
 *            violated: (k: number, x: Float64Array, u: Float64Array) => boolean}} Constraint
 *
 * @typedef {Object} PlannerOptions
 * @property {Model} model               predicts the plans
 * @property {number} dt                  one step of the horizon (s)
 * @property {number[]} segments          the steps in each segment (they add up to the horizon N)
 * @property {Float64Array | Float64Array[]} controls  the controls a segment may pick from, `nu` numbers each: one set
 *   for every segment, or a set per segment (a set of one zero control makes a segment that coasts, say)
 * @property {CostTerm[]} [costs]         added up into a plan's cost
 * @property {Constraint[]} [constraints] a plan that breaks one is dropped
 * @property {number} [maxPlans=2e6]      refuse sets larger than this
 */

export class Planner {
  /** @param {PlannerOptions} options */
  constructor(options) {
    /** @type {CostTerm[]} */ this.costs = [];
    /** @type {Constraint[]} */ this.constraints = [];
    this.maxPlans = 2e6;
    this.configure(options);
  }

  /**
   * Change any of the options; what is left out stays. Reallocates the buffers when the plans change. The cost and
   * constraint arrays may also be changed in place between calls to `plan`.
   * @param {Partial<PlannerOptions>} options
   */
  configure(options = {}) {
    const o = {model: this.model, dt: this.dt, segments: this.segments, controls: this.controls, costs: this.costs,
               constraints: this.constraints, maxPlans: this.maxPlans, ...options};
    const {model, dt, segments, controls} = o;
    if (!model || typeof model.step !== "function") throw new TypeError("Planner: a model with step() is needed");
    if (!(dt > 0)) throw new RangeError("Planner: dt must be positive");
    if (!segments || !segments.length || segments.some((n) => !(Number.isInteger(n) && n >= 1)))
      throw new RangeError("Planner: segments must be whole numbers of steps, at least 1 each");
    const S = segments.length, nu = model.nu;
    const sets = Array.isArray(controls) ? controls : Array(S).fill(controls);
    if (sets.length !== S) throw new RangeError("Planner: one control set per segment, or one for all");
    for (const set of sets) {
      if (!set || !set.length || set.length % nu) throw new RangeError(`Planner: a control set must hold whole controls of ${nu}`);
    }
    const sizes = sets.map((set) => set.length / nu), count = sizes.reduce((a, n) => a * n, 1);
    if (count > o.maxPlans) throw new RangeError(`Planner: ${count} plans is more than maxPlans (${o.maxPlans})`);

    Object.assign(this, {model, dt, segments: segments.slice(), controls, costs: o.costs, constraints: o.constraints,
                         maxPlans: o.maxPlans});
    this.S = S; this.nu = nu; this.nx = model.nx; this.pos = model.pos ?? 0;
    this.N = segments.reduce((a, n) => a + n, 0);
    this.sizes = sizes; this.count = count;
    this.span = sizes.map((_, s) => sizes.slice(s + 1).reduce((a, n) => a * n, 1));   // plans under one choice at s
    this.segStart = segments.map((_, s) => segments.slice(0, s).reduce((a, n) => a + n, 0));
    this.segOf = new Int32Array(this.N);
    segments.forEach((n, s) => this.segOf.fill(s, this.segStart[s], this.segStart[s] + n));
    this.views = sets.map((set, s) => Array.from({length: sizes[s]}, (_, i) => set.subarray(i * nu, (i + 1) * nu)));

    // what plan() fills in, one entry per plan
    const N = this.N, stride = 3 * (N + 1);
    /** the cost (Infinity if dropped) */ this.J = new Float64Array(count);
    /** each cost term (count × number of terms; up to the step it broke a constraint, if dropped) */ this.terms = new Float64Array(0);
    /** the step at which it broke a constraint, or -1 */ this.hit = new Int32Array(count);
    /** which constraint it broke (an index into constraints), or -1 */ this.what = new Int16Array(count);
    /** the predicted positions, N + 1 points of 3 (after a break, the point where it broke, repeated) */
    this.traj = new Float32Array(count * stride);
    /** the plans kept, cheapest first (the first `feasible` entries) */ this.sorted = new Int32Array(count);
    /** each plan's place among those kept, 0 the cheapest to 1 the dearest (1 if dropped) */ this.rank = new Float32Array(count);
    this.best = -1; this.fallback = -1; this.feasible = 0; this.steps = 0; this.ms = 0; this.x0 = null; this.t0 = 0;

    this._x = new Float64Array(this.nx);
    this._xs = Array.from({length: S}, () => new Float64Array(this.nx));
    this._cur = new Float32Array(stride);
    this._tmp = new Float32Array(stride);
    this._nT = -1;
    return this;
  }

  /** The control plan `p` holds during step `k` (a view into its set: do not change it). */
  control(p, k) {
    const s = this.segOf[k];
    return this.views[s][Math.floor(p / this.span[s]) % this.sizes[s]];
  }

  /** Plan `p`'s choice in each segment (indices into the sets). */
  digits(p) {
    return this.sizes.map((n, s) => Math.floor(p / this.span[s]) % n);
  }

  /** The plan that picks control `digits[s]` in each segment s. */
  planOf(digits) {
    return digits.reduce((p, d, s) => p + d * this.span[s], 0);
  }

  /**
   * Predict plan `p` from x0 with the model and return every state, (N + 1) × nx numbers (not cut short at a broken
   * constraint).
   * @param {number} p
   * @param {ArrayLike<number>} [x0]  default: where the last plan() started
   * @param {Float64Array} [out]
   */
  rollout(p, x0 = this.x0, out = new Float64Array((this.N + 1) * this.nx)) {
    const nx = this.nx, x = Float64Array.from(x0);
    out.set(x, 0);
    for (let k = 0; k < this.N; k++) { this.model.step(x, this.control(p, k), this.dt); out.set(x, (k + 1) * nx); }
    return out;
  }

  /**
   * Predict, score and check every plan from the state x0 at time t0. Fills J, terms, hit, what, traj, sorted and
   * rank, and sets `best` (the cheapest plan kept, or -1 if every plan is dropped), `fallback` (when none is kept,
   * the plan that breaks a constraint latest, cheapest first among ties), `feasible` (how many are kept), `steps`
   * (model steps taken) and `ms` (time taken). The arrays are reused by the next call.
   * @param {ArrayLike<number>} x0
   * @param {number} [t0=0]
   * @returns {this}
   */
  plan(x0, t0 = 0) {
    const tic = now(), {count, N, nx, pos} = this, nT = this.costs.length;
    if (x0.length < nx) throw new RangeError(`Planner.plan: the state has ${nx} numbers`);
    if (nT !== this._nT) { this.terms = new Float64Array(count * Math.max(nT, 1)); this._acc = Array.from({length: this.S}, () => new Float64Array(nT)); this._a = new Float64Array(nT); this._nT = nT; }
    this.x0 = Float64Array.from(x0).subarray(0, nx); this.t0 = t0;
    const ctx = {x0: this.x0, t0, dt: this.dt, N, model: this.model, pos};
    for (const c of this.costs) if (c.prepare) c.prepare(ctx);
    for (const c of this.constraints) if (c.prepare) c.prepare(ctx);

    this._xs[0].set(this.x0); this._acc[0].fill(0);
    this._cur[0] = this.x0[pos]; this._cur[1] = this.x0[pos + 1]; this._cur[2] = this.x0[pos + 2];
    this.steps = 0;
    this._descend(0, 0);

    // rank the plans kept
    let f = 0;
    for (let p = 0; p < count; p++) if (this.hit[p] < 0) this.sorted[f++] = p;
    const J = this.J, kept = this.sorted.subarray(0, f);
    kept.sort((a, b) => J[a] - J[b] || a - b);
    this.rank.fill(1);
    for (let i = 0; i < f; i++) this.rank[kept[i]] = f > 1 ? i / (f - 1) : 0;
    this.feasible = f;
    this.best = f ? kept[0] : -1;
    this.fallback = this.best;
    if (this.best < 0) {
      let bh = -1, bc = Infinity;
      for (let p = 0; p < count; p++) {
        let c = 0;
        for (let j = 0; j < nT; j++) c += this.terms[p * nT + j];
        if (this.hit[p] > bh || (this.hit[p] === bh && c < bc)) { bh = this.hit[p]; bc = c; this.fallback = p; }
      }
    }
    this.ms = now() - tic;
    return this;
  }

  // predict the choices of segment s for the plans from `base` on, from the state and costs reached so far
  _descend(s, base) {
    const {model, dt, constraints, costs, N, pos} = this, x = this._x, cur = this._cur, a = this._a;
    const nC = constraints.length, nT = costs.length, k0 = this.segStart[s], k1 = k0 + this.segments[s];
    const opts = this.views[s], sp = this.span[s], last = s === this.S - 1, stride = 3 * (N + 1);
    for (let i = 0; i < opts.length; i++) {
      const u = opts[i], p0 = base + i * sp;
      x.set(this._xs[s]); a.set(this._acc[s]);
      let hit = -1, what = -1;
      for (let k = k0; k < k1; k++) {
        model.step(x, u, dt);
        this.steps++;
        const o = 3 * (k + 1);
        cur[o] = x[pos]; cur[o + 1] = x[pos + 1]; cur[o + 2] = x[pos + 2];
        for (let c = 0; c < nC; c++) if (constraints[c].violated(k, x, u)) { what = c; break; }
        if (what >= 0) { hit = k; break; }
        for (let j = 0; j < nT; j++) a[j] += costs[j].stage(k, x, u);
      }
      if (hit >= 0) { this._drop(p0, sp, hit, what); continue; }
      if (last) {
        let J = 0;
        for (let j = 0; j < nT; j++) {
          if (costs[j].terminal) a[j] += costs[j].terminal(x);
          this.terms[p0 * nT + j] = a[j]; J += a[j];
        }
        this.J[p0] = J; this.hit[p0] = -1; this.what[p0] = -1;
        this.traj.set(cur, p0 * stride);
      } else {
        this._xs[s + 1].set(x); this._acc[s + 1].set(a);
        this._descend(s + 1, p0);
      }
    }
  }

  // drop the `n` plans from p0 on: they share the prediction up to step k, where it broke constraint `what`
  _drop(p0, n, k, what) {
    const nT = this.costs.length, stride = 3 * (this.N + 1), cur = this._cur, tmp = this._tmp, a = this._a;
    const e = 3 * (k + 2);
    tmp.set(cur.subarray(0, e));
    for (let o = e; o < stride; o += 3) { tmp[o] = cur[e - 3]; tmp[o + 1] = cur[e - 2]; tmp[o + 2] = cur[e - 1]; }
    for (let p = p0; p < p0 + n; p++) {
      this.J[p] = Infinity; this.hit[p] = k; this.what[p] = what;
      for (let j = 0; j < nT; j++) this.terms[p * nT + j] = a[j];
      this.traj.set(tmp, p * stride);
    }
  }
}
