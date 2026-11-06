// Cost terms. A plan's cost is the sum of its terms, each the sum over the steps of the horizon of `stage(k, x, u)`:
// x the state after step k, u the control held during it (and, if a term has one, `terminal(x)` of the last state).
// `prepare(ctx)` runs once before the plans are scored, with where they start from, to work out what every plan
// shares (a reference, say). Weights and other settings are plain properties: change them between plans.

/**
 * λ Σ |u_k|²: what the controls cost.
 * @param {{weight?: number, name?: string}} [options]
 */
export function effort({weight = 1, name = "effort"} = {}) {
  return {
    name, weight,
    stage(k, x, u) {
      let s = 0;
      for (let i = 0; i < u.length; i++) s += u[i] * u[i];
      return this.weight * s;
    },
  };
}

/**
 * w Σ |p_k − r_k|²: the squared distance from a reference point that sets off from the point on the path nearest the
 * start and moves along the path at `speed` (r_k is (k + 1) dt speed further on). Tracks the path and the speed on it.
 * @param {{path: import("./path.js").Path3, speed: number, weight?: number, name?: string}} options
 */
export function trackPath({path, speed, weight = 1, name = "path"}) {
  return {
    name, path, speed, weight, ref: new Float64Array(0), s0: 0, pos: 0,
    prepare(ctx) {
      const {x0, N, dt, pos} = ctx;
      if (this.ref.length !== 3 * N) this.ref = new Float64Array(3 * N);
      this.pos = pos;
      this.s0 = this.path.project(x0[pos], x0[pos + 1], x0[pos + 2]).s;
      for (let k = 0; k < N; k++) this.path.pointAt(this.s0 + this.speed * (k + 1) * dt, this.ref, 3 * k);
    },
    stage(k, x) {
      const r = this.ref, o = 3 * k, p = this.pos;
      const dx = x[p] - r[o], dy = x[p + 1] - r[o + 1], dz = x[p + 2] - r[o + 2];
      return this.weight * (dx * dx + dy * dy + dz * dz);
    },
  };
}

/**
 * w Σ |v_k − U t_k|²: how far the velocity is from `speed` along the path, t_k the direction of the leg of the path
 * ahead (from the point nearest the start, `ahead` on) nearest p_k. With `pathDistance`, a path-following cost for
 * models that choose their own speed: it rewards going along the path, not chasing a point on it.
 * @param {{path: import("./path.js").Path3, speed: number, ahead: number | ((ctx: object) => number), weight?: number,
 *          vel?: number, name?: string}} options  vel: where the velocity is in the state (default the model's `vel`)
 */
export function pathVelocity({path, speed, ahead, weight = 1, vel, name = "velocity"}) {
  return {
    name, path, speed, ahead, weight, vel, sub: path, pos: 0, v: 3,
    prepare(ctx) {
      const {x0, pos, model} = ctx, len = typeof this.ahead === "function" ? this.ahead(ctx) : this.ahead;
      this.pos = pos;
      this.v = this.vel ?? model.vel ?? pos + 3;
      this.sub = this.path.ahead(x0[pos], x0[pos + 1], x0[pos + 2], len);
    },
    stage(k, x) {
      const p = this.pos, v = this.v, U = this.speed, g = this.sub.nearestLeg(x[p], x[p + 1], x[p + 2]);
      const dx = x[v] - U * g.tx, dy = x[v + 1] - U * g.ty, dz = x[v + 2] - U * g.tz;
      return this.weight * (dx * dx + dy * dy + dz * dz);
    },
  };
}

/**
 * w Σ d_k²: the squared distance from the path ahead of the start (from the point nearest it, `ahead` on), as in the
 * 2D app. Says nothing about speed: for models that keep their own, or with `pathVelocity`.
 * @param {{path: import("./path.js").Path3, ahead: number | ((ctx: object) => number), weight?: number, name?: string}} options
 */
export function pathDistance({path, ahead, weight = 1, name = "path"}) {
  return {
    name, path, ahead, weight, sub: path, pos: 0,
    prepare(ctx) {
      const {x0, pos} = ctx, len = typeof this.ahead === "function" ? this.ahead(ctx) : this.ahead;
      this.pos = pos;
      this.sub = this.path.ahead(x0[pos], x0[pos + 1], x0[pos + 2], len);
    },
    stage(k, x) {
      const p = this.pos;
      return this.weight * this.sub.distance2(x[p], x[p + 1], x[p + 2]);
    },
  };
}
