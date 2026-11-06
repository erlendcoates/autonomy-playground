// Constraints. A plan is dropped at the first step k whose state x breaks one: `violated(k, x, u)` returns true (x the
// state after step k, u the control held during it). They are checked in the order given, so the first one listed is
// the one reported when a step breaks several. Limits on the controls themselves belong in the control sets instead.
// `prepare(ctx)` runs once before the plans are checked (to predict where moving things will be at each step).

/**
 * Keep a point of the state at least `margin` from a shape (outside it, measured from its surface). With a velocity,
 * the shape is predicted straight on from where it is now: at step k it has moved (k + 1) dt velocity.
 * @param {{shape: import("./shapes.js").Shape, margin?: number, velocity?: ArrayLike<number> | null, name?: string}} options
 */
export function keepOut({shape, margin = 0, velocity = null, name = shape.kind}) {
  return {
    name, shape, margin, velocity, off: null, pos: 0,
    prepare(ctx) {
      const {N, dt, pos} = ctx, v = this.velocity;
      this.pos = pos;
      if (v && (v[0] || v[1] || v[2])) {
        if (!this.off || this.off.length !== 3 * N) this.off = new Float64Array(3 * N);
        for (let k = 0; k < N; k++) for (let i = 0; i < 3; i++) this.off[3 * k + i] = (k + 1) * dt * v[i];
      } else this.off = null;
    },
    violated(k, x) {
      const p = this.pos;
      let px = x[p], py = x[p + 1], pz = x[p + 2];
      if (this.off) { const o = 3 * k; px -= this.off[o]; py -= this.off[o + 1]; pz -= this.off[o + 2]; }
      return this.shape.sdf(px, py, pz) < this.margin;
    },
  };
}

/**
 * Keep a point of the state at least `distance` from something whose position at any time `predict(t)` says (another
 * vehicle on a known route, say). Predicted once per plan set, at t0 + (k + 1) dt.
 * @param {{predict: (t: number) => ArrayLike<number>, distance: number, name?: string}} options
 */
export function keepAway({predict, distance, name = "moving"}) {
  return {
    name, predict, distance, at: new Float64Array(0), pos: 0,
    prepare(ctx) {
      const {N, dt, t0, pos} = ctx;
      this.pos = pos;
      if (this.at.length !== 3 * N) this.at = new Float64Array(3 * N);
      for (let k = 0; k < N; k++) {
        const q = this.predict(t0 + (k + 1) * dt);
        this.at[3 * k] = q[0]; this.at[3 * k + 1] = q[1]; this.at[3 * k + 2] = q[2];
      }
    },
    violated(k, x) {
      const p = this.pos, o = 3 * k, a = this.at;
      const dx = x[p] - a[o], dy = x[p + 1] - a[o + 1], dz = x[p + 2] - a[o + 2];
      return dx * dx + dy * dy + dz * dz < this.distance * this.distance;
    },
  };
}

/**
 * Keep a point of the state on one side of a plane: normal · p ≤ offset (a floor or a ceiling).
 * @param {{normal: ArrayLike<number>, offset: number, name?: string}} options
 */
export function halfSpace({normal, offset, name = "half-space"}) {
  return {
    name, normal: Float64Array.from(normal), offset, pos: 0,
    prepare(ctx) { this.pos = ctx.pos; },
    violated(k, x) {
      const p = this.pos, n = this.normal;
      return n[0] * x[p] + n[1] * x[p + 1] + n[2] * x[p + 2] > this.offset;
    },
  };
}

/**
 * Keep the length of `size` numbers of the state, from `index` on, at most `max` (a speed limit, say).
 * @param {{index: number, size?: number, max: number, name?: string}} options
 */
export function maxNorm({index, size = 3, max, name = "limit"}) {
  return {
    name, index, size, max,
    violated(k, x) {
      let s = 0;
      for (let i = this.index, e = this.index + this.size; i < e; i++) s += x[i] * x[i];
      return s > this.max * this.max;
    },
  };
}
