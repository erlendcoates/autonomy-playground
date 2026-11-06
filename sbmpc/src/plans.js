// Building blocks for plan sets: the controls a segment may pick from, and how the horizon is split into segments.

/**
 * `n` evenly spaced values from `a` to `b`. A single value is the midpoint.
 * @param {number} a
 * @param {number} b
 * @param {number} n
 * @returns {number[]}
 */
export function linspace(a, b, n) {
  if (!(n >= 1)) throw new RangeError("linspace: n must be at least 1");
  if (n === 1) return [(a + b) / 2];
  return Array.from({length: n}, (_, i) => a + (b - a) * i / (n - 1));
}

/**
 * A lattice of controls: every combination of `levels[i]` evenly spaced values from `min[i]` to `max[i]` on each
 * axis, the first axis the most significant (so control `j` of a 3-axis lattice has the indices
 * `j = (i0 * levels[1] + i1) * levels[2] + i2`).
 *
 *   lattice([3, 3, 3], [2, 2, 2])   // 27 accelerations, each axis -2, 0 or 2
 *   lattice([9], [8 * Math.PI / 180])   // 9 turn rates from -8 to 8 deg/s
 *
 * @param {number[]} levels  values on each axis (1 gives the midpoint only)
 * @param {number[]} max     the largest value on each axis
 * @param {number[]} [min]   the smallest (default `-max`)
 * @returns {Float64Array} `count * dims` numbers, one control after another
 */
export function lattice(levels, max, min = max.map((v) => -v)) {
  const dims = levels.length;
  if (max.length !== dims || min.length !== dims) throw new RangeError("lattice: levels, max and min differ in length");
  const axes = levels.map((n, i) => linspace(min[i], max[i], n));
  const count = levels.reduce((a, n) => a * n, 1), out = new Float64Array(count * dims);
  for (let j = 0; j < count; j++) {
    for (let d = dims - 1, q = j; d >= 0; d--) { out[j * dims + d] = axes[d][q % levels[d]]; q = Math.floor(q / levels[d]); }
  }
  return out;
}

/**
 * A single control of zeros: a segment that may only pick it holds the model unforced (a coast, say).
 * @param {number} nu  the control's size
 * @returns {Float64Array}
 */
export function zeros(nu) {
  return new Float64Array(nu);
}

/**
 * The 2D app's split of a horizon of `N` steps into `S` segments: the first one `first` steps long, the rest shared
 * out evenly (the last one taking what is left over).
 * @param {number} N      steps in the horizon
 * @param {number} S      segments (at most N)
 * @param {number} first  steps in the first segment, when there is more than one
 * @returns {number[]} the steps in each segment, adding up to N
 */
export function splitHorizon(N, S, first) {
  if (!(N >= 1)) throw new RangeError("splitHorizon: N must be at least 1");
  S = Math.max(1, Math.min(S, N));
  if (S === 1) return [N];
  const Na = Math.min(Math.max(Math.round(first), 1), N - (S - 1)), out = [Na];
  let left = N - Na;
  for (let s = 1; s < S; s++) {
    const n = s === S - 1 ? left : Math.max(1, Math.floor(left / (S - s)));
    out.push(n); left -= n;
  }
  return out;
}
