// Shapes to keep clear of, each with a signed distance: how far a point is outside it (negative inside). Their
// centres and sizes are plain properties, so an app can move or resize a shape while it is in use.

/**
 * @typedef {Object} Shape
 * @property {string} kind
 * @property {Float64Array} center
 * @property {(x: number, y: number, z: number) => number} sdf  the signed distance from (x, y, z) to the surface
 */

/**
 * A ball.
 * @param {ArrayLike<number>} center
 * @param {number} radius
 */
export function sphere(center, radius) {
  return {
    kind: "sphere", center: Float64Array.from(center), radius,
    sdf(x, y, z) {
      const c = this.center, dx = x - c[0], dy = y - c[1], dz = z - c[2];
      return Math.sqrt(dx * dx + dy * dy + dz * dz) - this.radius;
    },
  };
}

/**
 * A box with its sides along the axes.
 * @param {ArrayLike<number>} center
 * @param {ArrayLike<number>} half  half its size along each axis
 */
export function box(center, half) {
  return {
    kind: "box", center: Float64Array.from(center), half: Float64Array.from(half),
    sdf(x, y, z) {
      const c = this.center, h = this.half;
      const qx = Math.abs(x - c[0]) - h[0], qy = Math.abs(y - c[1]) - h[1], qz = Math.abs(z - c[2]) - h[2];
      const ox = qx > 0 ? qx : 0, oy = qy > 0 ? qy : 0, oz = qz > 0 ? qz : 0;
      const inside = Math.min(Math.max(qx, qy, qz), 0);
      return Math.sqrt(ox * ox + oy * oy + oz * oz) + inside;
    },
  };
}

/**
 * A cylinder with flat ends, its axis along x (0), y (1) or z (2).
 * @param {ArrayLike<number>} center      the middle of its axis
 * @param {number} radius
 * @param {number} halfLength             half its length
 * @param {0|1|2} [axis=2]
 */
export function cylinder(center, radius, halfLength, axis = 2) {
  const a = axis, b = (axis + 1) % 3, c2 = (axis + 2) % 3;
  return {
    kind: "cylinder", center: Float64Array.from(center), radius, halfLength, axis,
    sdf(x, y, z) {
      const c = this.center, d0 = x - c[0], d1 = y - c[1], d2 = z - c[2];
      const d = a === 0 ? d0 : a === 1 ? d1 : d2;
      const p = b === 0 ? d0 : b === 1 ? d1 : d2, q = c2 === 0 ? d0 : c2 === 1 ? d1 : d2;
      const qa = Math.sqrt(p * p + q * q) - this.radius, qb = Math.abs(d) - this.halfLength;
      const oa = qa > 0 ? qa : 0, ob = qb > 0 ? qb : 0;
      return Math.sqrt(oa * oa + ob * ob) + Math.min(Math.max(qa, qb), 0);
    },
  };
}
