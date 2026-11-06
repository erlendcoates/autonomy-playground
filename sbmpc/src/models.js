// Prediction models. A model advances a state `x` by `dt` under a control `u`, in place, and says where the position
// (three numbers) is in the state. The planner calls `step` for every step of every plan, so it must not allocate.

/**
 * @typedef {Object} Model
 * @property {string} name
 * @property {number} nx   the state's size
 * @property {number} nu   the control's size
 * @property {number} pos  where the position [x, y, z] starts in the state
 * @property {(x: Float64Array, u: ArrayLike<number>, dt: number) => void} step  advance x by dt under u, in place
 */

/**
 * A point mass driven by its acceleration: a multirotor seen from above its attitude loop, the acceleration command
 * being what the plans choose. State [x, y, z, vx, vy, vz], control [ax, ay, az]. Exact for an acceleration held over
 * the step (zero-order hold): p += dt v + dt²/2 a, v += dt a.
 * @returns {Model & {vel: number}}
 */
export function pointMass3() {
  return {
    name: "point mass", nx: 6, nu: 3, pos: 0, vel: 3,
    step(x, u, dt) {
      const h = 0.5 * dt * dt;
      x[0] += dt * x[3] + h * u[0]; x[1] += dt * x[4] + h * u[1]; x[2] += dt * x[5] + h * u[2];
      x[3] += dt * u[0]; x[4] += dt * u[1]; x[5] += dt * u[2];
    },
  };
}

/**
 * The kinematic vessel of the 2D app, at a constant speed in the horizontal plane: state [x, y, z, ψ] with ψ the
 * heading from the x axis towards y, control [r] the turn rate (rad/s). Forward Euler, as in the video:
 * x += dt U cos ψ, y += dt U sin ψ, ψ += dt r. `speed` may be changed between plans.
 * @param {{speed: number}} options
 * @returns {Model & {speed: number}}
 */
export function unicycle({speed}) {
  return {
    name: "unicycle", nx: 4, nu: 1, pos: 0, speed,
    step(x, u, dt) {
      const U = this.speed, psi = x[3];
      x[0] += dt * U * Math.cos(psi); x[1] += dt * U * Math.sin(psi); x[3] += dt * u[0];
    },
  };
}
