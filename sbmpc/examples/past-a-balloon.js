// A quadrotor flies a path past a balloon, by simulation-based MPC, with the model itself standing in for the drone.
//   node examples/past-a-balloon.js
import {Planner, Path3, lattice, zeros, models, costs, constraints, shapes} from "../src/index.js";

const path = new Path3([[0, 0, -10], [60, 0, -10], [60, 40, -15]]);      // north, east, down (m)
const balloon = shapes.sphere([30, 0, -10], 2);                           // right on the path
const set = lattice([3, 3, 3], [2.5, 2.5, 2.5]);                          // 27 accelerations (m/s²)

const planner = new Planner({
  model: models.pointMass3(),                                             // state [p, v], control a
  dt: 0.25,
  segments: [6, 6, 8],                                                    // 1.5 s, 1.5 s, then 2 s…
  controls: [set, set, zeros(3)],                                         // …of coasting: 27 × 27 × 1 = 729 plans
  costs: [costs.trackPath({path, speed: 4}), costs.effort({weight: 0.2})],
  constraints: [
    constraints.keepOut({shape: balloon, margin: 1.5, name: "the balloon"}),
    constraints.halfSpace({normal: [0, 0, 1], offset: -1, name: "the ground"}),   // altitude at least 1 m
    constraints.maxNorm({index: 3, max: 8, name: "the speed limit"}),
  ],
});

const x = Float64Array.from([0, 0, -10, 4, 0, 0]);
let closest = Infinity;
for (let t = 0; t < 25; t += planner.dt) {
  planner.plan(x, t);
  const p = planner.best >= 0 ? planner.best : planner.fallback;           // none kept: the one that breaks latest
  planner.model.step(x, planner.control(p, 0), planner.dt);               // carry out its first step, then plan again
  closest = Math.min(closest, balloon.sdf(x[0], x[1], x[2]));
}
console.log(`${planner.count} plans, ${planner.steps} model steps and ${planner.ms.toFixed(2)} ms a replan`);
console.log(`ended at north ${x[0].toFixed(1)}, east ${x[1].toFixed(1)}, altitude ${(-x[2]).toFixed(1)} m`);
console.log(`closest to the balloon's surface: ${closest.toFixed(2)} m (kept 1.5 m)`);
