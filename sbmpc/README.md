# sbmpc

Simulation-based model predictive control (MPC) in 3D, for any model. A small JavaScript library with no
dependencies that runs in a browser or Node. It is the kernel behind
[Simulation-Based MPC in 3D](../simulation-based-mpc-3d/). Given the 2D app's own model, plans, cost and constraints,
it scores every plan as [Simulation-Based MPC](../simulation-based-mpc/) does.

Simulation-based MPC predicts every plan in a finite set with a model and scores each one by a cost. It drops the
plans that break a constraint and keeps the cheapest. The first control of that plan is carried out for one step,
then the whole set is predicted again from where the vehicle really is.

## The idea, in the kernel's terms

- **A model** advances a state `x` by `dt` under a control `u`, in place, and says where the position (three
  numbers) is in the state. Planar models keep the third coordinate constant.
- **A plan** holds a control constant over each of S segments of the horizon. Segment s picks one control from its
  **control set**, so there are |set 0| × |set 1| × … plans. A set of a single zero control makes a segment that
  coasts.
- **Cost terms** are added up over the steps. Each term's `stage(k, x, u)` sees the state after step k and the
  control held during it.
- **Constraints** drop a plan at the first step whose state breaks one. Limits on the controls themselves belong in
  the control sets.
- **Plans that start the same way share that part of the prediction.** The planner walks the plans as a tree: each
  first segment is predicted once, each second segment once for every first, and so on. When a prefix breaks a
  constraint, every plan under it is dropped at once. For two segments of 27 accelerations and a coast (729 plans,
  20 steps) that is about 5 000 model steps a replan instead of 14 580.

## Example

```js
import {Planner, Path3, lattice, zeros, models, costs, constraints, shapes} from "sbmpc";

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
for (let t = 0; t < 25; t += planner.dt) {
  planner.plan(x, t);
  const p = planner.best >= 0 ? planner.best : planner.fallback;           // none kept: the one that breaks latest
  planner.model.step(x, planner.control(p, 0), planner.dt);               // carry out its first step, then plan again
}
```

This is [`examples/past-a-balloon.js`](examples/past-a-balloon.js), with the model standing in for the drone: `node
examples/past-a-balloon.js`. Inside this repository, import from `../sbmpc/src/index.js` instead of `"sbmpc"`.

## API

### `new Planner(options)`

| option | |
|---|---|
| `model` | predicts the plans (see Models) |
| `dt` | one step of the horizon (s) |
| `segments` | the steps in each segment; they add up to the horizon N |
| `controls` | the controls a segment may pick from, `nu` numbers each, in a `Float64Array`: one set for every segment, or an array of a set per segment |
| `costs` | cost terms, added up into a plan's cost |
| `constraints` | a plan that breaks one is dropped |
| `maxPlans` | refuse sets larger than this (default 2 000 000) |

`planner.configure(options)` changes any of them and reallocates what it must. The `costs` and `constraints` arrays,
and the settings on the terms (weights, margins, speeds), may also be changed in place between calls to `plan`.

`planner.plan(x0, t0)` predicts, scores and checks every plan from the state `x0` at time `t0`, and returns the
planner, with:

| | one entry per plan `p` |
|---|---|
| `J[p]` | its cost (`Infinity` if dropped) |
| `terms[p * costs.length + j]` | cost term j (for a dropped plan: up to where it broke a constraint) |
| `hit[p]` | the step at which it broke a constraint, or −1 |
| `what[p]` | which constraint (an index into `constraints`), or −1 |
| `traj` | the predicted positions, `3 (N + 1)` numbers per plan (`Float32Array`); after a break, the point where it broke, repeated |
| `sorted` | the plans kept, cheapest first (the first `feasible` entries) |
| `rank[p]` | its place among those kept, 0 the cheapest to 1 the dearest (1 if dropped) |

It also sets `best` (the cheapest plan kept, or −1), `fallback` (when none is kept, the plan that breaks a constraint
latest; the cheaper one of a tie), `feasible` (how many are kept), `steps` (model steps taken) and `ms` (time taken).
The arrays are reused by the next call.

`planner.control(p, k)` is the control plan `p` holds during step `k`. `planner.digits(p)` is its choice in each
segment, and `planner.planOf(digits)` is the inverse. `planner.rollout(p)` predicts plan `p` again and returns every
state, `(N + 1) × nx` numbers. Plan p's choice in segment s is `Math.floor(p / span[s]) % sizes[s]`, with segment 0
the most significant.

### Models (`models`)

- `pointMass3()`: state `[x, y, z, vx, vy, vz]`, control the acceleration `[ax, ay, az]`, held exactly over a step.
  A multirotor seen from above its attitude loop.
- `unicycle({speed})`: the 2D app's vessel at a constant speed. State `[x, y, z, ψ]`, control the turn rate `[r]`
  (rad/s), forward Euler.

Your own model is an object `{nx, nu, pos, step(x, u, dt)}`, where `step` advances `x` in place. It runs for every
step of every plan, so it must not allocate.

### Cost terms (`costs`)

- `trackPath({path, speed, weight})`: w Σ |p_k − r_k|². The reference point r sets off from the point on the path
  nearest the start and moves along it at `speed`, so the term asks for both the path and the speed on it.
- `pathDistance({path, ahead, weight})`: w Σ d_k², the squared distance from the path ahead of the start (as in the
  2D app). `ahead` is a length in metres, or a function of the context.
- `pathVelocity({path, speed, ahead, weight})`: w Σ |v_k − U t_k|², the velocity against `speed` along the direction
  of the path ahead.
- `effort({weight})`: w Σ |u_k|².

Your own term is an object `{name, stage(k, x, u), prepare?(ctx), terminal?(x)}`. `prepare` gets
`{x0, t0, dt, N, model, pos}` once per `plan`, to work out what every plan shares.

### Constraints (`constraints`)

- `keepOut({shape, margin, velocity})`: keep the position at least `margin` from the surface of a shape. With a
  velocity, the shape is predicted straight on from where it is now.
- `keepAway({predict, distance})`: keep the position at least `distance` from `predict(t)`, for something whose
  future positions you know.
- `halfSpace({normal, offset})`: keep `normal · p ≤ offset` (a floor or a ceiling).
- `maxNorm({index, size, max})`: keep the length of `size` numbers of the state, from `index` on, at most `max` (a
  speed limit).

Your own constraint is an object `{name, violated(k, x, u), prepare?(ctx)}`.

### Geometry

- `new Path3(points, {closed})`: straight legs between 3D points. `project`, `distance2`, `nearestLeg`, `pointAt`,
  `tangentAt`, `ahead`.
- `shapes.sphere(center, radius)`, `shapes.box(center, half)` (sides along the axes) and
  `shapes.cylinder(center, radius, halfLength, axis)`, each with a signed distance `sdf(x, y, z)`. Their `center`
  and sizes may be changed while in use.
- `lattice(levels, max, min)`, every combination of evenly spaced values on each axis; `zeros(nu)`;
  `linspace(a, b, n)`; and `splitHorizon(N, S, first)`, the 2D app's split of a horizon into segments.

## Good to know

- **Constraints are checked at the steps only.** A plan moving further in one step than an obstacle is thick can
  step over it. Keep margins, or obstacles, larger than the distance moved in a step.
- **The number of plans is the product of the set sizes.** It grows as a power of the segments: 3 segments of 27
  is 19 683 plans. Trying every plan only works when there are relatively few, which is why other kinds of MPC use
  numerical optimization.
- **Coarse control sets quantize the controls.** A plan family whose only choices are large overshoots small errors,
  so the cheapest plan may leave them alone. A short first segment followed by long ones can also make the plans
  put corrections off ("accelerate later"), again and again. With acceleration plans, segments of equal length, a
  finer lattice, or a smaller `dt` help.
- **The kernel does not mind which frame or units it is given.** The 3D app uses north-east-down.

## Tests

```
node --test
```

The tests check the tree of plans against predicting each plan on its own. They also check the kernel against the
2D app's `evaluate()`, copied into the test, plan by plan.

## License

MIT, see `LICENSE` in the repository root.
