import {test} from "node:test";
import assert from "node:assert/strict";
import {Planner, Path3, lattice, zeros, models, costs, constraints, shapes} from "../src/index.js";

// A scene with every kind of constraint, some plans dropped early, some late.
function scene() {
  const path = new Path3([[0, 0, -5], [30, 0, -5], [40, 20, -10]]);
  const model = models.pointMass3();
  const set = lattice([3, 3, 3], [2, 2, 1.5]);
  const planner = new Planner({
    model, dt: 0.25, segments: [4, 6, 6], controls: [set, set, zeros(3)],
    costs: [costs.trackPath({path, speed: 4}), costs.effort({weight: 0.3})],
    constraints: [
      constraints.keepOut({shape: shapes.sphere([14, 6, -5], 0), margin: 4, velocity: [0, -2.5, 0], name: "intruder"}),
      constraints.halfSpace({normal: [0, 0, 1], offset: -1, name: "ground"}),
      constraints.maxNorm({index: 3, max: 6.5, name: "speed"}),
      constraints.keepOut({shape: shapes.box([12, -4, -5], [1, 1, 5]), margin: 1, name: "building"}),
      constraints.keepOut({shape: shapes.cylinder([20, 2, -10], 1, 10), margin: 1, name: "pillar"}),
    ],
  });
  return {planner, x0: [0, 1, -4, 4, 0, 0.5]};
}

// The same, the obvious way: every plan predicted on its own from the start.
function bruteForce(planner, x0, t0) {
  const {N, count, dt, model, costs: C, constraints: K} = planner;
  const ctx = {x0: Float64Array.from(x0), t0, dt, N, model, pos: model.pos};
  for (const c of [...C, ...K]) c.prepare?.(ctx);
  const out = [];
  for (let p = 0; p < count; p++) {
    const x = Float64Array.from(x0), terms = C.map(() => 0);
    let hit = -1, what = -1;
    for (let k = 0; k < N && hit < 0; k++) {
      const u = planner.control(p, k);
      model.step(x, u, dt);
      what = K.findIndex((c) => c.violated(k, x, u));
      if (what >= 0) { hit = k; break; }
      C.forEach((c, j) => { terms[j] += c.stage(k, x, u); });
    }
    out.push({J: hit >= 0 ? Infinity : terms.reduce((a, b) => a + b, 0), hit, what, terms});
  }
  return out;
}

test("the tree of plans gives what predicting each plan on its own gives", () => {
  const {planner, x0} = scene();
  planner.plan(x0, 3);
  const ref = bruteForce(planner, x0, 3);
  assert.equal(planner.count, 27 * 27);
  let dropped = 0;
  const kinds = new Set();
  for (let p = 0; p < planner.count; p++) {
    const r = ref[p];
    assert.equal(planner.hit[p], r.hit, `plan ${p}: hit`);
    assert.equal(planner.what[p], r.what, `plan ${p}: what`);
    if (r.hit < 0) {
      assert.ok(Math.abs(planner.J[p] - r.J) <= 1e-9 * Math.max(1, r.J), `plan ${p}: J ${planner.J[p]} vs ${r.J}`);
      r.terms.forEach((v, j) => assert.ok(Math.abs(planner.terms[p * 2 + j] - v) <= 1e-9 * Math.max(1, v)));
    } else { dropped++; kinds.add(r.what); assert.equal(planner.J[p], Infinity); }
  }
  assert.ok(dropped > 0 && dropped < planner.count, "the scene drops some plans and keeps some");
  assert.ok(kinds.size >= 3, `the scene breaks several constraints (${[...kinds]})`);
  assert.ok(planner.steps < planner.count * planner.N, "shared prefixes are predicted once");
});

test("the cheapest plan, the ranks and the order", () => {
  const {planner, x0} = scene();
  planner.plan(x0, 0);
  const kept = [...Array(planner.count).keys()].filter((p) => planner.hit[p] < 0);
  const cheapest = kept.reduce((a, b) => (planner.J[b] < planner.J[a] ? b : a));
  assert.equal(planner.best, cheapest);
  assert.equal(planner.feasible, kept.length);
  assert.equal(planner.rank[planner.best], 0);
  const sorted = Array.from(planner.sorted.subarray(0, planner.feasible));
  for (let i = 1; i < sorted.length; i++) assert.ok(planner.J[sorted[i - 1]] <= planner.J[sorted[i]]);
  assert.equal(planner.rank[sorted.at(-1)], 1);
});

test("the stored positions are the model's prediction, up to where a plan breaks a constraint", () => {
  const {planner, x0} = scene();
  planner.plan(x0, 0);
  const {N, nx} = planner, stride = 3 * (N + 1);
  for (const p of [planner.best, 0, 100, 400, planner.count - 1]) {
    const X = planner.rollout(p), last = planner.hit[p] >= 0 ? planner.hit[p] + 1 : N;
    for (let k = 0; k <= N; k++) {
      const kk = Math.min(k, last);
      for (let i = 0; i < 3; i++) {
        const a = planner.traj[p * stride + 3 * k + i], b = X[kk * nx + i];
        assert.ok(Math.abs(a - b) < 1e-4 * Math.max(1, Math.abs(b)), `plan ${p} step ${k}: ${a} vs ${b}`);
      }
    }
  }
});

test("digits, planOf and control agree", () => {
  const {planner} = scene();
  const p = planner.planOf([5, 20, 0]);
  assert.deepEqual(planner.digits(p), [5, 20, 0]);
  assert.deepEqual(Array.from(planner.control(p, 0)), Array.from(planner.views[0][5]));
  assert.deepEqual(Array.from(planner.control(p, 4)), Array.from(planner.views[1][20]));
  assert.deepEqual(Array.from(planner.control(p, 15)), [0, 0, 0]);
});

test("when every plan is dropped: no best, and the fallback breaks a constraint latest", () => {
  const model = models.pointMass3();
  const planner = new Planner({
    model, dt: 0.5, segments: [2, 4], controls: lattice([3, 1, 1], [1, 0, 0]),
    // a wall thicker than a step (constraints are checked at the steps only: a thin one could be stepped over)
    constraints: [constraints.keepOut({shape: shapes.box([6, 0, 0], [3, 10, 10]), margin: 0})],
  });
  planner.plan([-4, 0, 0, 4, 0, 0]);
  assert.equal(planner.best, -1);
  assert.equal(planner.feasible, 0);
  assert.equal(planner.hit[planner.fallback], Math.max(...planner.hit));
  assert.equal(planner.hit[planner.fallback], 5);
  assert.deepEqual(planner.digits(planner.fallback), [0, 0]);   // braking all the way reaches the wall last
});

test("cost weights and constraint settings can change between plans", () => {
  const {planner, x0} = scene();
  planner.plan(x0);
  const before = planner.feasible;
  planner.constraints[2].max = 100;                            // lift the speed limit
  planner.plan(x0);
  assert.ok(planner.feasible > before);
  const J0 = planner.J[planner.best];
  planner.costs[1].weight = 0;                                 // effort free
  planner.plan(x0);
  assert.ok(planner.J[planner.best] <= J0);
});

test("configure: refuses what cannot be planned", () => {
  const model = models.pointMass3();
  assert.throws(() => new Planner({model, dt: 0.1, segments: [3], controls: new Float64Array(4)}), RangeError);
  assert.throws(() => new Planner({model, dt: 0, segments: [3], controls: zeros(3)}), RangeError);
  assert.throws(() => new Planner({model, dt: 0.1, segments: [3, 0], controls: zeros(3)}), RangeError);
  assert.throws(() => new Planner({model, dt: 0.1, segments: [1, 1, 1, 1], controls: lattice([5, 5, 5], [1, 1, 1]), maxPlans: 1e6}), RangeError);
});
