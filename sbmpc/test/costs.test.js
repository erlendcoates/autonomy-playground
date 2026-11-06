import {test} from "node:test";
import assert from "node:assert/strict";
import {Path3, models, costs} from "../src/index.js";

const ctx = (x0, N = 4, dt = 0.5) => ({x0: Float64Array.from(x0), t0: 0, dt, N, model: models.pointMass3(), pos: 0});

test("trackPath: a reference point that moves along the path at the speed", () => {
  const path = new Path3([[0, 0, 0], [100, 0, 0]]), c = costs.trackPath({path, speed: 2, weight: 3});
  c.prepare(ctx([1, 3, 0, 0, 0, 0]));
  assert.deepEqual(Array.from(c.ref), [2, 0, 0, 3, 0, 0, 4, 0, 0, 5, 0, 0]);   // from s = 1, then 1 m a step
  assert.equal(c.stage(0, [2, 3, 0]), 3 * 9);
  assert.equal(c.stage(3, [5, 0, -1]), 3 * 1);
});

test("pathVelocity: the velocity against the speed along the path ahead", () => {
  const path = new Path3([[0, 0, 0], [10, 0, 0], [10, 10, 0]]), c = costs.pathVelocity({path, speed: 2, ahead: 30, weight: 2});
  c.prepare(ctx([1, 0, 0, 2, 0, 0]));
  assert.equal(c.stage(0, [5, 0, 0, 1, 1, 0]), 2 * ((1 - 2) ** 2 + 1));        // on the first leg: along x
  assert.equal(c.stage(0, [11, 6, 0, 0, 2, 0]), 0);                           // on the second: along y
});

test("pathDistance and effort", () => {
  const path = new Path3([[0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0]], {closed: true});
  const d = costs.pathDistance({path, ahead: 15});
  d.prepare(ctx([8, -1, 0, 0, 0, 0]));
  assert.equal(d.stage(0, [8, 2, 0]), 4);
  // 15 m ahead of (8, 0) ends at (7, 10): the rest of the loop, nearer (-3, 10), is not ahead
  assert.equal(d.stage(0, [-3, 10, 0]), 100);
  const e = costs.effort({weight: 0.5});
  assert.equal(e.stage(0, [], [1, 2, 2]), 4.5);
});
