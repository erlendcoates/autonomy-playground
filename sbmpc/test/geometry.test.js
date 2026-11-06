import {test} from "node:test";
import assert from "node:assert/strict";
import {Path3, shapes} from "../src/index.js";

const close = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) <= eps, `${a} is not ${b}`);

test("Path3: nearest point, a point along it and its direction", () => {
  const P = new Path3([[0, 0, 0], [10, 0, 0], [10, 10, 5]]);
  close(P.length, 10 + Math.hypot(10, 5));
  const q = P.project(4, 3, 0);
  close(q.s, 4); close(q.d2, 9); assert.equal(q.i, 0);
  close(P.distance2(4, 3, 0), 9);
  assert.deepEqual(P.pointAt(-5), [0, 0, 0]);                 // held at the ends of an open path
  assert.deepEqual(P.pointAt(1e6), [10, 10, 5]);
  const p = P.pointAt(10 + Math.hypot(10, 5) / 2);
  close(p[0], 10); close(p[1], 5); close(p[2], 2.5);
  const t = P.tangentAt(12);
  close(t[1], 10 / Math.hypot(10, 5)); close(t[2], 5 / Math.hypot(10, 5));
});

test("Path3: a closed path wraps round", () => {
  const P = new Path3([[0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0]], {closed: true});
  close(P.length, 40);
  assert.deepEqual(P.pointAt(45), [5, 0, 0]);
  assert.deepEqual(P.pointAt(-5), [0, 5, 0]);
  const A = P.ahead(-1, 7, 0, 12);                           // nearest (0, 7) on the last leg: on round the corner
  close(A.length, 12);
  assert.deepEqual(A.points, [[0, 7, 0], [0, 0, 0], [5, 0, 0]]);
  assert.equal(A.closed, false);
});

test("Path3: what is ahead at the end of an open path is a point", () => {
  const P = new Path3([[0, 0, 0], [10, 0, 0]]);
  const A = P.ahead(20, 3, 0, 5);
  close(A.length, 0);
  close(A.distance2(20, 3, 0), 100 + 9);
  assert.deepEqual(A.pointAt(3), [10, 0, 0]);
});

test("shapes: signed distances", () => {
  const s = shapes.sphere([1, 2, 3], 2);
  close(s.sdf(1, 2, 8), 3); close(s.sdf(1, 2, 3), -2);
  const b = shapes.box([0, 0, 0], [1, 2, 3]);
  close(b.sdf(4, 0, 0), 3); close(b.sdf(0, 0, 0), -1);
  close(b.sdf(4, 6, 0), 5);                                  // past a vertical edge: to the edge
  const c = shapes.cylinder([0, 0, -10], 1, 10, 2);           // a pillar standing on z = 0, its top at z = -20 (NED)
  close(c.sdf(4, 0, -5), 3); close(c.sdf(0, 0, -23), 3); close(c.sdf(0, 0, -10), -1);
  close(c.sdf(4, 0, -24), 5);                                // past its rim
  s.center[0] = 11;                                           // shapes can be moved in place
  close(s.sdf(11, 2, 8), 3);
});

test("Path3: the nearest leg", () => {
  const P = new Path3([[0, 0, 0], [10, 0, 0], [10, 10, 0]]);
  assert.equal(P.nearestLeg(4, -2, 0), P.segs[0]);
  assert.equal(P.nearestLeg(12, 7, 0), P.segs[1]);
  assert.equal(P.nearestLeg(4, -2, 0).tx, 1);
});
