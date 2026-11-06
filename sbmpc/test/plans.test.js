import {test} from "node:test";
import assert from "node:assert/strict";
import {linspace, lattice, zeros, splitHorizon} from "../src/index.js";

test("linspace: ends included, one value is the midpoint", () => {
  assert.deepEqual(linspace(-2, 2, 5), [-2, -1, 0, 1, 2]);
  assert.deepEqual(linspace(-3, 1, 1), [-1]);
  assert.throws(() => linspace(0, 1, 0), RangeError);
});

test("lattice: every combination, the first axis the most significant", () => {
  const L = lattice([2, 3], [1, 2]);
  assert.equal(L.length, 2 * 3 * 2);
  assert.deepEqual(Array.from(L), [-1, -2, -1, 0, -1, 2, 1, -2, 1, 0, 1, 2]);
  const A = lattice([3, 3, 3], [2, 2, 1]);
  assert.equal(A.length / 3, 27);
  const j = (2 * 3 + 0) * 3 + 1;                         // indices (2, 0, 1)
  assert.deepEqual(Array.from(A.subarray(3 * j, 3 * j + 3)), [2, -2, 0]);
  assert.deepEqual(Array.from(lattice([1, 3], [5, 1], [3, -1])), [4, -1, 4, 0, 4, 1]);
  assert.deepEqual(Array.from(zeros(3)), [0, 0, 0]);
});

test("splitHorizon: as the 2D app splits it", () => {
  assert.deepEqual(splitHorizon(30, 1, 8), [30]);
  assert.deepEqual(splitHorizon(30, 2, 8), [8, 22]);
  assert.deepEqual(splitHorizon(30, 3, 8), [8, 11, 11]);
  assert.deepEqual(splitHorizon(31, 3, 8), [8, 11, 12]);
  assert.deepEqual(splitHorizon(4, 3, 9), [2, 1, 1]);           // the first segment leaves room for the others
  assert.deepEqual(splitHorizon(2, 3, 1), [1, 1]);              // never more segments than steps
});
