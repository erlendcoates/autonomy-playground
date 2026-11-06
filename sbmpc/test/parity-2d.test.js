// The kernel, given the 2D app's model, plans, cost and constraints, scores every plan as the 2D app does. The 2D
// app's code below is copied from simulation-based-mpc/index.html (its scene and evaluate()), with only the drawing
// bookkeeping left out.
import {test} from "node:test";
import assert from "node:assert/strict";
import {Planner, Path3, lattice, splitHorizon, models, costs, constraints, shapes} from "../src/index.js";

// ── the 2D app ──────────────────────────────────────────────────────────────────────────────────────────────────────
const D2R = Math.PI / 180, DT = 0.5, R_KEEP = 2.2;
const LOOP = [[0, 0], [80, 0], [125, 45], [105, 115], [25, 125], [-20, 60]];
const KAY = {n: 50, e0: -30, e1: 30, v: 0.7, phase0: 13};
const SEG = LOOP.map((a, i) => {
  const b = LOOP[(i + 1) % LOOP.length], tn = b[0] - a[0], te = b[1] - a[1], L = Math.hypot(tn, te);
  return {an: a[0], ae: a[1], tn: tn / L, te: te / L, L};
});
function segsAhead(n, e, len) {
  let best = Infinity, si = 0, su = 0;
  SEG.forEach((s, i) => {
    const wn = n - s.an, we = e - s.ae;
    const u = Math.min(Math.max(wn * s.tn + we * s.te, 0), s.L);
    const d2 = (wn - u * s.tn) ** 2 + (we - u * s.te) ** 2;
    if (d2 < best) { best = d2; si = i; su = u; }
  });
  const out = [];
  for (let left = len; left > 0; si = (si + 1) % SEG.length, su = 0) {
    const s = SEG[si], L = Math.min(s.L - su, left);
    if (L > 1e-9) out.push({an: s.an + su * s.tn, ae: s.ae + su * s.te, tn: s.tn, te: s.te, L});
    left -= L;
  }
  return out;
}
function distTo(segs, n, e) {
  let best = Infinity;
  for (const s of segs) {
    const wn = n - s.an, we = e - s.ae;
    let u = wn * s.tn + we * s.te;
    u = u < 0 ? 0 : u > s.L ? s.L : u;
    const dn = wn - u * s.tn, de = we - u * s.te, d2 = dn * dn + de * de;
    if (d2 < best) best = d2;
  }
  return best;
}
const CYC = 2 * (KAY.e1 - KAY.e0);
function kayakAt(t, phase) {
  const s = (((phase + KAY.v * t) % CYC) + CYC) % CYC, half = CYC / 2;
  return s < half ? {n: KAY.n, e: KAY.e0 + s, vn: 0, ve: KAY.v} : {n: KAY.n, e: KAY.e1 - (s - half), vn: 0, ve: -KAY.v};
}
function buildPlans(P) {
  const N = Math.max(1, Math.round(P.T / DT)), n = P.n, S = Math.min(P.S, N);
  const R = Array.from({length: n}, (_, i) => -P.rmax + 2 * P.rmax * i / (n - 1));
  let segs;
  if (S === 1) segs = [N];
  else {
    const Na = Math.min(Math.max(Math.round(P.Ta / DT), 1), N - (S - 1));
    if (S === 2) segs = [Na, N - Na];
    else { const nb = Math.max(1, Math.floor((N - Na) / 2)); segs = [Na, nb, N - Na - nb]; }
  }
  const segOf = new Int32Array(N);
  for (let k = 0, s = 0, acc = segs[0]; k < N; k++) { while (k >= acc) { s++; acc += segs[s]; } segOf[k] = s; }
  const count = n ** S, idx = [];
  for (let p = 0; p < count; p++) {
    const ii = new Array(S);
    for (let s = S - 1, q = p; s >= 0; s--) { ii[s] = q % n; q = Math.floor(q / n); }
    idx.push(ii);
  }
  return {N, n, S, R, segs, segOf, count, idx, J: new Float64Array(count), Jp: new Float64Array(count),
          Jt: new Float64Array(count), hit: new Int32Array(count), what: new Int8Array(count), best: -1};
}
const rateOf = (F, p, k) => F.R[F.idx[p][F.segOf[k]]];
function evaluate(F, P, Sx, n0, e0, psi0, t0) {
  const N = F.N, U = P.U, lam = P.lam, d2safe = P.dsafe ** 2, r2keep = R_KEEP ** 2;
  const kay = P.kayak ? kayakAt(t0, Sx.phase) : null;
  const ahead = segsAhead(n0, e0, U * N * DT + 30);
  let best = -1, bestJ = Infinity;
  for (let p = 0; p < F.count; p++) {
    let n = n0, e = e0, psi = psi0, Jp = 0, Jt = 0, hit = -1, what = 0;
    for (let k = 0; k < N; k++) {
      const r = rateOf(F, p, k);
      n += DT * U * Math.cos(psi); e += DT * U * Math.sin(psi); psi += DT * r * D2R;
      if (hit >= 0) continue;
      if (kay) {
        const s = (k + 1) * DT, dn = n - (kay.n + kay.vn * s), de = e - (kay.e + kay.ve * s);
        if (dn * dn + de * de < d2safe) { hit = k; what = 1; continue; }
      }
      for (const b of Sx.buoys) if ((n - b.n) ** 2 + (e - b.e) ** 2 < r2keep) { hit = k; what = 2; break; }
      if (hit >= 0) continue;
      Jp += distTo(ahead, n, e); Jt += lam * r * r;
    }
    F.Jp[p] = Jp; F.Jt[p] = Jt; F.hit[p] = hit; F.what[p] = what;
    F.J[p] = hit >= 0 ? Infinity : Jp + Jt;
    if (F.J[p] < bestJ) { bestJ = F.J[p]; best = p; }
  }
  F.best = best;
  return kay;
}

// ── the same with the kernel ────────────────────────────────────────────────────────────────────────────────────────
function kernel(P, Sx, kay) {
  const N = Math.max(1, Math.round(P.T / DT));
  const path = new Path3(LOOP.map(([n, e]) => [n, e, 0]), {closed: true});
  const model = models.unicycle({speed: P.U});
  const K = [];
  if (kay) K.push(constraints.keepOut({shape: shapes.sphere([kay.n, kay.e, 0], 0), margin: P.dsafe, velocity: [kay.vn, kay.ve, 0], name: "kayak"}));
  for (const b of Sx.buoys) K.push(constraints.keepOut({shape: shapes.sphere([b.n, b.e, 0], 0), margin: R_KEEP, name: "buoy"}));
  return new Planner({
    model, dt: DT, segments: splitHorizon(N, P.S, P.Ta / DT), controls: lattice([P.n], [P.rmax * D2R]),
    costs: [costs.pathDistance({path, ahead: P.U * N * DT + 30}), costs.effort({weight: P.lam / (D2R * D2R)})],
    constraints: K,
  });
}

const VIDEO = {T: 15, Ta: 4, S: 2, n: 9, lam: 0.05, rmax: 8, kayak: true, dsafe: 8, U: 1.5};
const CASES = [
  {name: "the video's settings, at the start", P: VIDEO, pose: [10, -10, 0], t: 0, buoys: []},
  {name: "the kayak just ahead", P: VIDEO, pose: [36, 1, 0.1], t: 8, buoys: []},
  {name: "a buoy on the path", P: VIDEO, pose: [60, 2, -0.2], t: 40, buoys: [{n: 72, e: 0}]},
  {name: "three segments, 5 rates, a long horizon", P: {...VIDEO, S: 3, n: 5, T: 25, Ta: 3, lam: 0.4}, pose: [30, -3, 0.3], t: 15, buoys: [{n: 45, e: 1}]},
  {name: "one segment, round a corner", P: {...VIDEO, S: 1, n: 17, kayak: false}, pose: [78, 4, 0.6], t: 0, buoys: []},
];

for (const c of CASES) {
  test(`parity with the 2D app: ${c.name}`, () => {
    const Sx = {phase: KAY.phase0, buoys: c.buoys}, F = buildPlans(c.P);
    const kay = evaluate(F, c.P, Sx, c.pose[0], c.pose[1], c.pose[2], c.t);
    const planner = kernel(c.P, Sx, kay).plan([c.pose[0], c.pose[1], 0, c.pose[2]], c.t);
    assert.equal(planner.count, F.count);
    assert.deepEqual(planner.segments, F.segs);
    let dropped = 0;
    for (let p = 0; p < F.count; p++) {
      assert.equal(planner.hit[p], F.hit[p], `plan ${p}: the step it breaks a constraint`);
      if (F.hit[p] >= 0) {
        dropped++;
        assert.equal(planner.what[p] === 0 && kay ? 1 : 2, F.what[p], `plan ${p}: what it breaks`);
        continue;
      }
      const rel = (a, b) => Math.abs(a - b) / Math.max(1, Math.abs(b));
      assert.ok(rel(planner.J[p], F.J[p]) < 1e-9, `plan ${p}: J ${planner.J[p]} vs ${F.J[p]}`);
      assert.ok(rel(planner.terms[2 * p], F.Jp[p]) < 1e-9, `plan ${p}: the path term`);
      assert.ok(rel(planner.terms[2 * p + 1], F.Jt[p]) < 1e-9, `plan ${p}: the turning term`);
    }
    assert.equal(planner.best, F.best);
    if (c.name.includes("kayak") || c.name.includes("buoy")) assert.ok(dropped > 0, "some plans are dropped");
  });
}
