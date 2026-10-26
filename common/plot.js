// A small canvas plotter for the apps: line charts, scatter, histograms and markers, crisp on high-DPI screens and
// coloured from the page's CSS variables (so light and dark themes both work).
//
//   const p = new Plot(canvas, {xLabel: "time (s)", yLabel: "u (m/s)"});
//   p.draw({series: [{x, y, color: "--los", label: "estimate", width: 1.5, dash: [4, 3]}],
//           points: [{x, y, color: "--cb", r: 2}], vlines: [{x: 12, color: "--muted", label: "cut"}],
//           hlines: [{y: 80.85, color: "--ok", label: "true"}], bars: {edges, counts, color},
//           xRange: [0, 60], yRange: null /* auto */});
// Colours are CSS colours or "--name" for a variable on :root.

export function css(name) {
  if (!name || !name.startsWith("--")) return name;
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || "#888";
}

export function niceTicks(lo, hi, n = 5) {
  if (!(hi > lo)) return [lo];
  const span = hi - lo, step0 = span / n, mag = 10 ** Math.floor(Math.log10(step0));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => span / s <= n + 0.5) || 10 * mag;
  const out = [];
  for (let v = Math.ceil(lo / step - 1e-9) * step; v <= hi + 1e-9 * span; v += step) out.push(Math.abs(v) < 1e-12 * span ? 0 : v);
  return out;
}

function fmt(v, ticks) {
  const step = ticks.length > 1 ? Math.abs(ticks[1] - ticks[0]) : Math.abs(v) || 1;
  let d = 0;                                   // as many decimals as the step needs (2.5 needs one)
  while (d < 6 && Math.abs(step * 10 ** d - Math.round(step * 10 ** d)) > 1e-6 * step * 10 ** d) d++;
  return v.toFixed(d).replace("-", "−");
}

export class Plot {
  constructor(canvas, opts = {}) {
    this.c = canvas;
    this.o = {xLabel: "", yLabel: "", legend: true, ...opts, pad: {l: 52, r: 12, t: 10, b: 34, ...(opts.pad || {})}};
    this.last = null;
    new ResizeObserver(() => this.last && this.draw(this.last)).observe(canvas);
  }

  // Data -> pixel transforms of the last draw (for mouse interaction).
  toPx(x, y) { const t = this.t; return [t.x0 + (x - t.xa) * t.sx, t.y0 - (y - t.ya) * t.sy]; }
  fromPx(px, py) { const t = this.t; return [t.xa + (px - t.x0) / t.sx, t.ya + (t.y0 - py) / t.sy]; }

  draw(spec) {
    this.last = spec;
    const c = this.c, dpr = window.devicePixelRatio || 1;
    const W = c.clientWidth, Hh = c.clientHeight;
    if (!W || !Hh) return;
    if (c.width !== Math.round(W * dpr) || c.height !== Math.round(Hh * dpr)) {
      c.width = Math.round(W * dpr); c.height = Math.round(Hh * dpr);
    }
    const g = c.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, Hh);
    const pad = {...this.o.pad, ...(spec.pad || {})};
    const font = css("--sans") || "sans-serif", mono = css("--mono") || "monospace";

    // ranges
    let [xa, xb] = spec.xRange || [Infinity, -Infinity];
    let [ya, yb] = spec.yRange || [Infinity, -Infinity];
    const grow = (xs, ys) => {
      if (!spec.xRange && xs) for (const v of xs) if (Number.isFinite(v)) { if (v < xa) xa = v; if (v > xb) xb = v; }
      if (!spec.yRange && ys) for (const v of ys) if (Number.isFinite(v)) { if (v < ya) ya = v; if (v > yb) yb = v; }
    };
    for (const s of spec.series || []) grow(s.x, s.y);
    for (const s of spec.points || []) grow(s.x, s.y);
    for (const h of spec.hlines || []) if (h.inRange !== false) grow(null, [h.y]);
    if (spec.bars) { grow(spec.bars.edges, [0, ...spec.bars.counts]); }
    if (!Number.isFinite(xa)) { xa = 0; xb = 1; }
    if (!Number.isFinite(ya)) { ya = 0; yb = 1; }
    if (xb - xa < 1e-12) { xa -= 0.5; xb += 0.5; }
    if (yb - ya < 1e-12) { ya -= 0.5; yb += 0.5; }
    if (!spec.yRange) { const m = 0.06 * (yb - ya); ya -= m; yb += m; }

    const x0 = pad.l, x1 = W - pad.r, y0 = Hh - pad.b, y1 = pad.t;
    this.t = {xa, ya, x0, y0, sx: (x1 - x0) / (xb - xa), sy: (y0 - y1) / (yb - ya)};
    const P = (x, y) => this.toPx(x, y);

    // grid and axes
    g.font = `11px ${mono}`;
    g.lineWidth = 1;
    const xt = niceTicks(xa, xb, Math.max(2, Math.floor((x1 - x0) / 80)));
    const yt = niceTicks(ya, yb, Math.max(2, Math.floor((y0 - y1) / 40)));
    g.strokeStyle = css("--grid"); g.fillStyle = css("--muted");
    g.textAlign = "center"; g.textBaseline = "top";
    for (const v of xt) {
      const [px] = P(v, ya);
      g.beginPath(); g.moveTo(px + 0.5, y1); g.lineTo(px + 0.5, y0); g.stroke();
      g.fillText(fmt(v, xt), px, y0 + 4);
    }
    g.textAlign = "right"; g.textBaseline = "middle";
    for (const v of yt) {
      const [, py] = P(xa, v);
      g.beginPath(); g.moveTo(x0, py + 0.5); g.lineTo(x1, py + 0.5); g.stroke();
      g.fillText(fmt(v, yt), x0 - 5, py);
    }
    g.strokeStyle = css("--line");
    g.strokeRect(x0 + 0.5, y1 + 0.5, x1 - x0 - 1, y0 - y1 - 1);
    g.font = `12px ${font}`; g.fillStyle = css("--muted");
    if (this.o.xLabel) { g.textAlign = "center"; g.textBaseline = "bottom"; g.fillText(this.o.xLabel, (x0 + x1) / 2, Hh - 2); }
    if (this.o.yLabel) {
      g.save(); g.translate(12, (y0 + y1) / 2); g.rotate(-Math.PI / 2);
      g.textAlign = "center"; g.textBaseline = "middle"; g.fillText(this.o.yLabel, 0, 0); g.restore();
    }

    g.save();
    g.beginPath(); g.rect(x0, y1, x1 - x0, y0 - y1); g.clip();
    if (spec.bands) for (const b of spec.bands) {
      const [pa] = P(b.x0, 0), [pb] = P(b.x1, 0);
      g.fillStyle = css(b.color || "--grid"); g.globalAlpha = b.alpha ?? 0.5;
      g.fillRect(pa, y1, pb - pa, y0 - y1); g.globalAlpha = 1;
    }
    if (spec.bars) {
      const {edges, counts, color, alpha = 0.55} = spec.bars;
      g.fillStyle = css(color || "--accent"); g.globalAlpha = alpha;
      for (let i = 0; i < counts.length; i++) {
        const [pa, pt] = P(edges[i], counts[i]), [pb, pz] = P(edges[i + 1], 0);
        g.fillRect(pa + 0.5, pt, Math.max(1, pb - pa - 1), pz - pt);
      }
      g.globalAlpha = 1;
    }
    for (const h of spec.hlines || []) {
      const [, py] = P(xa, h.y);
      g.strokeStyle = css(h.color || "--muted"); g.lineWidth = h.width || 1; g.setLineDash(h.dash || [5, 4]);
      g.beginPath(); g.moveTo(x0, py); g.lineTo(x1, py); g.stroke(); g.setLineDash([]);
      if (h.label) { g.fillStyle = css(h.color || "--muted"); g.font = `11px ${font}`; g.textAlign = "left"; g.textBaseline = "bottom"; g.fillText(h.label, x0 + 4, py - 2); }
    }
    for (const v of spec.vlines || []) {
      const [px] = P(v.x, ya);
      g.strokeStyle = css(v.color || "--muted"); g.lineWidth = v.width || 1; g.setLineDash(v.dash || [5, 4]);
      g.beginPath(); g.moveTo(px, y1); g.lineTo(px, y0); g.stroke(); g.setLineDash([]);
      if (v.label) { g.fillStyle = css(v.color || "--muted"); g.font = `11px ${font}`; g.textAlign = "left"; g.textBaseline = "top"; g.fillText(v.label, px + 3, y1 + 3); }
    }
    for (const s of spec.series || []) {
      g.strokeStyle = css(s.color || "--fg"); g.lineWidth = s.width || 1.5; g.setLineDash(s.dash || []);
      g.globalAlpha = s.alpha ?? 1; g.lineJoin = "round";
      g.beginPath();
      let pen = false;
      const n = s.y.length, stride = Math.max(1, Math.floor(n / ((x1 - x0) * 2)));
      for (let i = 0; i < n; i += stride) {
        const xv = s.x ? s.x[i] : i, yv = s.y[i];
        if (!Number.isFinite(yv) || !Number.isFinite(xv)) { pen = false; continue; }
        const [px, py] = P(xv, yv);
        if (pen) g.lineTo(px, py); else { g.moveTo(px, py); pen = true; }
      }
      g.stroke(); g.setLineDash([]); g.globalAlpha = 1;
    }
    for (const s of spec.points || []) {
      g.fillStyle = css(s.color || "--fg"); g.globalAlpha = s.alpha ?? 1;
      const r = s.r || 2.2;
      for (let i = 0; i < s.y.length; i++) {
        const xv = s.x ? s.x[i] : i, yv = s.y[i];
        if (!Number.isFinite(yv) || !Number.isFinite(xv)) continue;
        const [px, py] = P(xv, yv);
        g.beginPath(); g.arc(px, py, r, 0, 2 * Math.PI); g.fill();
      }
      g.globalAlpha = 1;
    }
    if (spec.extra) spec.extra(g, P, this.t);
    g.restore();

    // legend
    const items = [...(spec.series || []), ...(spec.points || [])].filter((s) => s.label);
    if (this.o.legend && items.length) {
      g.font = `11.5px ${font}`;
      const w = Math.max(...items.map((s) => g.measureText(s.label).width)) + 28, hgt = items.length * 16 + 8;
      const lx = spec.legendLeft ? x0 + 6 : x1 - w - 6, ly = y1 + 6;
      g.fillStyle = css("--panel"); g.globalAlpha = 0.86; g.fillRect(lx, ly, w, hgt); g.globalAlpha = 1;
      items.forEach((s, i) => {
        const yy = ly + 12 + i * 16;
        g.strokeStyle = g.fillStyle = css(s.color || "--fg");
        if (s.r !== undefined || (spec.points || []).includes(s)) { g.beginPath(); g.arc(lx + 12, yy, 3, 0, 2 * Math.PI); g.fill(); }
        else { g.lineWidth = 2; g.setLineDash(s.dash || []); g.beginPath(); g.moveTo(lx + 5, yy); g.lineTo(lx + 19, yy); g.stroke(); g.setLineDash([]); }
        g.fillStyle = css("--fg"); g.textAlign = "left"; g.textBaseline = "middle"; g.fillText(s.label, lx + 24, yy);
      });
    }
  }
}

// A histogram of values over [lo, hi] in nb bins: {edges, counts}.
export function histogram(values, lo, hi, nb = 30) {
  const edges = Array.from({length: nb + 1}, (_, i) => lo + (hi - lo) * i / nb), counts = new Array(nb).fill(0);
  for (const v of values) {
    const i = Math.floor((v - lo) / (hi - lo) * nb);
    if (i >= 0 && i < nb) counts[i]++;
  }
  return {edges, counts};
}
