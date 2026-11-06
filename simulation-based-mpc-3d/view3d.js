// The 3D view of the app, drawn with Three.js: the path, the drone and its fan of plans, the other drone and the
// obstacles, which can be picked and moved with a 3-axis gizmo.
//
// The simulation works in NED (north, east, down). Everything it knows about is put in the group `ned`, whose matrix
// turns NED into Three.js's frame (x east, y up, z south), so the objects in it use NED coordinates as they are, and
// the kernel's predicted positions go to the GPU unchanged.
import * as THREE from "three";
import {OrbitControls} from "three/addons/controls/OrbitControls.js";
import {TransformControls} from "three/addons/controls/TransformControls.js";
import {Line2} from "three/addons/lines/Line2.js";
import {LineMaterial} from "three/addons/lines/LineMaterial.js";
import {LineGeometry} from "three/addons/lines/LineGeometry.js";
import {LineSegments2} from "three/addons/lines/LineSegments2.js";
import {LineSegmentsGeometry} from "three/addons/lines/LineSegmentsGeometry.js";
import {css} from "../common/plot.js";
import {PATH, LOOP, DT} from "./sim.js";

const NED = new THREE.Matrix4().set(0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1);
const toThree = (p, out = new THREE.Vector3()) => out.set(p[1], -p[2], -p[0]);
const color = (name) => new THREE.Color(css(name));
const GROUND = -0.03;                     // d of things drawn on the ground (just above it)
const DRAWN = 2;                          // the drones are drawn this many times their size, to be seen from afar

// a round dot (or ring) for points drawn at a fixed size on the screen
function spriteTexture(ring) {
  const c = document.createElement("canvas");
  c.width = c.height = 64;
  const g = c.getContext("2d");
  g.beginPath(); g.arc(32, 32, ring ? 26 : 28, 0, 2 * Math.PI);
  if (ring) { g.lineWidth = 7; g.strokeStyle = "#fff"; g.stroke(); } else { g.fillStyle = "#fff"; g.fill(); }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// A quadrotor in its body frame (x forward, y right, z down), its arm tips 0.45 m from the centre; the front arms
// in `front`, so its heading shows.
function makeDrone(body, front) {
  const g = new THREE.Group(), mats = [];
  const mat = (c) => { const m = new THREE.MeshStandardMaterial({color: c, roughness: 0.55, metalness: 0.15}); mats.push(m); return m; };
  const bodyMat = mat(body), frontMat = mat(front), propMat = mat(0x30343c);
  g.add(new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.22, 0.12), bodyMat));
  const nose = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.14, 0.07), frontMat);
  nose.position.set(0.19, 0, 0.01); g.add(nose);
  const arm = new THREE.BoxGeometry(0.5, 0.045, 0.035);
  const discGeo = new THREE.CylinderGeometry(0.17, 0.17, 0.01, 28).rotateX(Math.PI / 2);
  const hubGeo = new THREE.CylinderGeometry(0.03, 0.03, 0.06, 12).rotateX(Math.PI / 2);
  const bladeGeo = new THREE.BoxGeometry(0.32, 0.03, 0.006);
  g.userData.props = []; g.userData.discs = [];
  for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    const isFront = sx > 0, a = new THREE.Mesh(arm, isFront ? frontMat : bodyMat);
    a.position.set(sx * 0.16, sy * 0.16, -0.01); a.rotation.z = Math.atan2(sy, sx); g.add(a);
    const tip = new THREE.Vector3(sx * 0.32, sy * 0.32, -0.05);
    const discMat = new THREE.MeshBasicMaterial({color: isFront ? front : body, transparent: true, opacity: 0.22, depthWrite: false});
    const disc = new THREE.Mesh(discGeo, discMat);
    disc.position.copy(tip); g.add(disc); g.userData.discs.push(discMat);
    const hub = new THREE.Mesh(hubGeo, bodyMat); hub.position.copy(tip); g.add(hub);
    const prop = new THREE.Mesh(bladeGeo, propMat); prop.position.copy(tip).setZ(-0.08); prop.rotation.z = Math.random() * 3;
    g.add(prop); g.userData.props.push(prop);
  }
  g.traverse((o) => { if (o.isMesh && o.material !== undefined && !o.material.transparent) o.castShadow = true; });
  g.userData.mats = {body: bodyMat, front: frontMat};
  return g;
}

// Fat line segments in one draw call, the buffers grown as needed and reused.
class Segments {
  constructor(material) {
    this.material = material;
    this.obj = new LineSegments2(new LineSegmentsGeometry(), material);
    this.obj.frustumCulled = false;
    this.cap = 0; this.n = 0;
  }
  ensure(n) {
    if (n <= this.cap) return;
    this.cap = Math.max(n, 64);
    this.pos = new Float32Array(this.cap * 6); this.col = new Float32Array(this.cap * 6);
    const geo = new LineSegmentsGeometry();
    this.pb = new THREE.InstancedInterleavedBuffer(this.pos, 6, 1).setUsage(THREE.DynamicDrawUsage);
    this.cb = new THREE.InstancedInterleavedBuffer(this.col, 6, 1).setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute("instanceStart", new THREE.InterleavedBufferAttribute(this.pb, 3, 0));
    geo.setAttribute("instanceEnd", new THREE.InterleavedBufferAttribute(this.pb, 3, 3));
    geo.setAttribute("instanceColorStart", new THREE.InterleavedBufferAttribute(this.cb, 3, 0));
    geo.setAttribute("instanceColorEnd", new THREE.InterleavedBufferAttribute(this.cb, 3, 3));
    this.obj.geometry.dispose();
    this.obj.geometry = geo;
  }
  commit(n) {
    this.n = n;
    for (const b of [this.pb, this.cb]) {
      if (!b) continue;
      b.clearUpdateRanges(); b.addUpdateRange(0, Math.max(n, 1) * 6); b.needsUpdate = true;
    }
    this.obj.geometry.instanceCount = n;
  }
}

// A fat polyline, its points replaced at will.
class Polyline {
  constructor(material) {
    this.obj = new Line2(new LineGeometry(), material);
    this.obj.frustumCulled = false;
    this.obj.visible = false;
  }
  set(points) {                  // a flat array of x, y, z
    if (points.length < 6) { this.obj.visible = false; return; }
    const geo = new LineGeometry();
    geo.setPositions(points);
    this.obj.geometry.dispose();
    this.obj.geometry = geo;
    if (this.obj.material.dashed) this.obj.computeLineDistances();
    this.obj.visible = true;
  }
}

export class View3D {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {import("./sim.js").Sim} sim
   * @param {{hover: number, beads: boolean, dropped: boolean, shadows: boolean}} ui  shared with the page
   * @param {{onSelect?: (sel: object|null) => void, onDrag?: (dragging: boolean, sel: object|null) => void}} hooks
   */
  constructor(canvas, sim, ui, hooks = {}) {
    Object.assign(this, {canvas, sim, ui, hooks});
    const r = this.renderer = new THREE.WebGLRenderer({canvas, antialias: true});
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFSoftShadowMap;
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(45, 16 / 9, 0.1, 5000);
    this.controls = new OrbitControls(this.camera, canvas);
    Object.assign(this.controls, {enableDamping: true, dampingFactor: 0.12, maxPolarAngle: Math.PI / 2 - 0.02,
                                  minDistance: 3, maxDistance: 900, zoomToCursor: true});
    this.ned = new THREE.Group();
    this.ned.matrixAutoUpdate = false; this.ned.matrix.copy(NED);
    this.scene.add(this.ned);
    this.mode = "follow"; this.tween = null; this.dragging = false; this.selected = null;
    this.ver = -1; this.drawnHover = -2; this.lineMats = [];
    this.dot = spriteTexture(false); this.ring = spriteTexture(true);

    this._lights();
    this._ground();
    this._path();
    this._fan();
    this._drones();
    this.obstacleObjs = new Map();
    this.syncObstacles();
    this._gizmo();
    this._pointer();
    this.applyTheme();
    new ResizeObserver(() => { this.resized = true; }).observe(canvas);
    this.resized = true;
    this.resetCamera();
  }

  // ── building the scene ──────────────────────────────────────────────────────────────────────────────────────────
  _lights() {
    this.hemi = new THREE.HemisphereLight(0xf4f7fb, 0x8a909a, 1.6);   // the same light in both themes
    this.sun = new THREE.DirectionalLight(0xffffff, 2.2);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    this.sun.shadow.bias = -0.0004;
    this.sunOffset = new THREE.Vector3(30, 80, 20);
    this.scene.add(this.hemi, this.sun, this.sun.target);
    this.shadowReach(50);
  }
  shadowReach(r) {
    const c = this.sun.shadow.camera;
    c.left = -r; c.right = r; c.top = r; c.bottom = -r; c.near = 1; c.far = 300;
    c.updateProjectionMatrix();
  }

  _ground() {
    this.groundMat = new THREE.MeshStandardMaterial({roughness: 1, metalness: 0});
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(4000, 4000), this.groundMat);
    ground.rotation.x = -Math.PI / 2; ground.receiveShadow = true;
    this.scene.add(ground);
    // a 10 m grid round the path, in NED on the ground
    const pts = [], R = 260, cn = 45, ce = 50;
    for (let x = -R; x <= R; x += 10) {
      pts.push(cn + x, ce - R, GROUND, cn + x, ce + R, GROUND, cn - R, ce + x, GROUND, cn + R, ce + x, GROUND);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
    this.gridMat = new THREE.LineBasicMaterial({transparent: true, opacity: 0.9});
    this.ned.add(new THREE.LineSegments(geo, this.gridMat));
  }

  _lineMat(opts) {
    const m = new LineMaterial({linewidth: 2, transparent: true, depthWrite: false, ...opts});
    this.lineMats.push(m);
    return m;
  }

  _path() {
    this.pathMat = this._lineMat({linewidth: 2.2, opacity: 0.8});
    const path = new Polyline(this.pathMat);
    path.set([...LOOP, LOOP[0]].flatMap(([n, e, h]) => [n, e, -h]));
    this.ned.add(path.obj);
    // which way it is flown: a small cone 60 % of the way along each leg (smaller than a drone, not to be taken for one)
    this.arrowMat = new THREE.MeshBasicMaterial({transparent: true, opacity: 0.6});
    const cone = new THREE.ConeGeometry(0.32, 1.1, 14);
    for (const g of PATH.segs) {
      const m = new THREE.Mesh(cone, this.arrowMat);
      m.position.set(g.ax + g.tx * g.L * 0.6, g.ay + g.ty * g.L * 0.6, g.az + g.tz * g.L * 0.6);
      m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(g.tx, g.ty, g.tz));
      this.ned.add(m);
    }
    // the corners' heights, down to the ground
    const pts = LOOP.flatMap(([n, e, h]) => [n, e, -h, n, e, GROUND]);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
    this.stalkMat = new THREE.LineBasicMaterial({transparent: true, opacity: 0.35});
    this.ned.add(new THREE.LineSegments(geo, this.stalkMat));
    // the path's shadow on the ground
    this.pathShadowMat = this._lineMat({linewidth: 1.2, opacity: 0.25});
    const sh = new Polyline(this.pathShadowMat);
    sh.set([...LOOP, LOOP[0]].flatMap(([n, e]) => [n, e, GROUND]));
    this.ned.add(sh.obj);
  }

  _fan() {
    // the plans kept, in three bands by their rank (a line's opacity is one for all of it): the cheap ones bright,
    // the dear ones fading, as the 2D app fades them into the water
    this.bands = [[0.15, 0.95, 1.5], [0.5, 0.5, 1.2], [1.01, 0.16, 1]].map(([upTo, opacity, linewidth], i) => {
      const seg = new Segments(this._lineMat({linewidth, vertexColors: true, opacity}));
      seg.upTo = upTo; seg.obj.renderOrder = 4 - i;
      this.ned.add(seg.obj);
      return seg;
    });
    this.dropMat = this._lineMat({linewidth: 1, opacity: 0.28});
    this.drop = new Segments(this.dropMat);
    this.drop.obj.renderOrder = 1;
    this.ned.add(this.drop.obj);
    this.bestMat = this._lineMat({linewidth: 3.5, opacity: 1});
    this.firstMat = this._lineMat({linewidth: 8, opacity: 1});
    this.hoverMat = this._lineMat({linewidth: 2.6, opacity: 1});
    this.bestShadowMat = this._lineMat({linewidth: 2, opacity: 0.35});
    this.best = new Polyline(this.bestMat); this.first = new Polyline(this.firstMat);
    this.hoverLine = new Polyline(this.hoverMat); this.bestShadow = new Polyline(this.bestShadowMat);
    for (const [o, ro] of [[this.best, 4], [this.first, 5], [this.hoverLine, 6], [this.bestShadow, 1]]) { o.obj.renderOrder = ro; this.ned.add(o.obj); }
    const pointsOf = (size, tex) => {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(3 * 64), 3));
      const m = new THREE.PointsMaterial({size, sizeAttenuation: false, map: tex, transparent: true, alphaTest: 0.3, depthWrite: false});
      const p = new THREE.Points(geo, m);
      p.frustumCulled = false; p.renderOrder = 7;
      this.ned.add(p);
      return p;
    };
    this.beads = pointsOf(6, this.dot);
    this.hoverBeads = pointsOf(6, this.dot);
    this.hitMark = pointsOf(22, this.ring);
    this.intrDots = pointsOf(5, this.dot);
    this.trailMat = this._lineMat({linewidth: 1.6, opacity: 0.6});
    this.trail = new Polyline(this.trailMat);
    this.ned.add(this.trail.obj);
    this.trailLen = -1;
  }

  _drones() {
    this.drone = makeDrone(0x2b3140, 0xe8743a);
    this.drone.scale.setScalar(DRAWN);
    this.drone.userData.pick = {type: "drone"};
    this.ned.add(this.drone);
    this.intruder = makeDrone(0x7a4fb0, 0xb99cf0);
    this.intruder.scale.setScalar(DRAWN);
    this.intruder.userData.pick = {type: "intruder"};
    this.ned.add(this.intruder);
    // its sphere of d_safe: see-through, with three great circles
    this.safeMat = new THREE.MeshBasicMaterial({transparent: true, opacity: 0.07, depthWrite: false});
    this.safe = new THREE.Mesh(new THREE.SphereGeometry(1, 40, 20), this.safeMat);
    this.safeRingMat = new THREE.LineBasicMaterial({transparent: true, opacity: 0.55});
    const circle = (ax) => {
      const pts = [];
      for (let i = 0; i <= 96; i++) {
        const a = 2 * Math.PI * i / 96, c = Math.cos(a), s = Math.sin(a);
        pts.push(...(ax === 0 ? [0, c, s] : ax === 1 ? [c, 0, s] : [c, s, 0]));
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
      return new THREE.Line(geo, this.safeRingMat);
    };
    this.safe.add(circle(0), circle(1), circle(2));
    this.safe.renderOrder = 3;
    this.ned.add(this.safe);
    // a ghost of it, where MPC predicted it when a plan came too close
    this.ghostMat = new THREE.MeshBasicMaterial({transparent: true, opacity: 0.12, depthWrite: false});
    this.ghost = new THREE.Mesh(new THREE.SphereGeometry(1, 32, 16), this.ghostMat);
    this.ghost.visible = false;
    this.ned.add(this.ghost);
    this.patrolMat = this._lineMat({linewidth: 1.4, opacity: 0.6, dashed: true, dashSize: 1.5, gapSize: 1.2});
    this.patrol = new Polyline(this.patrolMat);
    this.ned.add(this.patrol.obj);
    // where each drone is over the ground: a line down and a ring
    this.dropLineMat = new THREE.LineBasicMaterial({transparent: true, opacity: 0.5});
    const stalk = () => {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(6), 3));
      const l = new THREE.Line(geo, this.dropLineMat); l.frustumCulled = false;
      this.ned.add(l); return l;
    };
    this.ringMat = new THREE.MeshBasicMaterial({transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthWrite: false});
    const ring = () => { const m = new THREE.Mesh(new THREE.RingGeometry(0.9, 1.3, 32), this.ringMat); this.ned.add(m); return m; };
    this.droneStalk = stalk(); this.droneRing = ring();
    this.intrStalk = stalk(); this.intrRing = ring();
    this.heading = 0; this.intrHeading = 0;
  }

  _obstacleMeshes(o) {
    const g = new THREE.Group(), s = o.shape;
    g.userData.pick = {type: "obstacle", id: o.id};
    const mat = new THREE.MeshStandardMaterial({roughness: o.kind === "balloon" ? 0.35 : 0.85, metalness: 0.05});
    let geo, shell;
    const dc = this.sim.P.dclear;
    if (o.kind === "balloon") {
      geo = new THREE.SphereGeometry(s.radius, 36, 24);
      shell = new THREE.SphereGeometry(s.radius + dc, 36, 24);
    } else if (o.kind === "building") {
      geo = new THREE.BoxGeometry(2 * s.half[0], 2 * s.half[1], 2 * s.half[2]);
      shell = new THREE.BoxGeometry(2 * (s.half[0] + dc), 2 * (s.half[1] + dc), 2 * (s.half[2] + dc));
    } else {
      geo = new THREE.CylinderGeometry(s.radius, s.radius, 2 * s.halfLength, 32).rotateX(Math.PI / 2);
      shell = new THREE.CylinderGeometry(s.radius + dc, s.radius + dc, 2 * (s.halfLength + dc), 32).rotateX(Math.PI / 2);
    }
    const body = new THREE.Mesh(geo, mat);
    body.castShadow = true; body.receiveShadow = true;
    g.add(body);
    if (o.kind === "building") {
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({transparent: true, opacity: 0.5}));
      g.add(edges); g.userData.edges = edges;
    }
    const shellMat = new THREE.MeshBasicMaterial({transparent: true, opacity: 0.06, depthWrite: false});
    const sh = new THREE.Mesh(shell, shellMat);
    sh.renderOrder = 3; sh.raycast = () => {};            // the shell is not picked: what is inside it is
    g.add(sh);
    if (o.kind === "balloon") {                              // its tether, down to the ground
      const tg = new THREE.BufferGeometry();
      tg.setAttribute("position", new THREE.BufferAttribute(new Float32Array(6), 3));
      const t = new THREE.Line(tg, new THREE.LineBasicMaterial({transparent: true, opacity: 0.6}));
      t.raycast = () => {};
      g.add(t); g.userData.tether = t;
    }
    Object.assign(g.userData, {body, shell: sh, mat, shellMat, kind: o.kind, dc, size: this._sizeKey(o)});
    return g;
  }
  _sizeKey(o) { const s = o.shape; return o.kind === "building" ? [...s.half].join() : o.kind === "balloon" ? `${s.radius}` : `${s.radius},${s.halfLength}`; }

  /** Make the obstacles' meshes agree with the simulation's list (after one is added, removed or resized). */
  syncObstacles() {
    const seen = new Set();
    for (const o of this.sim.obstacles) {
      seen.add(o.id);
      let g = this.obstacleObjs.get(o.id);
      if (g && (g.userData.dc !== this.sim.P.dclear || g.userData.size !== this._sizeKey(o))) {
        const wasSelected = this.tc && this.tc.object === g;
        this.ned.remove(g); g = null; this.obstacleObjs.delete(o.id);
        if (wasSelected) this._rebuildSelected = o.id;
      }
      if (!g) {
        g = this._obstacleMeshes(o);
        this.obstacleObjs.set(o.id, g); this.ned.add(g);
        this._themeObstacle(g);
        if (this._rebuildSelected === o.id) { this.tc.attach(g); this._highlight(g, true); this._rebuildSelected = null; }
      }
    }
    for (const [id, g] of this.obstacleObjs) {
      if (!seen.has(id)) {
        if (this.tc && this.tc.object === g) this.select(null);
        this.ned.remove(g); this.obstacleObjs.delete(id);
      }
    }
  }

  _gizmo() {
    const tc = this.tc = new TransformControls(this.camera, this.canvas);
    tc.setSpace("world");
    tc.setSize(matchMedia("(pointer: coarse)").matches ? 1.25 : 0.85);
    this.scene.add(tc.getHelper());
    tc.addEventListener("dragging-changed", (ev) => {
      this.controls.enabled = !ev.value;
      this.dragging = ev.value;
      if (ev.value) this._dragStart();
      if (this.hooks.onDrag) this.hooks.onDrag(ev.value, this.selected);
    });
    tc.addEventListener("objectChange", () => this._dragMove());
  }

  // ── picking and moving ──────────────────────────────────────────────────────────────────────────────────────────
  _pointer() {
    const c = this.canvas;
    this.ray = new THREE.Raycaster();
    let down = null;
    c.addEventListener("pointerdown", (ev) => { down = {x: ev.clientX, y: ev.clientY, type: ev.pointerType}; });
    c.addEventListener("pointerup", (ev) => {
      if (!down || this.dragging) { down = null; return; }
      const moved = Math.hypot(ev.clientX - down.x, ev.clientY - down.y);
      if (moved < 6 && this.tc.axis === null) {
        const pick = this.pick(ev);
        if (pick) this.select(pick);
        else {
          this.select(null);
          if (down.type !== "mouse") { const [X, Y] = this._xy(ev); this.ui.hover = this.planNear(X, Y, 14); }
        }
      }
      down = null;
    });
    c.addEventListener("pointermove", (ev) => {
      if (ev.pointerType !== "mouse" || this.dragging || (ev.buttons && !this.dragging)) return;
      this._pending = ev;
    });
    c.addEventListener("pointerleave", () => { this._pending = null; if (!this.dragging) this.ui.hover = -1; });
  }
  _xy(ev) { const r = this.canvas.getBoundingClientRect(); return [ev.clientX - r.left, ev.clientY - r.top]; }

  /** What is under the pointer: the drone, the other drone or an obstacle, as {type, id?}, or null. */
  pick(ev) {
    const [X, Y] = this._xy(ev), r = this.canvas.getBoundingClientRect();
    this.ray.setFromCamera(new THREE.Vector2(X / r.width * 2 - 1, -(Y / r.height) * 2 + 1), this.camera);
    const targets = [this.drone, ...(this.sim.P.intruder ? [this.intruder] : []), ...this.obstacleObjs.values()];
    const hits = this.ray.intersectObjects(targets, true);
    for (const h of hits) {
      let o = h.object;
      while (o && !o.userData.pick) o = o.parent;
      if (o) return o.userData.pick;
    }
    // small things are hard to hit: take a drone within 14 px of the pointer
    for (const [obj, pick] of [[this.drone, {type: "drone"}], ...(this.sim.P.intruder ? [[this.intruder, {type: "intruder"}]] : [])]) {
      const s = this._screen(obj.getWorldPosition(new THREE.Vector3()));
      if (s && Math.hypot(s[0] - X, s[1] - Y) < 14) return pick;
    }
    return null;
  }
  _screen(v) {
    const p = v.clone().project(this.camera);
    if (p.z > 1) return null;
    const r = this.canvas.getBoundingClientRect();
    return [(p.x + 1) / 2 * r.width, (1 - p.y) / 2 * r.height];
  }

  objectOf(sel) {
    if (!sel) return null;
    if (sel.type === "drone") return this.drone;
    if (sel.type === "intruder") return this.intruder;
    return this.obstacleObjs.get(sel.id) || null;
  }

  select(sel) {
    const prev = this.objectOf(this.selected);
    if (prev) this._highlight(prev, false);
    this.selected = sel;
    const obj = this.objectOf(sel);
    if (obj) {
      this.tc.attach(obj);
      // buildings and pillars stand on the ground: they move across it only
      const grounded = sel.type === "obstacle" && obj.userData.kind !== "balloon";
      this.tc.showY = !grounded;
      this._highlight(obj, true);
    } else this.tc.detach();
    if (this.hooks.onSelect) this.hooks.onSelect(sel);
  }
  _highlight(obj, on) {
    const ms = obj.userData.mats ? Object.values(obj.userData.mats) : obj.userData.mat ? [obj.userData.mat] : [];
    for (const m of ms) m.emissive?.set(on ? 0x553300 : 0x000000);
  }

  _dragStart() {
    const sel = this.selected, sim = this.sim;
    if (!sel) return;
    if (sel.type === "intruder") {
      const it = sim.intruderAt(sim.t);
      this.drag0 = {p: new THREE.Vector3(...it.p), along: it.along, a: Float64Array.from(sim.ia), b: Float64Array.from(sim.ib), u: it.u};
    }
  }
  _dragMove() {
    const sel = this.selected, sim = this.sim, obj = this.objectOf(sel);
    if (!sel || !obj) return;
    const p = obj.position;
    if (sel.type === "drone") {
      if (p.z > -0.2) p.z = -0.2;                        // not into the ground
      sim.p.set([p.x, p.y, p.z]); sim.trail = []; sim.plans = [];
    } else if (sel.type === "intruder") {
      const d0 = this.drag0, dp = new THREE.Vector3().subVectors(p, d0.p), u = new THREE.Vector3(...d0.u);
      const along = dp.dot(u), perp = dp.clone().addScaledVector(u, -along);
      // along its line: where it is on it; across: the line itself moves
      for (const [dst, src] of [[sim.ia, d0.a], [sim.ib, d0.b]]) {
        dst[0] = src[0] + perp.x; dst[1] = src[1] + perp.y; dst[2] = Math.min(src[2] + perp.z, -0.5);
      }
      sim.setIntruderAlong(d0.along + along);
      sim.closest = Infinity;
    } else {
      const o = sim.obstacles.find((x) => x.id === sel.id);
      if (!o) return;
      if (o.kind === "balloon") p.z = Math.min(p.z, -o.shape.radius);   // a balloon above the ground
      o.shape.center.set([p.x, p.y, p.z]);
      sim.clearest = Infinity;
    }
    sim.replan();
  }

  /** The plan drawn nearest the screen point (X, Y), within `tol` px, or -1. */
  planNear(X, Y, tol = 8) {
    const pl = this.sim.planner, N = pl.N, T = pl.traj, stride = 3 * (N + 1), r = this.canvas.getBoundingClientRect();
    const M = new THREE.Matrix4().multiplyMatrices(this.camera.projectionMatrix, this.camera.matrixWorldInverse).multiply(this.ned.matrixWorld);
    const e = M.elements, W = r.width, Hh = r.height;
    let best = -1, bd = tol;
    const show = this.ui.dropped;
    for (let p = 0; p < pl.count; p++) {
      if (pl.hit[p] >= 0 && !show) continue;
      const last = pl.hit[p] >= 0 ? pl.hit[p] + 1 : N;
      let ax = 0, ay = 0, ok = false;
      for (let k = 0; k <= last; k++) {
        const o = p * stride + 3 * k, x = T[o], y = T[o + 1], z = T[o + 2];
        const w = e[3] * x + e[7] * y + e[11] * z + e[15];
        if (w <= 0.1) { ok = false; continue; }
        const bx = ((e[0] * x + e[4] * y + e[8] * z + e[12]) / w + 1) / 2 * W, by = (1 - (e[1] * x + e[5] * y + e[9] * z + e[13]) / w) / 2 * Hh;
        if (ok) {
          const dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
          const u = L2 > 0 ? Math.min(Math.max(((X - ax) * dx + (Y - ay) * dy) / L2, 0), 1) : 0;
          const d = Math.hypot(ax + u * dx - X, ay + u * dy - Y);
          if (d < bd) { bd = d; best = p; }
        }
        ax = bx; ay = by; ok = true;
      }
    }
    return best;
  }

  // ── the camera ──────────────────────────────────────────────────────────────────────────────────────────────────
  _sceneBox() {
    const box = new THREE.Box3();
    for (const [n, e, h] of LOOP) box.expandByPoint(toThree([n, e, -h]));
    box.expandByPoint(new THREE.Vector3(box.min.x, 0, box.min.z));
    return box.expandByScalar(12);
  }
  setMode(mode) {
    this.mode = mode;
    const box = this._sceneBox(), c = box.getCenter(new THREE.Vector3()), R = box.getSize(new THREE.Vector3()).length() / 2;
    const dist = R / Math.sin(THREE.MathUtils.degToRad(this.camera.fov) / 2) * 0.95;
    if (mode === "whole") this._flyTo(c, c.clone().add(new THREE.Vector3(-0.45, 0.62, 0.65).normalize().multiplyScalar(dist)));
    else if (mode === "top") this._flyTo(c, c.clone().add(new THREE.Vector3(0, 1, 0.012).multiplyScalar(dist * 0.95)));
    else this._flyTo(...this._followPose());
    this.shadowReach(mode === "follow" ? 50 : 170);
  }
  _followPose() {
    const p = toThree(this.sim.p), v = toThree(this.sim.v);
    v.y = 0;
    if (v.lengthSq() < 0.01) v.set(0, 0, -1);
    v.normalize();
    const target = p.clone().addScaledVector(v, 7);
    return [target, p.clone().addScaledVector(v, -30).add(new THREE.Vector3(-v.z, 0, v.x).multiplyScalar(-12)).setY(p.y + 19)];
  }
  resetCamera() {
    const [t, pos] = this.mode === "follow" ? this._followPose() : [this.controls.target.clone(), this.camera.position.clone()];
    if (this.mode === "follow") { this.controls.target.copy(t); this.camera.position.copy(pos); this.tween = null; }
    else this.setMode(this.mode);
  }
  _flyTo(target, pos) {
    this.tween = {t: 0, t0: this.controls.target.clone(), p0: this.camera.position.clone(), t1: target, p1: pos};
  }
  _updateCamera(dt) {
    const c = this.controls;
    if (this.tween) {
      const tw = this.tween;
      tw.t = Math.min(1, tw.t + dt / 0.7);
      const s = tw.t * tw.t * (3 - 2 * tw.t);
      if (this.mode === "follow") { const [t1, p1] = this._followPose(); tw.t1 = t1; tw.p1 = p1; }
      c.target.lerpVectors(tw.t0, tw.t1, s); this.camera.position.lerpVectors(tw.p0, tw.p1, s);
      if (tw.t >= 1) this.tween = null;
    } else if (this.mode === "follow" && !this.dragging) {
      const p = toThree(this.sim.p), v = toThree(this.sim.v);
      const look = v.multiplyScalar(0.3 * this.sim.P.T);
      if (look.length() > 10) look.setLength(10);
      const want = p.add(look), k = 1 - Math.exp(-dt * 2.5), d = want.sub(c.target).multiplyScalar(k);
      c.target.add(d); this.camera.position.add(d);
    }
    c.update();
    this.sun.target.position.copy(c.target);
    this.sun.position.copy(c.target).add(this.sunOffset);
  }

  // ── colours ─────────────────────────────────────────────────────────────────────────────────────────────────────
  applyTheme() {
    const sky = color("--sky");
    this.scene.background = sky;
    this.scene.fog = new THREE.Fog(sky, 160, 700);
    this.groundMat.color = color("--ground");
    this.gridMat.color = color("--gridline");
    for (const m of [this.pathMat, this.arrowMat, this.stalkMat, this.pathShadowMat]) m.color = color("--truth");
    this.trailMat.color = color("--muted");
    this.dropMat.color = color("--bad");
    for (const m of [this.bestMat, this.firstMat, this.bestShadowMat]) m.color = color("--best");
    this.beads.material.color = color("--best");
    this.hoverMat.color = color("--fg"); this.hoverBeads.material.color = color("--fg");
    this.hitMark.material.color = color("--bad");
    for (const m of [this.safeMat, this.safeRingMat, this.ghostMat, this.patrolMat]) m.color = color("--intr");
    this.intrDots.material.color = color("--intr");
    this.dropLineMat.color = color("--muted"); this.ringMat.color = color("--muted");
    this.drone.userData.mats.body.color = color("--hull"); this.drone.userData.mats.front.color = color("--deck");
    this.intruder.userData.mats.body.color = color("--intr"); this.intruder.userData.mats.front.color = color("--intr-light");
    for (const g of this.obstacleObjs.values()) this._themeObstacle(g);
    // the fan: cheap to dear
    const a = color("--cheap"), b = color("--dear");
    this.pal = Array.from({length: 65}, (_, i) => a.clone().lerp(b, i / 64));
    this.ver = -1;
  }
  _themeObstacle(g) {
    const kind = g.userData.kind;
    g.userData.mat.color = color(kind === "balloon" ? "--balloon" : "--obst");
    g.userData.shellMat.color = color("--bad");
    if (g.userData.edges) g.userData.edges.material.color = color("--obst-edge");
    if (g.userData.tether) g.userData.tether.material.color = color("--muted");
  }

  // ── every frame ─────────────────────────────────────────────────────────────────────────────────────────────────
  frame(dt) {
    if (this.resized) {
      const w = this.canvas.clientWidth || 1, h = this.canvas.clientHeight || 1;
      this.renderer.setSize(w, h, false);
      this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
      this.resized = false;
    }
    if (this._pending) {
      const [X, Y] = this._xy(this._pending);
      this.canvas.style.cursor = this.tc.axis ? "" : this.pick(this._pending) ? "pointer" : "";
      this.ui.hover = this.planNear(X, Y);
      this._pending = null;
    }
    this._updateDrones(dt);
    this._updateObstacles();
    this._updateTrail();
    if (this.sim.version !== this.ver) this._updateFan();
    if (this.ui.hover !== this.drawnHover || this.sim.version !== this.hoverVer) this._updateHover();
    this._updateCamera(dt);
    this.renderer.render(this.scene, this.camera);
  }

  _updateDrones(dt) {
    const sim = this.sim, P = sim.P;
    // the drone: tilted by its acceleration (its thrust holds it up and pushes it along), nose along its velocity
    if (!(this.dragging && this.selected?.type === "drone")) this.drone.position.set(...sim.p);
    this._attitude(this.drone, sim.aAct, sim.v, "heading", dt);
    this._stalk(this.droneStalk, this.droneRing, this.drone.position, this.ui.shadows);
    const spin = dt * 60;
    for (const pr of this.drone.userData.props) pr.rotation.z += spin;
    // the other drone
    const on = P.intruder;
    for (const o of [this.intruder, this.safe, this.patrol.obj]) o.visible = on;
    if (!on) {
      for (const o of [this.ghost, this.intrDots, this.intrStalk, this.intrRing]) o.visible = false;
      return;
    }
    const it = sim.intruderAt(sim.t);
    if (!(this.dragging && this.selected?.type === "intruder")) this.intruder.position.set(...it.p);
    this._attitude(this.intruder, [0, 0, 0], it.v, "intrHeading", dt);
    for (const pr of this.intruder.userData.props) pr.rotation.z += spin;
    this.safe.position.copy(this.intruder.position); this.safe.scale.setScalar(P.dsafe);
    this._stalk(this.intrStalk, this.intrRing, this.intruder.position, this.ui.shadows);
    const key = [...sim.ia, ...sim.ib].join();
    if (key !== this.patrolKey) { this.patrol.set([...sim.ia, ...sim.ib]); this.patrolKey = key; }
  }
  _attitude(obj, acc, vel, key, dt) {
    const vh = Math.hypot(vel[0], vel[1]);
    if (vh > 0.3) {
      const want = Math.atan2(vel[1], vel[0]);
      let d = want - this[key];
      d = Math.atan2(Math.sin(d), Math.cos(d));
      this[key] += d * Math.min(1, dt * 6);
    }
    const zb = new THREE.Vector3(-acc[0], -acc[1], 9.81 - acc[2]).normalize();          // down, the opposite of thrust
    const h = new THREE.Vector3(Math.cos(this[key]), Math.sin(this[key]), 0);
    const xb = h.addScaledVector(zb, -h.dot(zb)).normalize(), yb = new THREE.Vector3().crossVectors(zb, xb);
    obj.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(xb, yb, zb));
  }
  _stalk(line, ring, p, visible) {
    const a = line.geometry.attributes.position;
    a.setXYZ(0, p.x, p.y, p.z); a.setXYZ(1, p.x, p.y, GROUND); a.needsUpdate = true;
    ring.position.set(p.x, p.y, GROUND);
    line.visible = ring.visible = visible;
  }

  _updateObstacles() {
    let stale = false;
    for (const o of this.sim.obstacles) {
      const g = this.obstacleObjs.get(o.id);
      if (!g || g.userData.dc !== this.sim.P.dclear || g.userData.size !== this._sizeKey(o)) { stale = true; continue; }
      const dragged = this.dragging && this.selected?.type === "obstacle" && this.selected.id === o.id;
      if (!dragged) g.position.set(...o.shape.center);
      const t = g.userData.tether;
      if (t) {
        const a = t.geometry.attributes.position, r = o.shape.radius, d = g.position.z;
        a.setXYZ(0, 0, 0, r); a.setXYZ(1, 0, 0, GROUND - d); a.needsUpdate = true;
      }
    }
    if (stale || this.obstacleObjs.size !== this.sim.obstacles.length) this.syncObstacles();
  }

  _updateTrail() {
    const tr = this.sim.trail;
    if (tr.length === this.trailLen && tr.length && tr[tr.length - 1] === this.trailLast) return;
    this.trailLen = tr.length; this.trailLast = tr[tr.length - 1];
    this.trail.set(tr.flat());
  }

  _updateFan() {
    const sim = this.sim, pl = sim.planner, N = pl.N, T = pl.traj, stride = 3 * (N + 1), show = this.ui.dropped;
    this.ver = sim.version;
    let nd = 0;
    for (let p = 0; p < pl.count; p++) if (p !== sim.chosen && pl.hit[p] >= 0 && show) nd += pl.hit[p] + 1;
    for (const b of this.bands) b.ensure(pl.feasible * N);
    this.drop.ensure(nd);
    // dropped plans, up to where they break a constraint
    let j = 0;
    const dp = this.drop.pos, dc = this.drop.col;
    if (show) {
      for (let p = 0; p < pl.count; p++) {
        if (pl.hit[p] < 0 || p === sim.chosen) continue;
        for (let k = 0, last = pl.hit[p] + 1; k < last; k++, j++) {
          const o = p * stride + 3 * k;
          for (let i = 0; i < 6; i++) { dp[6 * j + i] = T[o + i]; dc[6 * j + i] = 1; }
        }
      }
    }
    this.drop.commit(j);
    // the plans kept, the dear ones first so the cheap ones are drawn over them
    const n = this.bands.map(() => 0);
    for (let i = pl.feasible - 1; i >= 0; i--) {
      const p = pl.sorted[i];
      if (p === sim.chosen) continue;
      const r = Math.pow(pl.rank[p], 0.8), c = this.pal[Math.round(r * 64)];
      const bi = this.bands.findIndex((b) => r < b.upTo), b = this.bands[bi], kp = b.pos, kc = b.col;
      let j = n[bi];
      for (let k = 0; k < N; k++, j++) {
        const o = p * stride + 3 * k;
        for (let q = 0; q < 6; q++) kp[6 * j + q] = T[o + q];
        kc[6 * j] = kc[6 * j + 3] = c.r; kc[6 * j + 1] = kc[6 * j + 4] = c.g; kc[6 * j + 2] = kc[6 * j + 5] = c.b;
      }
      n[bi] = j;
    }
    this.bands.forEach((b, i) => b.commit(n[i]));
    // the plan carried out: the cheapest (or, when none is kept, the one that breaks a constraint latest)
    const b = sim.chosen;
    if (b >= 0) {
      const pts = Array.from(T.subarray(b * stride, (b + 1) * stride));
      this.best.set(pts);
      this.first.set(pts.slice(0, 6));
      this.bestShadow.set(pts.map((v, i) => (i % 3 === 2 ? GROUND : v)));
      this.bestShadow.obj.visible = this.ui.shadows;
      this._points(this.beads, pts.slice(3), this.ui.beads);
      this.bestMat.color = color(sim.none ? "--bad" : "--best");
    }
    // where MPC predicts the other drone over the horizon
    if (sim.seen) {
      const pts = [];
      for (let k = 2; k <= N; k += 2) for (let i = 0; i < 3; i++) pts.push(sim.seen.p[i] + sim.seen.v[i] * k * DT);
      this._points(this.intrDots, pts, true);
    }
  }
  _points(obj, pts, visible) {
    const a = obj.geometry.attributes.position;
    if (a.array.length < pts.length) {
      obj.geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(pts.length * 2), 3));
    }
    obj.geometry.attributes.position.array.set(pts);
    obj.geometry.attributes.position.needsUpdate = true;
    obj.geometry.setDrawRange(0, pts.length / 3);
    obj.visible = visible && pts.length > 0;
  }

  _updateHover() {
    const sim = this.sim, pl = sim.planner, p = this.ui.hover, N = pl.N, stride = 3 * (N + 1);
    this.drawnHover = p; this.hoverVer = sim.version;
    this.ghost.visible = false; this.hitMark.visible = false;
    if (p < 0 || p >= pl.count) { this.hoverLine.obj.visible = false; this.hoverBeads.visible = false; return; }
    const last = pl.hit[p] >= 0 ? pl.hit[p] + 1 : N;
    const pts = Array.from(pl.traj.subarray(p * stride, p * stride + 3 * (last + 1)));
    this.hoverLine.set(pts);
    this._points(this.hoverBeads, pts.slice(3), true);
    if (pl.hit[p] >= 0) {
      this._points(this.hitMark, pts.slice(-3), true);
      const c = pl.constraints[pl.what[p]];
      if (c === sim.intr && sim.seen) {                       // the other drone where MPC predicted it at that step
        const t = (pl.hit[p] + 1) * DT;
        this.ghost.position.set(...sim.seen.p.map((x, i) => x + sim.seen.v[i] * t));
        this.ghost.scale.setScalar(sim.P.dsafe);
        this.ghost.visible = true;
      }
    }
  }

  /** The angle (rad, clockwise from screen up) at which north points on the screen. */
  northAngle() {
    const t = this.controls.target, a = this._screen(t), b = this._screen(t.clone().add(new THREE.Vector3(0, 0, -10)));
    if (!a || !b) return 0;
    return Math.atan2(b[0] - a[0], -(b[1] - a[1]));
  }
}
