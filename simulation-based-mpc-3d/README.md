# Simulation-Based MPC in 3D

An interactive, browser-only app for simulation-based model predictive control (MPC) in 3D, the 3D version of
[Simulation-Based MPC](../simulation-based-mpc/). A quadrotor follows a closed path that climbs and descends. At
every step of Δt = 0.25 s it predicts where each plan in a set of acceleration plans would take it, and scores every
plan by a cost. It drops the plans that come too close to another drone, an obstacle or the ground, or that go too
fast, then carries out the first step of the cheapest plan and makes the plans again from where it really is. The
planning is done by the [`sbmpc`](../sbmpc/) kernel in this repository.

**Live page:** https://marceloceccon.github.io/autonomy-playground/simulation-based-mpc-3d/

**Video:** the 2D app's, [Model Predictive Control (MPC) Explained: Simulation-Based MPC for a
USV](https://youtu.be/npO-H_k6Q-0).

## What it shows

- **The plans**, predicted with the model and drawn in 3D, coloured by the rank of their cost: cheap ones bright,
  dear ones fading. The cheapest is amber, with its steps as beads and its first step (the one carried out) drawn
  thick. Dropped plans are red up to the step where they break a constraint.
- **The model.** A point mass driven by its acceleration, held exactly over a step:
  `p_{k+1} = p_k + Δt v_k + ½ Δt² a_k`, `v_{k+1} = v_k + Δt a_k`. Positions are north, east and down (NED).
- **The plans.** An acceleration held over each of one, two or three segments, each from a lattice of evenly spaced
  values from −a_max to a_max on each axis (3, 5 or 7 north and east, 1, 3 or 5 up and down), then a coast to the
  horizon. The number of plans is the accelerations per segment to the power of the segments.
- **The cost.** `J = Σ_{k=1}^{N} |p_k − r_k|² + λ Σ_{k=0}^{N−1} |a_k|²`, with r_k a reference point that sets off
  from the point on the path nearest the drone and moves along the path at the speed U.
- **The constraints.** The other drone is predicted straight on at its present velocity and kept d_safe from. The
  obstacles (balloons, pillars, buildings) are kept d_clear from their surfaces. The altitude stays at least 1 m and
  the speed at most v_max. When every plan is dropped, the drone carries out the one that breaks a constraint latest.
- **The receding horizon.** Charts against the time from now, measured (the past) and planned (the horizon ahead),
  the three plans before the latest fading: the distance off the path (across and above it), the speed, and the
  acceleration.
- **The cost grid.** The cost of the cheapest plan for each first acceleration: north × east, with a panel for each
  vertical level. Hovering a cell or a plan in the view shows its accelerations and its cost, split into the path
  and accelerating terms. For a dropped plan it shows the step at which it breaks a constraint and, for the other
  drone, where MPC predicted it at that step.
- **The drone** is the same model, simulated every 20 ms, except that its acceleration follows the commanded one with
  a first-order lag τ, and an optional wind pushes it east. The model leaves both out. The drone tilts by its
  acceleration and turns its nose along its velocity.

The defaults: U = 4 m/s, a horizon of 5 s (N = 20), two segments of 1.5 s, then a 2 s coast, and 27 accelerations a
segment of up to 2.5 m/s² (729 plans). Also λ = 0.2, v_max = 8 m/s, d_safe = 5 m and d_clear = 1.5 m. With them the
drone brakes on the first leg and lets the other drone cross ahead of it.

## Controls

The buttons: Run/Pause, **Step Δt** (carry out one step, then plan again), Reset, the time scale, and a view that
follows the drone, shows the whole scene, or looks from above. Sliders and buttons set:

- the horizon, the segments and their length, and the accelerations per axis and their limit;
- λ, U, v_max, d_safe, d_clear, and the other drone (on or off, and its speed);
- obstacles placed on the path ahead, or cleared;
- the acceleration lag, and the wind.

Drag in the view to turn it, right-drag to pan, and scroll to zoom. Click the drone, the other drone or an obstacle
to move it with the arrows: anywhere in 3D for a balloon, across the ground for pillars and buildings. The obstacles
can be moved while the drone flies. Dragging the other drone along its line moves it on the line; dragging it
across moves the line. A selected obstacle can be resized or removed (Delete). Escape lets go.

On a phone or tablet, one finger turns the view and two zoom and pan. A tap selects, or shows the plan under it.
On a narrow screen the view stays at the top while the controls and charts scroll under it.

## Running it

`index.html`, `sim.js` (the scene and the simulation, without drawing: it runs in Node as well) and `view3d.js` (the
3D view). The page uses `../common/` and `../sbmpc/`. Unlike the other apps it has a dependency:
[three.js](https://threejs.org/) 0.170.0, loaded from cdn.jsdelivr.net through an import map. There is still no build
step. If three.js cannot load, or there is no WebGL, the charts still run. Serve the repository with any static server
and open `simulation-based-mpc-3d/` (see the repository's README).

## References

- T. A. Johansen, T. Perez and A. Cristofaro, "Ship collision avoidance and COLREGS compliance using simulation-based
  control behavior selection with predictive hazard assessment," *IEEE Transactions on Intelligent Transportation
  Systems*, 17(12), 3407–3422, 2016.
- S. Liu, N. Atanasov, K. Mohta and V. Kumar, "Search-based motion planning for quadrotors using linear quadratic
  minimum time control," *IEEE/RSJ IROS*, 2017.

## License

MIT, see `LICENSE` in the repository root.
