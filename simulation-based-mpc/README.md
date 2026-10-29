# Simulation-Based MPC

An interactive, browser-only app for simulation-based model predictive control (MPC). An Otter uncrewed surface
vehicle (USV), the 2 m vessel made by Maritime Robotics, follows a closed path. At every step of Δt = 0.5 s it predicts
where each plan in a fan of turn-rate plans would take it, scores every plan by a cost, drops the plans that break a
constraint, carries out the first step of the cheapest plan, and makes the fan again from where it really is.

**Live page:** https://erlendcoates.github.io/autonomy-playground/simulation-based-mpc/

**Video:** [Model Predictive Control (MPC) Explained: Simulation-Based MPC for a USV](https://youtu.be/npO-H_k6Q-0),
in the playlists [Guidance and Control](https://www.youtube.com/playlist?list=PLcYnzy5eJtss) and [Autonomous
Systems](https://www.youtube.com/playlist?list=PLCppSPyphAW0).

## What it shows

- **The fan.** Every plan, predicted with the model and drawn on the water, coloured by the rank of its cost (cheap
  bright, dear fading), the cheapest one in amber with its steps as beads, and its first step (the one carried out)
  drawn thick. Dropped plans are red up to the step where they break a constraint.
- **The model.** Kinematic, no current and no sideslip, one step ahead by forward Euler:
  `xⁿ_{k+1} = xⁿ_k + Δt U cos ψ_k`, `yⁿ_{k+1} = yⁿ_k + Δt U sin ψ_k`, `ψ_{k+1} = ψ_k + Δt r_k`.
- **The plans.** The turn rate is held constant in each of one, two or three segments; each segment takes one of an
  evenly spaced set of rates from −r_max to r_max. The number of plans is the rates to the power of the segments.
- **The cost.** `J = Σ_{k=1}^{N} y_{e,k}² + λ Σ_{k=0}^{N−1} r_k²`, with y_e the distance from the path ahead of the Otter
  (from the point nearest it on, so that sailing the loop backwards does not score well; signed in the charts, positive to
  starboard).
- **The constraints.** The turn-rate limit (no plan asks for more), a kayak paddling back and forth across the first
  leg (predicted straight on at its present velocity, kept d_safe from), and buoys (kept 2.2 m from). When every plan
  is dropped, the Otter keeps its last turn rate.
- **The receding horizon.** Two charts against the time from now: the distance off the path and the turn rate,
  measured (the past) and planned (the horizon ahead), the three plans before the latest fading.
- **The cost grid.** The cost of every plan over (r_a, r_b), cheapest outlined; with three segments each cell holds its
  cheapest r_c. Hovering a cell or a plan on the map shows its cost split into the path and turning terms, or the step
  at which it breaks a constraint (and, for the kayak, where MPC predicted it at that step).
- **The Otter** is the same kinematic model, simulated every 20 ms, except that its turn rate follows the commanded
  one with a first-order lag τ that the model leaves out.

The defaults are the video's: U = 1.5 m/s, Δt = 0.5 s, a horizon of 15 s (N = 30), r_a for 4 s and r_b for 11 s,
9 rates from −8 to 8 deg/s (81 plans), λ = 0.05 and d_safe = 8 m. With them the Otter passes behind the kayak.

## Controls

Run/Pause, **Step Δt** (carry out one step, then plan again), Reset, the time scale, and a view that follows the
Otter or shows the whole path. Sliders and buttons set the horizon, the segments and their rates, the first segment's
length, λ, r_max, d_safe, the kayak on or off, buoys on the path ahead, the speed and the turn lag. Drag the Otter to
move it, the small circle at its bow to turn it, the kayak along its line, or a buoy.

On a phone or tablet, touch the Otter, its bow handle, the kayak or a buoy to drag it, or a plan to see its cost; a
touch anywhere else scrolls the page. On a narrow screen the map stays at the top while the controls and charts scroll
under it.

## Simulation-based and optimization-based MPC

Simulation-based MPC tries every plan in a small set and keeps the cheapest; optimization-based MPC searches for the
best plan with a numerical solver instead. A well-known simulation-based MPC for ships is SB-MPC (Johansen, Perez and
Cristofaro, 2016): it simulates the ship for a small set of course and speed offsets, each held over the horizon, and
keeps the one with the lowest cost, with collision risk and COLREGs compliance in the cost. The MPC here differs in its
details (turn-rate plans in segments, following a path), but the idea is the same.

## Running it

A single HTML file using `../common/style.css` and `../common/plot.js`, no build step and no dependencies apart from a
web font. Serve the repository with any static server and open `simulation-based-mpc/` (see the repository's
README).

## References

- T. A. Johansen, T. Perez and A. Cristofaro, "Ship collision avoidance and COLREGS compliance using simulation-based
  control behavior selection with predictive hazard assessment," *IEEE Transactions on Intelligent Transportation
  Systems*, 17(12), 3407–3422, 2016.
- T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley, 2021.

## License

MIT, see `LICENSE` in the repository root.
