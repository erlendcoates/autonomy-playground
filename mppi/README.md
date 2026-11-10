# MPPI

An interactive, browser-only app for model predictive path integral control (MPPI). An Otter uncrewed surface vehicle
(USV), the 2 m vessel made by Maritime Robotics, sails Nørvasundet in Ålesund. At every step it adds random wiggles to
its current plan, simulates every plan with a model, weights each by its cost, and carries out the first step of the
weighted average.

**Live page:** https://erlendcoates.github.io/autonomy-playground/mppi/

**Video:** [Model Predictive Path Integral (MPPI) Control Explained: Sampling-Based MPC for a USV](https://youtu.be/LReozcbB-ZE),
in the playlists [Guidance and Control](https://www.youtube.com/playlist?list=PLcYnzy5eJtss) and [Autonomous
Systems](https://www.youtube.com/playlist?list=PLCppSPyphAW0).

## What it shows

- **Two tasks, one MPPI.** A button switches between following a closed path and collecting drifting litter. The core is
  the same; only the cost changes. Each task keeps its own settings, starting from the video's.
- **Following a path.** `J_i = Σ y_{e,k}² + λ Σ r_{i,k}²`, with M = 1000, Δt = 0.5 s, N = 60 (30 s ahead), σ = 3 deg/s, α = 0.8, T = 40,
  λ = 0.05 and r_max = 10 deg/s (the video: N = 30 and 8 deg/s). Plans into shallow water, near a buoy or near another Otter sailing the loop the other way are dropped.
- **Collecting litter.** A camera with a narrow view, a cost of four terms (look where it hasn't looked lately, keep seen
  litter in view, stay inside the area, avoid hard turns), a simple pursuit within 15 m, and a net between the hulls. M = 150,
  Δt = 1 s, N = 30, σ = 4 deg/s, α = 0.8, T = 3.
  Optionally (not in the video) speed is a second input: each plan also has a speed for every step, with its own smooth
  wiggle, between 0.3 m/s and U_max, and a fifth term keeps it near U; the Otter's speed follows with a 1.5 s lag.
- **The votes.** Each plan's vote against its cost above the cheapest, with the effective number of plans. A switch steers by
  the cheapest plan instead of the weighted average.
- **The Otter** is the same kinematic model as the plans, simulated every 20 ms, except for a first-order lag τ in its turn rate (and 1.5 s in its speed).

## Controls

Drag a square handle to reshape the path; double-click a handle to remove it, or a leg to add one. Drag the Otter, its bow
handle, a buoy or the other Otter. Space pauses. On a touch screen, touch what you want to drag; anywhere else scrolls.

## The map

`make_map.py` makes `map_light.jpg`, `map_dark.jpg` and `depth.bin` from the open data behind the video's digital twin: terrain
and sea chart depth data © Kartverket (CC BY 4.0), roads © Statens vegvesen, NVDB (NLOD), buildings © OpenStreetMap contributors
(ODbL). The depths are low resolution and only for this app.

## References

G. Williams, A. Aldrich and E. A. Theodorou, "Model predictive path integral control: from theory to parallel computation,"
*Journal of Guidance, Control, and Dynamics*, 40(2), 344–357, 2017.

Sanjeev Ramkumar Sudha, Marija Popović and Erlend M. Coates, "MPPI-based informative trajectory planning for search and capture of
drifting targets with ASVs," accepted for publication in *IEEE Robotics and Automation Letters*. [arXiv:2606.12019](https://arxiv.org/abs/2606.12019).
Code: [github.com/sanjeevrs2000/mppi-search-and-capture](https://github.com/sanjeevrs2000/mppi-search-and-capture).
