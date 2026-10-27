# LOS Guidance

An interactive, browser-only app for line-of-sight (LOS) path-following guidance. Two point-particle vessels follow
the same waypoint path with the same lookahead-based LOS law, but with different ways of choosing the reference
point on the path. Reshape the path while they are moving and watch how each reacts.

**Live page:** https://erlendcoates.github.io/autonomy-playground/los-guidance/

**Video:** [Line-of-Sight (LOS) Guidance Explained: Path Following for USVs, AUVs and
Drones](https://youtu.be/e-B-lBFzHRs), in the playlist [Guidance and
Control](https://www.youtube.com/playlist?list=PLcYnzy5eJtss).

## What it shows

- **Path.** Straight legs between draggable waypoints, joined by the symmetric polynomial transition curves of
  Kobryń (2014). The curves have zero curvature at both ends and are sized by a minimum turn radius. The path is a
  closed loop by default and can be opened; it can be followed in either direction. A dashed line joins the
  waypoints.
- **Guidance.** Lookahead-based LOS guidance (Breivik and Fossen, 2005; Fossen, 2021) in vector form. The desired
  velocity is `v_d = U η_d` with the LOS direction `η_d = (Δ t − e) / sqrt(Δ² + |e|²)`, where `t` is the path
  tangent at the reference point, `e` the cross-track error to it and `Δ` the lookahead distance.
- **Reference point, two ways.**
  - *Projected:* the reference is the point on the path closest to the vessel. Simple, but it can jump when the
    vessel is far from the path or near a tight turn.
  - *Propagated particle:* the reference moves along the path with its own dynamics,
    `ṡ_r = t·ṗ + Δ_s tanh(k_s s_e / Δ_s)`, so it tracks the vessel's progress and pulls the along-track error
    `s_e` to zero without ever jumping.
- **Vessel.** Drawn as a hull outline pointing along its velocity, but modelled as a point particle whose velocity
  follows `v_d` with a first-order lag `τ` and an optional speed cap. No heading dynamics, no environmental
  disturbances.

The strip charts show the cross-track error and the speed over the last 120 s; the table shows the live state of
both vessels.

## Controls

Drag a square handle to move a waypoint. Drag a vessel to move it. Space pauses. Sliders set the cruise speed and
its ramp rate, the path's minimum turn radius, the lookahead distance, the particle gains, the vessel's velocity lag
and the simulation time scale.

On a phone or tablet, touch a handle or a vessel to drag it; a touch anywhere else scrolls the page. On a narrow
screen the map stays at the top while the controls and plots scroll under it.

## Running it

It is a single HTML file with no build step and no dependencies apart from a web font. Open `index.html` in a
browser, or serve the repository with any static server (see the repository's README).

## References

- M. Breivik and T. I. Fossen, "Principles of guidance-based path following in 2D and 3D," *Proc. 44th IEEE
  Conference on Decision and Control*, 2005.
- T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley, 2021.
- A. Kobryń, "New solutions for general transition curves," *Journal of Surveying Engineering*, 140(1), 12–21,
  2014.

## License

MIT, see `LICENSE` in the repository root.
