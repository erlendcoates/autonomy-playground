# State Observer

An interactive, browser-only app for estimating what you cannot measure: the velocity of a ball on a beam, from a
noisy distance sensor. Three estimates run side by side on the same measurements, and you choose which one the
controller uses.

**Live page:** https://erlendcoates.github.io/autonomy-playground/state-observer/

**Video:** [State Observers Explained: Estimating Velocity on a Ball and Beam](https://youtu.be/101cJb9GJKM), and
next, [The Kalman Filter Explained with a Ball and Beam](https://youtu.be/K1OMj2yAJLA). Playlists: [Kalman
Filters](https://www.youtube.com/playlist?list=PLfGhcnOuoBes) and [Ball and
Beam](https://www.youtube.com/playlist?list=PLaLSHgFFcrB4).

## What it shows

- **The rig.** A ball and beam held at a desired position r by a PD controller, `θ = −k_p (p − r) − k_d v`. The
  true ball runs the nonlinear model (`g sin θ`, the centrifugal term, the servo's lag and rate limit and its
  linkage) at 1 kHz; the distance sensor measures the position p every `Δt = 20 ms` with noise σ.
- **Three velocity estimates.**
  - *Finite difference*, `(y_k − y_{k−1}) / Δt`: noisy, `√2 σ/Δt`.
  - *Filtered derivative*, `ω_c s / (s + ω_c)`: smoother, but it lags by about `1/ω_c`.
  - *Luenberger observer* on the linear model `ẍ = b θ` (`b = 5g/7`), discretized with forward Euler:
    `x̂_{k+1} = A_d x̂_k + B_d u_k + L (y_k − C x̂_k)`. Smooth and with little lag, because the model knows how
    the tilt accelerates the ball.
- **The gains.** The eigenvalues of `A_d − LC` are drawn in the unit circle; inside, every initial error dies out.
  With `L = 0` the observer is a simulator, and its error grows: the ball and beam is unstable on its own.

The gauge on the rig shows the true velocity and the three estimates live; the plots show velocity, position and
the beam angle over the last 10 s, and the table the rms error and lag of each estimate.

## Controls

Choose which estimate feeds the controller; set the sensor noise, the filter's cut-off and the observer's gains
(or pick the video's gains and their variants); open the observer's loop; start the observer in the wrong place;
drag the ▲ under the beam to move r. On a phone or tablet, drag the ▲ sideways; a swipe anywhere else scrolls the
page, and the rig stays at the top while the controls and plots scroll under it.

## Running it

Plain HTML and JavaScript modules, using `../common/style.css` and `../common/plot.js`. Serve the repository with
any static server (see the repository's README); modules do not load from `file://`.

## References

- D. G. Luenberger, "Observing the state of a linear system," *IEEE Transactions on Military Electronics*, 8(2),
  74–80, 1964.
- T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley, 2021.

## License

MIT, see `LICENSE` in the repository root.
