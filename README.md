# Autonomy Playground

Interactive apps for the video lessons on guidance, navigation and control of autonomous vehicles. Each app lets
you try what a video shows: drag, change a gain, run it again, and watch what happens.

**Live pages:** https://erlendcoates.github.io/autonomy-playground/

**Videos:** https://www.youtube.com/@ErlendCoates

| Folder | App | Video |
|---|---|---|
| [`los-guidance/`](los-guidance/) | LOS Guidance: line-of-sight path following on a waypoint path, with a projected or a propagated reference point | [Line-of-Sight (LOS) Guidance Explained](https://youtu.be/e-B-lBFzHRs) |
| [`state-observer/`](state-observer/) | State Observer: velocity from a noisy distance sensor on a ball and beam, by finite differences, a filtered derivative or a Luenberger observer | [State Observers Explained](https://youtu.be/101cJb9GJKM) |

## Running them

Plain HTML, CSS and JavaScript: no build step and no dependencies apart from a web font. Serve the folder with any
static web server (the apps load JavaScript modules, which browsers do not load from `file://`), for example

```
python -m http.server 8000
```

and browse to http://localhost:8000/. `common/` holds the shared style (`style.css`) and a small canvas plotter
(`plot.js`).

## References

- M. Breivik and T. I. Fossen, "Principles of guidance-based path following in 2D and 3D," *Proc. 44th IEEE
  Conference on Decision and Control*, 2005.
- T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley, 2021.
- A. Kobryń, "New solutions for general transition curves," *Journal of Surveying Engineering*, 140(1), 12–21,
  2014.
- D. G. Luenberger, "Observing the state of a linear system," *IEEE Transactions on Military Electronics*, 8(2),
  74–80, 1964.

## License

MIT, see `LICENSE`.
