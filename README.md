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
(`plot.js`). Each app's folder has its own README, with what it shows, its controls and its references.

## License

MIT, see `LICENSE`.
