"""Makes the app's map and its depth grid from the Norvasundet twin's open data (not needed to run the app):
map_light.jpg, map_dark.jpg, depth.bin (uint8, 0.1 m a step, 0 on land, row 0 at the north edge).

Terrain: (c) Kartverket, hoydedata.no (CC BY 4.0). Seabed, from the chart's depth data: Sjokart - Dybdedata (c) Kartverket
(CC BY 4.0). Roads: (c) Statens vegvesen, NVDB (NLOD). Buildings: (c) OpenStreetMap contributors (ODbL). The orthophoto the twin
drapes is not open data and is not used here.

    py -3.12 make_map.py <threepp checkout>
"""
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter
from scipy.ndimage import gaussian_filter

REPO = sys.argv[1]
TWIN = os.path.join(REPO, "python", "examples", "twins", "norvasundet")
GEO = os.path.join(REPO, "geodata", "norvasundet")
HERE = os.path.dirname(os.path.abspath(__file__))
E0, E1, N0, N1 = -240.0, 260.0, -125.0, 155.0        # m east and north of the twin's origin: the map's edges
PX = 4.0                                             # pixels a metre
SS = 2                                               # supersampling of the vector layers

site = json.load(open(os.path.join(TWIN, "norvasundet_site.json"), encoding="utf-8"))["ground"]
z = np.load(os.path.join(TWIN, "norvasundet_ground.npz"))
g, sea = z["ground"], z["sea"]


def crop(a, step):
    """a at every `step` m over the map, row 0 at the north edge."""
    es = np.arange(E0 + step / 2, E1, step)
    ns = np.arange(N1 - step / 2, N0, -step)
    c = np.round((es - site["x0"]) / site["res"]).astype(int)
    r = np.round((-ns - site["z0"]) / site["res"]).astype(int)
    return a[np.ix_(r, c)]


# the depth grid the app steers by
dg = crop(g, 2.0)
depth = np.where(crop(sea, 2.0), np.clip(-dg, 0.0, 25.5), 0.0)
np.round(depth * 10.0).astype(np.uint8).tofile(os.path.join(HERE, "depth.bin"))
print("depth.bin", depth.shape, "water", (depth > 0).mean())

W, H = int((E1 - E0) * PX), int((N1 - N0) * PX)
g1 = np.asarray(Image.fromarray(crop(g, 1.0).astype(np.float32)).resize((W, H), Image.BICUBIC))
s1 = np.asarray(Image.fromarray((crop(sea, 1.0) * 255).astype(np.uint8)).resize((W, H), Image.BICUBIC).filter(ImageFilter.GaussianBlur(7))) / 255.0
s1 = np.clip((s1 - 0.5) * 6.0 + 0.5, 0.0, 1.0)       # a clean shore from the smoothed 1 m mask
gs = gaussian_filter(g1, 10.0)
gy, gx = np.gradient(gs, 1.0 / PX)
light = np.clip(0.62 + 0.3 * (gx - gy) / np.sqrt(1.0 + 0.2 * (gx * gx + gy * gy)), 0.0, 1.0)      # lit from the north-west
d1 = np.clip(-g1, 0.0, 30.0)


def tp(p):
    return [((x - E0) * PX * SS, (N1 + z_) * PX * SS) for x, z_ in p]


def rgb(h):
    return np.array([(h >> 16) & 255, (h >> 8) & 255, h & 255], float)


buildings = json.load(open(os.path.join(GEO, "buildings.json"), encoding="utf-8"))["buildings"]
roads = json.load(open(os.path.join(GEO, "roads.json"), encoding="utf-8"))["roads"]
chart = json.load(open(os.path.join(GEO, "chart.json"), encoding="utf-8"))
print(list(buildings[0].keys()), list(roads[0].keys()))


def outline(b):
    for k in ("outer", "footprint", "outline", "points", "polygon"):
        if k in b:
            return b[k]
    raise KeyError(list(b.keys()))


def make(name, water0, water1, land0, land1, contour, shore, bld, bld_edge, road, casing, pier):
    t = np.clip(d1 / 8.0, 0.0, 1.0)[..., None]
    wat = rgb(water0) * (1 - t) + rgb(water1) * t
    # depth curves, each 2 m
    band = np.floor(d1 / 2.0)
    edge = (band != np.roll(band, 1, 0)) | (band != np.roll(band, 1, 1))
    edge = np.asarray(Image.fromarray((edge * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.8))) / 255.0
    wat = wat * (1 - 0.5 * edge[..., None]) + rgb(contour) * 0.5 * edge[..., None]
    lnd = rgb(land0) * (1 - light[..., None]) + rgb(land1) * light[..., None]
    a = s1[..., None]
    img = lnd * (1 - a) + wat * a
    sh = (np.abs(s1 - 0.5) < 0.3)[..., None]
    img = np.where(sh, img * 0.4 + rgb(shore) * 0.6, img)
    base = Image.fromarray(img.astype(np.uint8)).resize((W * SS, H * SS), Image.BICUBIC)
    d = ImageDraw.Draw(base)
    for r in roads:
        pts = r.get("points") or r.get("centerline") or r.get("line")
        if pts is None:
            continue
        p = tp([(q[0], q[-1]) for q in pts])
        wd = 0.75 * float(r.get("width", 5.0)) * PX * SS
        d.line(p, fill=tuple(rgb(casing).astype(int)), width=int(wd + 3 * SS), joint="curve")
    for r in roads:
        pts = r.get("points") or r.get("centerline") or r.get("line")
        if pts is None:
            continue
        p = tp([(q[0], q[-1]) for q in pts])
        wd = 0.75 * float(r.get("width", 5.0)) * PX * SS
        d.line(p, fill=tuple(rgb(road).astype(int)), width=int(wd), joint="curve")
    for key in ("jetty", "quay", "mole"):
        for f in chart[key]:
            for part in f["points"]:
                d.line(tp(part), fill=tuple(rgb(pier).astype(int)), width=int(1.2 * PX * SS), joint="curve")
    for b in buildings:
        fp = outline(b)
        p = tp([(q[0], q[-1]) for q in fp])
        if len(p) >= 3:
            d.polygon(p, fill=tuple(rgb(bld).astype(int)), outline=tuple(rgb(bld_edge).astype(int)))
    base = base.resize((W, H), Image.LANCZOS)
    base.save(os.path.join(HERE, name), quality=86, optimize=True)
    print(name, os.path.getsize(os.path.join(HERE, name)) // 1024, "kB")


make("map_light.jpg", 0xd3e6f0, 0x8fbdd8, 0xc9cdbd, 0xf6f5ec, 0x5f93b5, 0x6d7a78, 0xb7ae9e, 0x8a8275, 0xffffff, 0xb9b6a8, 0x5e6a70)
make("map_dark.jpg", 0x173047, 0x0b1b2b, 0x171b20, 0x2c323a, 0x2d5674, 0x4d5a64, 0x3b424d, 0x59616d, 0x3e4651, 0x262c34, 0x77828c)
