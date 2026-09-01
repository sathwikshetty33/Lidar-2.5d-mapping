"""
build viewer_data.json + viewer_surface.json from any (points, labels) frame.

  python export_viewer.py scene                    # the synthetic test scene
  python export_viewer.py kitti/000000.bin         # ground-truth labels
  python export_viewer.py kitti/000000.bin --model # labels from the network

viewer_data   the sparse adaptive cells -- what you would store and transmit
viewer_surface  the dense field those cells describe, one node wherever a
                cell of that tier could exist, for the shaded surface view
"""

import base64, io, json, sys, time
import numpy as np
from scipy import ndimage
import grid25 as g

CLASSES = ['ground', 'road', 'building', 'pole', 'vegetation', 'car', 'pedestrian', 'other']
b64 = lambda a: base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()


def i16(v, s=1000.0, lo=-32000, hi=32000):
    v = np.nan_to_num(np.asarray(v, float) * s, nan=0, posinf=hi, neginf=lo)
    return np.clip(np.round(v), lo, hi).astype(np.int16)


def cells_json(m, ms, npts, source, views, pc):
    q = m['cx'] / 0.025
    assert np.allclose(q, np.round(q)), 'cell centres are not on the 25 mm lattice'
    s = g.memstats(m)
    cells = dict(
        cx=b64(i16(m['cx'], 40)), cy=b64(i16(m['cy'], 40)),
        lvl=b64(m['lvl'].astype(np.uint8)),
        zg=b64(i16(m['zg'])), zmax=b64(i16(m['zmax'])),
        clear=b64(i16(m['clear'], hi=30000)), obsh=b64(i16(m['obsh'])),
        rough=b64(np.clip(np.round(m['rough']*1000), 0, 65000).astype(np.uint16)),
        step=b64(i16(m['step'])), dip=b64(i16(m['dip'])),
        cls=b64(m['cls'].astype(np.uint8)), trav=b64(m['trav'].astype(np.uint8)),
        n=b64(np.clip(m['n'], 0, 65000).astype(np.uint16)))
    meta = dict(source=source, ncells=int(len(m['n'])), npts=int(npts), ms=round(ms, 1),
                res0=g.res0, bounds=list(g.bounds), maxrange=g.maxrange,
                uniform=s['uniform'], fine=s['fine'], adaptive=s['cells'],
                lvlcount=[int((m['lvl'] == i).sum()) for i in range(4)],
                classes=CLASSES,
                clscount=[int(c) for c in np.bincount(m['cls'], minlength=8)],
                travpct=round(100*float(m['trav'].mean()), 1), views=views)
    return dict(meta=meta, cells=cells, pc=pc)


def surface_json(m, x, y, z, lab, mult=1, quiet=False):
    """the dense field, on the same tiers, for the shaded surface view."""
    rast, dist, gox, goy = g.groundmap(x, y, z, lab)
    ckey = ((m['lvl'].astype(np.int64) << 62) |
            ((m['ix'] & 0x7fffffff) << 31) | (m['iy'] & 0x7fffffff))
    xlo, xhi = np.floor(x.min())-1, np.ceil(x.max())+1
    ylo, yhi = np.floor(y.min())-1, np.ceil(y.max())+1
    bnd = list(g.bounds) + [g.maxrange]
    tiers, tot = [], 0
    for t in range(4):
        res = g.res0 * (1 << t) * mult; R = bnd[t]
        x0 = np.floor(max(xlo, -R)/res)*res
        y0 = np.floor(max(ylo, -R)/res)*res
        nx = int(round((min(xhi, R)-x0)/res)) + 1
        ny = int(round((min(yhi, R)-y0)/res)) + 1
        XX, YY = np.meshgrid(x0 + np.arange(nx)*res, y0 + np.arange(ny)*res)
        X, Y = XX.ravel(), YY.ravel()
        lv = np.digitize(np.hypot(X, Y), g.bounds).astype(np.int64)
        nk = ((lv << 62)
              | (((np.floor(X/g.res0).astype(np.int64) >> lv) & 0x7fffffff) << 31)
              | ((np.floor(Y/g.res0).astype(np.int64) >> lv) & 0x7fffffff))
        j = np.minimum(np.searchsorted(ckey, nk), len(ckey)-1)
        hit = (ckey[j] == nk).reshape(ny, nx)
        zgnd = g.sample(rast, gox, goy, X, Y).reshape(ny, nx)
        # how far this node is from a real ground return. beyond gnear the
        # raster is extrapolation, not terrain -- the same rule traversable()
        # uses to refuse to call unknown space drivable. the viewer drops
        # those quads rather than drawing an invented plane.
        gdist = g.sample(dist, gox, goy, X, Y).reshape(ny, nx)
        known = gdist < g.gnear
        top = np.where(hit, m['zmax'][j].reshape(ny, nx), np.nan)
        # a node with no cell is unmeasured, not "at ground level". fill from
        # the nearest real observation as far as one can honestly reach, then
        # fall back to terrain -- otherwise a wall alternates with the road
        # and the surface renders as a comb.
        bad = np.isnan(top)
        d, idx = ndimage.distance_transform_edt(bad, return_indices=True)
        ztop = np.where(d*res <= max(0.6, 4*res), top[tuple(idx)], zgnd)
        ztop = np.maximum(ndimage.median_filter(ztop, 3, mode='nearest'), zgnd)
        cls = np.where(hit, m['cls'][j].reshape(ny, nx), 7).astype(np.uint8)
        flag = (hit.astype(np.uint8)
                | (np.where(hit, m['trav'][j].reshape(ny, nx), 0).astype(np.uint8) << 1)
                | (known.astype(np.uint8) << 2))
        tiers.append(dict(res=res, x0=float(x0), y0=float(y0), nx=nx, ny=ny,
                          rin=0.0 if t == 0 else g.bounds[t-1], rout=float(R),
                          ztop=b64(i16(ztop)), zgnd=b64(i16(zgnd)),
                          cls=b64(cls), flag=b64(flag)))
        tot += nx*ny
        if not quiet:
            print(f'  tier {t} {res*100:4.0f} cm  {nx:4d}x{ny:4d}={nx*ny:8d}  '
                  f'observed {100*hit.mean():4.1f}%  terrain known {100*known.mean():4.1f}%')
    zt = np.concatenate([np.frombuffer(base64.b64decode(t['ztop']), np.int16)
                         for t in tiers]) / 1000
    if not quiet:
        print(f'  {tot:,} nodes')
    return dict(tiers=tiers, zlo=float(np.percentile(zt, .5)),
                zhi=float(np.percentile(zt, 99.5)))


def autoviews(m):
    """a jump-to for the nearest instance of the classes worth looking at."""
    r = np.hypot(m['cx'], m['cy'])
    out = [['Whole scene', None], ['Near field', dict(x=0, y=0, m=14, ry=0)]]
    for cid, nm, span in [(5, 'Nearest car', 10), (6, 'Pedestrian', 8),
                          (3, 'Pole', 8), (2, 'Building', 16)]:
        k = (m['cls'] == cid) & (r > 3)
        if k.sum() < 4:
            continue
        i = np.flatnonzero(k)[np.argmin(r[k])]
        out.append([nm, dict(x=round(float(m['cx'][i]), 1),
                             y=round(float(m['cy'][i]), 1), m=span,
                             ry=round(float(m['cy'][i]), 1))])
    return out


def run(source, use_model=False):
    if source == 'scene':
        import scene
        pts, lab = scene.make()
        name = 'synthetic test scene'
    elif use_model:
        import predict
        pts4 = np.fromfile(source, np.float32).reshape(-1, 4)
        model, cfg = predict.load()
        t0 = time.perf_counter()
        lab, info = predict.predict(pts4, model, cfg)
        print(f'network labels in {(time.perf_counter()-t0)*1000:.0f} ms: '
              f'{info["clusters"]} clusters, {info["counts"]}')
        pts = pts4[:, :3].astype(np.float64)
        name = ('PointNet detector on ' + source.split('/')[-1].replace('.bin', '')
                + ' (predicted labels)')
    else:
        import kitti
        lb = source.replace('.bin', '.label').replace('velodyne', 'labels')
        pts, lab, mov = kitti.load(source, lb)
        name = 'SemanticKITTI ' + source.split('/')[-1].replace('.bin', '')
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    keep = np.hypot(x, y) < g.maxrange
    x, y, z, lab = x[keep], y[keep], z[keep], lab[keep].astype(np.int64)
    pts = np.stack([x, y, z], 1)

    g.build(pts, lab)
    t = [];
    for _ in range(3):
        t0 = time.perf_counter(); m = g.build(pts, lab); t.append(time.perf_counter()-t0)
    ms = min(t)*1000
    print(f'{name}: {len(x):,} points -> {len(m["n"]):,} cells in {ms:.0f} ms')

    # the free-space margin, which on a roof-mounted sensor is nothing like
    # the ground-level synthetic case. this is the number CLAUDE.md says to
    # re-measure before trusting the overhang rescue.
    d = m['zray'] - m['zg']
    fin = np.isfinite(d)
    print(f'  zray - zg over cells with a ray: median {np.median(d[fin]):+.2f} m, '
          f'90th {np.percentile(d[fin], 90):+.2f} m, '
          f'{100*fin.mean():.0f}% of cells have one')

    pc = dict(x=b64(i16(x, 100)), y=b64(i16(y, 100)), z=b64(i16(z, 100)),
              l=b64(lab.astype(np.uint8)))
    io.open('viewer_data.json', 'w').write(json.dumps(
        cells_json(m, ms, len(x), name, autoviews(m), pc), separators=(',', ':')))
    print('surface:')
    io.open('viewer_surface.json', 'w').write(json.dumps(
        surface_json(m, x, y, z, lab), separators=(',', ':')))
    for f in ('viewer_data.json', 'viewer_surface.json'):
        print(f'  {f}  {len(io.open(f).read())/1e6:.2f} MB')


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    run(args[0] if args else 'scene', use_model='--model' in sys.argv)
