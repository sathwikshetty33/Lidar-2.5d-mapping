"""
run every frame in kitti/ through the network and the grid, and emit one
coarse elevation surface per frame for the sequence player.

no ego poses are used, so each frame is its own independent map with the
sensor at the origin -- which is honest: the pipeline is single-frame, there
is no temporal fusion yet, and pretending otherwise would imply an
accumulated map we have not built.

the surface is exported at a 4x coarser base (20/40/80/160 cm instead of
5/10/20/40) purely to fit 11 frames in one page. it is still an exact
coarsening of the same adaptive grid -- every node still resolves to the real
cell that owns it -- just sampled more sparsely.
"""

import io, json, glob, time
import numpy as np
import grid25 as g, kitti, predict
from export_viewer import surface_json, CLASSES

MULT = 4


def main(out='sequence.json'):
    frames = sorted(f for f in glob.glob('kitti/*.bin')
                    if glob.glob(f.replace('.bin', '.label')))
    model, cfg = predict.load()
    predict.predict(np.fromfile(frames[0], np.float32).reshape(-1, 4), model, cfg)

    out_frames, zlo, zhi = [], 1e9, -1e9
    for i, f in enumerate(frames):
        pts4 = np.fromfile(f, np.float32).reshape(-1, 4)
        t0 = time.perf_counter()
        lab, info = predict.predict(pts4, model, cfg)
        tm = (time.perf_counter() - t0) * 1000

        x, y, z = (pts4[:, j].astype(float) for j in range(3))
        k = np.hypot(x, y) < g.maxrange
        x, y, z, lab = x[k], y[k], z[k], lab[k].astype(np.int64)
        t0 = time.perf_counter()
        m = g.build(np.stack([x, y, z], 1), lab)
        tg = (time.perf_counter() - t0) * 1000

        srf = surface_json(m, x, y, z, lab, mult=MULT, quiet=True)
        zlo = min(zlo, srf['zlo']); zhi = max(zhi, srf['zhi'])
        s = g.memstats(m)
        out_frames.append(dict(
            name=f.split('/')[-1].replace('.bin', ''),
            tiers=srf['tiers'],
            npts=int(k.sum()), ncells=int(len(m['n'])), fine=int(s['fine']),
            drivable=round(100 * float(m['trav'].mean()), 1),
            clusters=info['clusters'], cars=info['counts']['Car'],
            vru=info['counts']['Pedestrian'] + info['counts']['Cyclist'],
            ground=info['ground'], ms_model=round(tm), ms_grid=round(tg),
            clscount=[int(c) for c in np.bincount(m['cls'], minlength=8)]))
        print(f'  {out_frames[-1]["name"]}  {k.sum():>7,} pts -> '
              f'{len(m["n"]):>6,} cells  drivable {out_frames[-1]["drivable"]:5.1f}%  '
              f'{info["clusters"]:>3} clusters, {info["counts"]["Car"]:>2} cars  '
              f'{tm:.0f}+{tg:.0f} ms')

    doc = dict(frames=out_frames, zlo=zlo, zhi=zhi, classes=CLASSES,
               res0=g.res0 * MULT, bounds=list(g.bounds), maxrange=g.maxrange,
               source='SemanticKITTI seq 00, labels from the PointNet detector',
               mult=MULT)
    io.open(out, 'w').write(json.dumps(doc, separators=(',', ':')))
    print(f'\n{len(out_frames)} frames -> {out}  '
          f'{len(io.open(out).read())/1e6:.2f} MB')


if __name__ == '__main__':
    main()
