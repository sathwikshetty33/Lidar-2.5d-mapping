"""
run the whole pipeline over every frame in kitti/ and score the network's
labels against SemanticKITTI ground truth, at two levels:

  point level   does it find the ground, the cars, the people
  map level     does the FINISHED 2.5D map differ, and in which direction

the map-level numbers are the ones that matter. the geometry is identical
either way -- labels never move a cell -- so any disagreement is purely the
semantic and terrain layers, and the only column worth worrying about is
"model says drivable, truth says not".
"""

import glob, sys, time
import numpy as np
import grid25 as g, kitti, predict

NAMES = ['ground', 'road', 'building', 'pole', 'vegetation', 'car', 'ped', 'other']


def pr(tp, fp, fn):
    return (tp/(tp+fp) if tp+fp else float('nan'),
            tp/(tp+fn) if tp+fn else float('nan'))


def main():
    frames = sorted(f for f in glob.glob('kitti/*.bin')
                    if glob.glob(f.replace('.bin', '.label')))
    if not frames:
        print('no frames in kitti/ -- run fetch_kitti.py first')
        return
    model, cfg = predict.load(sys.argv[1] if len(sys.argv) > 1 else predict.CKPT)
    print(f'{len(frames)} frames   model params '
          f'{sum(p.numel() for p in model.parameters()):,}\n')

    acc = dict(gtp=0, gfp=0, gfn=0, ctp=0, cfp=0, cfn=0, ptp=0, pfp=0, pfn=0,
               cells=0, agree=0, unsafe=0, cons=0, npts=0,
               tmodel=0.0, tgrid=0.0, dtrue=0, dmodel=0)
    unsafe_cls = np.zeros(8, np.int64)
    rows = []

    predict.predict(np.fromfile(frames[0], np.float32).reshape(-1, 4), model, cfg)

    for f in frames:
        pts4 = np.fromfile(f, np.float32).reshape(-1, 4)
        _, gt, _ = kitti.load(f, f.replace('.bin', '.label'))
        t0 = time.perf_counter()
        lab, info = predict.predict(pts4, model, cfg)
        tm = time.perf_counter() - t0

        x, y, z = (pts4[:, i].astype(float) for i in range(3))
        k = np.hypot(x, y) < g.maxrange
        P = np.stack([x[k], y[k], z[k]], 1)
        t0 = time.perf_counter()
        A = g.build(P, gt[k].astype(np.int64))
        B = g.build(P, lab[k].astype(np.int64))
        tg = (time.perf_counter() - t0) / 2

        gg, mg = np.isin(gt, g.groundcls), np.isin(lab, g.groundcls)
        acc['gtp'] += int((gg & mg).sum()); acc['gfp'] += int((~gg & mg).sum())
        acc['gfn'] += int((gg & ~mg).sum())
        for a, b, key in ((gt == g.car, lab == g.car, 'c'),
                          (gt == g.ped, lab == g.ped, 'p')):
            acc[key+'tp'] += int((a & b).sum()); acc[key+'fp'] += int((~a & b).sum())
            acc[key+'fn'] += int((a & ~b).sum())

        unsafe = B['trav'] & ~A['trav']
        cons = A['trav'] & ~B['trav']
        unsafe_cls += np.bincount(A['cls'][unsafe], minlength=8)
        acc['cells'] += len(A['n']); acc['npts'] += int(k.sum())
        acc['agree'] += int((A['trav'] == B['trav']).sum())
        acc['unsafe'] += int(unsafe.sum()); acc['cons'] += int(cons.sum())
        acc['dtrue'] += int(A['trav'].sum()); acc['dmodel'] += int(B['trav'].sum())
        acc['tmodel'] += tm; acc['tgrid'] += tg

        rows.append((f.split('/')[-1].replace('.bin', ''), int(k.sum()), len(A['n']),
                     info['clusters'], info['counts']['Car'], info['counts']['Pedestrian']
                     + info['counts']['Cyclist'],
                     100*A['trav'].mean(), 100*B['trav'].mean(),
                     100*(A['trav'] == B['trav']).mean(), 100*unsafe.mean(), tm*1000))

    print(f'{"frame":>7} {"points":>8} {"cells":>7} {"clus":>5} {"car":>4} {"vru":>4} '
          f'{"drv-gt":>7} {"drv-md":>7} {"agree":>7} {"unsafe":>7} {"ms":>6}')
    for r in rows:
        print(f'{r[0]:>7} {r[1]:>8,} {r[2]:>7,} {r[3]:>5} {r[4]:>4} {r[5]:>4} '
              f'{r[6]:>6.1f}% {r[7]:>6.1f}% {r[8]:>6.2f}% {r[9]:>6.2f}% {r[10]:>6.0f}')

    n = len(frames)
    print(f'\n=== aggregate over {n} frames, {acc["npts"]:,} points, '
          f'{acc["cells"]:,} cells ===')
    for nm, key in (('GROUND', 'g'), ('CAR', 'c'), ('PED+CYCLIST', 'p')):
        p, r = pr(acc[key+'tp'], acc[key+'fp'], acc[key+'fn'])
        print(f'  {nm:12s} precision {p:.3f}  recall {r:.3f}   '
              f'(truth {acc[key+"tp"]+acc[key+"fn"]:,}, model {acc[key+"tp"]+acc[key+"fp"]:,})')
    print(f'\n  drivable   ground truth {100*acc["dtrue"]/acc["cells"]:.1f} %'
          f'   model {100*acc["dmodel"]/acc["cells"]:.1f} %')
    print(f'  traversability agrees on {100*acc["agree"]/acc["cells"]:.2f} % of cells')
    print(f'    model drivable, truth NOT : {acc["unsafe"]:,} '
          f'({100*acc["unsafe"]/acc["cells"]:.2f} %)  <-- unsafe direction')
    print(f'    truth drivable, model NOT : {acc["cons"]:,} '
          f'({100*acc["cons"]/acc["cells"]:.2f} %)  (conservative, harmless)')
    print('\n  what ground truth says is in the unsafe cells:')
    for i, c in enumerate(unsafe_cls):
        if c:
            print(f'    {NAMES[i]:12s} {c:6,d}  {100*c/unsafe_cls.sum():5.1f} %')
    print(f'\n  timing per frame: network {1000*acc["tmodel"]/n:.0f} ms + '
          f'grid {1000*acc["tgrid"]/n:.0f} ms = {1000*(acc["tmodel"]+acc["tgrid"])/n:.0f} ms'
          f'  ({n/(acc["tmodel"]+acc["tgrid"]):.1f} Hz on this CPU)')


if __name__ == '__main__':
    main()
