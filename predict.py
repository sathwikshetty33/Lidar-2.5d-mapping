"""
per-point class labels from the pointnet detector in MadhankumarAI/trail,
mapped into grid25's class space.

WHAT THIS MODEL ACTUALLY IS
---------------------------
It is a cluster-wise 3D *detector*, not a semantic segmenter. `ClusterNet`
takes 256 points from one cluster and emits a single label over

    [Background, Car, Pedestrian, Cyclist]

plus a box. It never labels an individual point, and it has no class at all for
road, sidewalk, building, vegetation or pole.

The per-point labels grid25 needs come from the three stages around it:

    remove_ground()   ground / not-ground per point   (geometry, no network)
    cluster_points()  cluster id per non-ground point (geometry, no network)
    ClusterNet        one class per cluster           (the network)

So the network contributes exactly one thing: the car / pedestrian / cyclist
split among the clusters. Everything static and non-ground -- buildings,
vegetation, poles, signs -- has no class to go to and lands in `other`.

WHAT THAT COSTS
---------------
Terrain analysis is unaffected: it only ever needed the ground / not-ground
split, and `remove_ground` supplies that directly. Kerbs, potholes, roughness,
slope and clearance all still work.

The semantic layer gets much coarser than SemanticKITTI ground truth. Road and
sidewalk collapse into one `ground`, and building / vegetation / pole collapse
into `other`. That is a real, visible downgrade and it should be presented as
one -- a 4-class detector cannot produce an 8-class map.

Cyclist maps onto `ped` so that it inherits the pedestrian priority override in
grid25.classify(): a vulnerable road user must not be voted away by a
road-dominated cell.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

import grid25 as g

REPO = Path(__file__).resolve().parent / 'trail'
SRC = REPO / 'pointnet-det' / 'src'
CKPT = REPO / 'best.pt'

# [Background, Car, Pedestrian, Cyclist] -> grid25
DET2GRID = np.array([g.other, g.car, g.ped, g.ped], np.int64)

# what the DETECTOR itself did with each point, which grid25's 8 classes
# cannot express. the important split is the last two: a point in a cluster
# the network examined and rejected is a very different thing from a point
# the network never saw, and both currently land in `other`.
PROV = ['ground', 'background', 'car', 'pedestrian', 'cyclist', 'unclustered']
P_GROUND, P_BG, P_CAR, P_PED, P_CYC, P_NONE = range(6)
DET2PROV = np.array([P_BG, P_CAR, P_PED, P_CYC], np.int64)


def _import():
    if str(SRC) not in sys.path:
        sys.path.insert(0, str(SRC))
    from pnd.config import Config
    from pnd.ground import remove_ground
    from pnd.cluster import cluster_points
    from pnd.bench_canon import pca2_batch
    from pnd.model import build
    return Config, remove_ground, cluster_points, pca2_batch, build


def load(ckpt=CKPT, device='cpu'):
    """load best.pt and rebuild the network it was trained as."""
    import torch
    Config, _, _, _, build = _import()
    ck = torch.load(ckpt, map_location=device, weights_only=False)
    saved = ck.get('cfg', {}) or {}
    cfg = Config()
    for k, v in saved.items():
        if hasattr(cfg, k) and k not in ('device', 'data_root', 'cache_dir', 'run_dir'):
            setattr(cfg, k, v)
    cfg.device = device
    model = build(cfg)
    model.load_state_dict(ck['model'])
    model.eval()
    return model, cfg


def _features(xyz, inten, agl, maxrange, pca2_batch):
    """
    exactly Collate.__call__ with train=False: centre each cluster, remove its
    yaw from a 2D pca of the footprint, scale to the unit sphere, then append
    the three rotation-invariant channels. any deviation here silently feeds
    the network something it was never trained on.
    """
    B, P = xyz.shape[:2]
    rng_raw = np.linalg.norm(xyz, axis=2)          # range BEFORE any rotation
    yaw = np.zeros(B)
    pca2_batch(np.ascontiguousarray(xyz.reshape(-1, 3)),
               (np.arange(B + 1) * P).astype(np.int64), yaw)
    c, s = np.cos(-yaw), np.sin(-yaw)
    Rc = np.zeros((B, 3, 3))
    Rc[:, 0, 0] = c; Rc[:, 0, 1] = -s
    Rc[:, 1, 0] = s; Rc[:, 1, 1] = c
    Rc[:, 2, 2] = 1.0
    tc = xyz.mean(axis=1)
    xc = np.einsum('bij,bpj->bpi', Rc, xyz - tc[:, None, :])
    scale = np.maximum(np.linalg.norm(xc, axis=2).max(axis=1), 1e-6)
    xc = xc / scale[:, None, None]
    return np.concatenate([xc,
                           (agl / 3.0)[:, :, None],
                           (rng_raw / maxrange)[:, :, None],
                           inten[:, :, None]], axis=2).transpose(0, 2, 1)


def predict(pts4, model, cfg, batch=256, seed=0, with_prov=False):
    """
    pts4: (N, 4) velodyne x y z intensity, sensor at the origin.
    returns (labels in grid25 classes, info dict), and with with_prov=True a
    third array saying what the detector actually did with each point (PROV).
    """
    import torch
    _, remove_ground, cluster_points, pca2_batch, _ = _import()
    rng = np.random.default_rng(seed)
    N = len(pts4)
    lab = np.full(N, g.other, np.int64)

    prov = np.full(N, P_NONE, np.int64)
    is_ground, agl, _ = remove_ground(pts4[:, :3], thresh=cfg.ground_thresh)
    lab[is_ground] = g.gnd
    prov[is_ground] = P_GROUND

    fg = np.flatnonzero(~is_ground)
    if len(fg) < cfg.min_cluster_pts:
        empty = dict(ground=int(is_ground.sum()), clusters=0, counts={})
        return (lab, empty, prov) if with_prov else (lab, empty)

    obj = pts4[fg]
    cl = cluster_points(obj[:, :3], voxel=cfg.cluster_voxel,
                        min_points=cfg.min_cluster_pts,
                        max_points=cfg.max_cluster_pts)
    ncl = int(cl.max()) + 1
    if ncl <= 0:
        empty = dict(ground=int(is_ground.sum()), clusters=0, counts={})
        return (lab, empty, prov) if with_prov else (lab, empty)

    # gather cfg.n_points per cluster, padding by resampling as training did
    order = np.argsort(cl, kind='stable')
    cs = cl[order]
    first = np.searchsorted(cs, np.arange(ncl))
    last = np.searchsorted(cs, np.arange(ncl), side='right')
    P = cfg.n_points
    sel = np.empty((ncl, P), np.int64)
    for k in range(ncl):
        idx = order[first[k]:last[k]]
        sel[k] = (rng.choice(idx, P, replace=False) if len(idx) >= P
                  else rng.choice(idx, P, replace=True))

    xyz = obj[sel][:, :, :3].astype(np.float64)
    inten = obj[sel][:, :, 3].astype(np.float64)
    a = agl[fg][sel].astype(np.float64)

    pred = np.empty(ncl, np.int64)
    conf = np.empty(ncl)
    with torch.no_grad():
        for i in range(0, ncl, batch):
            f = _features(xyz[i:i+batch], inten[i:i+batch], a[i:i+batch],
                          cfg.max_range, pca2_batch)
            p = model(torch.from_numpy(f).float())['logits'].softmax(1).numpy()
            pred[i:i+batch] = p.argmax(1)
            conf[i:i+batch] = p.max(1)

    hit = cl >= 0
    lab[fg[hit]] = DET2GRID[pred[cl[hit]]]
    prov[fg[hit]] = DET2PROV[pred[cl[hit]]]

    names = ['Background', 'Car', 'Pedestrian', 'Cyclist']
    out = dict(
        ground=int(is_ground.sum()),
        clusters=ncl,
        clustered_points=int(hit.sum()),
        unclustered=int((~hit).sum()),
        counts={names[i]: int((pred == i).sum()) for i in range(4)},
        provcount={PROV[i]: int((prov == i).sum()) for i in range(6)},
        mean_conf=float(conf.mean()))
    return (lab, out, prov) if with_prov else (lab, out)


if __name__ == '__main__':
    import time, kitti
    src = sys.argv[1] if len(sys.argv) > 1 else 'kitti/000000.bin'
    pts4 = np.fromfile(src, np.float32).reshape(-1, 4)
    model, cfg = load()
    print(f'model  canon={cfg.canon}  classes={cfg.num_classes}  '
          f'in_ch={cfg.in_ch}  params={sum(p.numel() for p in model.parameters()):,}')
    t0 = time.perf_counter()
    lab, info = predict(pts4, model, cfg)
    ms = (time.perf_counter() - t0) * 1000
    print(f'{len(pts4):,} points in {ms:.0f} ms')
    for k, v in info.items():
        print(f'  {k}: {v}')
    names = ['ground', 'road', 'building', 'pole', 'vegetation', 'car', 'ped', 'other']
    print('  per-point labels:', {names[i]: int(c)
                                  for i, c in enumerate(np.bincount(lab, minlength=8)) if c})
