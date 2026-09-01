"""
one frame, end to end: fetch -> label -> grid -> surface.

nothing here is precomputed. a frame is pulled from the remote SemanticKITTI
archives on demand (a few MB of HTTP range reads, not the 80 GB zip), labelled
by the network, converted by grid25, and reduced to a surface the browser can
draw. results are cached on disk so a second request for the same frame is
instant, but the cache is an optimisation, not the source of truth.
"""

from __future__ import annotations

import base64, io, json, os, queue, sys, threading, time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import grid25 as g            # noqa: E402
import kitti                  # noqa: E402
import fetch_kitti as FK      # noqa: E402
from export_viewer import surface_json, CLASSES   # noqa: E402

RAW = ROOT / 'cache' / 'raw'
OUT = ROOT / 'cache' / 'frames'
RAW.mkdir(parents=True, exist_ok=True)
OUT.mkdir(parents=True, exist_ok=True)

SURF_MULT = 4          # surface base 20/40/80/160 cm, keeps a frame ~400 KB
_model_lock = threading.Lock()
_model = None
# a pool rather than one shared handle: ZipFile is not thread safe, and
# building one costs a read of a 44k-entry central directory, so they are
# reused rather than recreated. fetches can then overlap.
_pool: dict[str, queue.Queue] = {}
_pool_lock = threading.Lock()
POOL_MAX = 3

# how long each frame's download actually took, recorded by whichever thread
# did it. without this the reported fetch time is 0 for every prefetched
# frame -- true of the build() call, but a lie about the pipeline.
_fetch_ms: dict[tuple, int] = {}


def model():
    """load the detector once, on first use."""
    global _model
    with _model_lock:
        if _model is None:
            import predict
            _model = predict.load()
        return _model


class _borrow:
    """lend a ZipFile from the pool for the duration of one read."""

    def __init__(self, url):
        self.url = url

    def __enter__(self):
        with _pool_lock:
            q = _pool.setdefault(self.url, queue.Queue())
        try:
            self.z = q.get_nowait()
        except queue.Empty:
            import zipfile
            self.z = zipfile.ZipFile(FK.httpfile(self.url))
        return self.z

    def __exit__(self, *exc):
        q = _pool[self.url]
        if q.qsize() < POOL_MAX:
            q.put(self.z)


def fetch(seq: str, frame: str, want_truth: bool):
    """make sure the raw files exist locally; pull them if not."""
    b = RAW / f'{seq}_{frame}.bin'
    lb = RAW / f'{seq}_{frame}.label'
    got = []
    t0 = time.perf_counter()
    if not b.exists() or b.stat().st_size == 0:
        with _borrow(FK.VEL) as z:
            data = z.read(f'dataset/sequences/{seq}/velodyne/{frame}.bin')
        b.write_bytes(data)          # read fully first: a failed read must not
        got.append('velodyne')       # leave a 0-byte file behind
    if want_truth and (not lb.exists() or lb.stat().st_size == 0):
        try:
            with _borrow(FK.LAB) as z:
                data = z.read(f'dataset/sequences/{seq}/labels/{frame}.label')
            lb.write_bytes(data)
            got.append('labels')
        except KeyError:
            pass                     # sequences 11-21 have no public labels
    if got:
        _fetch_ms[(seq, frame)] = round((time.perf_counter() - t0) * 1000)
    return b, (lb if lb.exists() else None), got


def prefetch(seq: str, frame: str, want_truth: bool):
    """pull the raw files only. safe to run several of these at once."""
    try:
        fetch(seq, frame, want_truth)
    except Exception:
        pass          # the worker will retry and report properly


def build(seq: str, frame: str, source: str = 'model'):
    """
    source 'model'  -> labels from the PointNet detector
           'truth'  -> SemanticKITTI ground truth
    returns the dict the browser draws.
    """
    key = OUT / f'{seq}_{frame}_{source}.json'
    if key.exists():
        d = json.loads(key.read_text())
        d['cached'] = True
        return d

    t0 = time.perf_counter()
    binp, labp, fetched = fetch(seq, frame, want_truth=(source == 'truth'))
    # if a prefetch thread already downloaded it, credit that thread's time
    t_fetch = max(time.perf_counter() - t0,
                  _fetch_ms.pop((seq, frame), 0) / 1000)

    pts4 = np.fromfile(binp, np.float32).reshape(-1, 4)
    info = {}
    t0 = time.perf_counter()
    if source == 'truth':
        if labp is None:
            raise ValueError(f'no ground-truth labels published for sequence {seq}')
        _, lab, _ = kitti.load(str(binp), str(labp))
    else:
        import predict
        m_, cfg = model()
        lab, info = predict.predict(pts4, m_, cfg)
    t_label = time.perf_counter() - t0

    x, y, z = (pts4[:, i].astype(float) for i in range(3))
    k = np.hypot(x, y) < g.maxrange
    x, y, z, lab = x[k], y[k], z[k], lab[k].astype(np.int64)

    t0 = time.perf_counter()
    m = g.build(np.stack([x, y, z], 1), lab)
    t_grid = time.perf_counter() - t0

    t0 = time.perf_counter()
    srf = surface_json(m, x, y, z, lab, mult=SURF_MULT, quiet=True)
    t_surf = time.perf_counter() - t0

    s = g.memstats(m)
    out = dict(
        seq=seq, frame=frame, source=source,
        tiers=srf['tiers'], zlo=srf['zlo'], zhi=srf['zhi'],
        zglo=float(min(np.frombuffer(base64.b64decode(t['zgnd']), np.int16).min()
                       for t in srf['tiers'])) / 1000,
        zghi=float(max(np.frombuffer(base64.b64decode(t['zgnd']), np.int16).max()
                       for t in srf['tiers'])) / 1000,
        npts=int(k.sum()), ncells=int(len(m['n'])), fine=int(s['fine']),
        uniform=int(s['uniform']),
        drivable=round(100 * float(m['trav'].mean()), 1),
        lvlcount=[int((m['lvl'] == i).sum()) for i in range(4)],
        clscount=[int(c) for c in np.bincount(m['cls'], minlength=8)],
        clusters=info.get('clusters', 0),
        cars=info.get('counts', {}).get('Car', 0),
        vru=(info.get('counts', {}).get('Pedestrian', 0)
             + info.get('counts', {}).get('Cyclist', 0)),
        ms=dict(fetch=round(t_fetch*1000), label=round(t_label*1000),
                grid=round(t_grid*1000), surface=round(t_surf*1000)),
        fetched=fetched, cached=False,
    )
    key.write_text(json.dumps(out, separators=(',', ':')))
    return out


def frame_ids(seq: str, mode: str, start: int, count: int, stride: int, seed: int = 0):
    """sequential = consecutive motion; random = scattered across the sequence."""
    n = SEQ_LEN.get(seq, 1000)
    if mode == 'random':
        rng = np.random.default_rng(seed)
        ids = sorted(rng.choice(n, size=min(count, n), replace=False).tolist())
    else:
        ids = [start + i*stride for i in range(count) if start + i*stride < n]
    return [f'{i:06d}' for i in ids]


# frame counts for the odometry sequences
SEQ_LEN = {'00': 4541, '01': 1101, '02': 4661, '03': 801, '04': 271, '05': 2761,
           '06': 1101, '07': 1101, '08': 4071, '09': 1591, '10': 1201}
