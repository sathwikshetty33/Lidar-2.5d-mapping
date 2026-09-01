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
CACHE_V = 3            # bump when the frame payload changes shape, so stale
                       # cache entries are rebuilt instead of silently served

# the left colour camera that rode along with the laser. it is FORWARD ONLY,
# about 90 degrees wide, while the map is the full 360 -- so the photo shows
# roughly the top-right quadrant of the map, not all of it.
CAM = 'https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_color.zip'
CALIB = 'https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip'
_calib: dict[str, dict] = {}
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


def cell_provenance(m, x, y, prov):
    """
    the detector's own verdict per cell, not per point.

    a cell that contains a car or a person takes that, whatever else is in it --
    same priority rule the class histogram uses, and for the same reason. what
    is left over resolves by majority, which is how "the network rejected this"
    stays distinguishable from "the network never saw this".
    """
    ix = np.floor(x / g.res0).astype(np.int64)
    iy = np.floor(y / g.res0).astype(np.int64)
    lv = g.blocklevel(ix, iy)
    pk = ((lv << 62) | (((ix >> lv) & 0x7fffffff) << 31) | ((iy >> lv) & 0x7fffffff))
    ck = ((m['lvl'].astype(np.int64) << 62)
          | ((m['ix'] & 0x7fffffff) << 31) | (m['iy'] & 0x7fffffff))
    cid = np.searchsorted(ck, pk)
    h = np.zeros((len(ck), 6), np.int64)
    np.add.at(h, (cid, prov), 1)
    out = h.argmax(1).astype(np.uint8)
    for critical in (2, 3, 4):                 # car, pedestrian, cyclist
        out[h[:, critical] > 0] = critical
    return out


def calib(seq: str):
    """
    where the camera points and how wide it sees, in the laser's own frame.

    all four KITTI cameras are one forward-facing stereo rig -- they differ
    only by a sideways offset of up to 54 cm -- so there is no rear or side
    view to be had. this exists to draw the slice the photo DOES cover onto
    the map, rather than describing it in a caption.
    """
    if seq in _calib:
        return _calib[seq]
    import zipfile, urllib.request
    p = RAW / 'calib.zip'
    if not p.exists() or not p.stat().st_size:
        p.write_bytes(urllib.request.urlopen(CALIB, timeout=120).read())
    with zipfile.ZipFile(p) as z:
        txt = z.read(f'dataset/sequences/{seq}/calib.txt').decode()
    v = {}
    for line in txt.strip().splitlines():
        k, rest = line.split(':', 1)
        v[k.strip()] = np.array([float(x) for x in rest.split()])
    fx = v['P2'].reshape(3, 4)[0, 0]
    fov = float(2*np.degrees(np.arctan(1241/(2*fx))))
    R = v['Tr'].reshape(3, 4)[:, :3]
    fwd = R.T @ np.array([0.0, 0.0, 1.0])        # camera +z, in laser coords
    yaw = float(np.degrees(np.arctan2(fwd[1], fwd[0])))
    _calib[seq] = dict(fov=round(fov, 1), yaw=round(yaw, 1))
    return _calib[seq]


def image(seq: str, frame: str):
    """
    the camera frame that goes with this sweep, pulled the same way as the
    scan. cheaper than the scan (~850 KB, ~5 s) but the first call pays a
    one-off ~50 s to read that archive's 87k-entry directory.
    """
    p = RAW / f'{seq}_{frame}_cam.png'
    if p.exists() and p.stat().st_size:
        return p
    with _borrow(CAM) as z:
        data = z.read(f'dataset/sequences/{seq}/image_2/{frame}.png')
    p.write_bytes(data)
    return p


def project(seq, xyz, lab, w=1241, h=376):
    """
    put the laser points onto the camera image.

    velodyne -> rectified camera (Tr) -> pixels (P2), then keep only what is
    in front of the lens and inside the frame. that is about 15% of a sweep:
    the camera sees 82 degrees of the laser's 360.
    """
    import zipfile
    with zipfile.ZipFile(RAW / 'calib.zip') as z:
        txt = z.read(f'dataset/sequences/{seq}/calib.txt').decode()
    V = {}
    for line in txt.strip().splitlines():
        k, r = line.split(':', 1)
        V[k.strip()] = np.array([float(x) for x in r.split()])
    P2, Tr = V['P2'].reshape(3, 4), V['Tr'].reshape(3, 4)
    cam = (Tr @ np.c_[xyz, np.ones(len(xyz))].T).T
    hom = (P2 @ np.c_[cam, np.ones(len(cam))].T).T
    d = hom[:, 2]
    ok = d > 0.1
    u = np.where(ok, hom[:, 0] / np.where(ok, d, 1), -1)
    v = np.where(ok, hom[:, 1] / np.where(ok, d, 1), -1)
    m = ok & (u >= 0) & (u < w) & (v >= 0) & (v < h)
    return dict(w=w, h=h, n=int(m.sum()),
                u=base64.b64encode(u[m].astype(np.uint16).tobytes()).decode(),
                v=base64.b64encode(v[m].astype(np.uint16).tobytes()).decode(),
                cls=base64.b64encode(lab[m].astype(np.uint8).tobytes()).decode())


def _safe_project(seq, xyz, lab):
    try:
        calib(seq)                      # makes sure calib.zip is on disk
        return project(seq, xyz, lab)
    except Exception:
        return None


def _safe_calib(seq):
    try:
        return calib(seq)
    except Exception:
        return None


def prefetch_image(seq: str, frame: str):
    try:
        image(seq, frame)
    except Exception:
        pass          # no photo is not an error; the map stands on its own


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
        if d.get('v') == CACHE_V:
            d['cached'] = True
            return d

    t0 = time.perf_counter()
    binp, labp, fetched = fetch(seq, frame, want_truth=(source == 'truth'))
    # if a prefetch thread already downloaded it, credit that thread's time
    t_fetch = max(time.perf_counter() - t0,
                  _fetch_ms.pop((seq, frame), 0) / 1000)

    pts4 = np.fromfile(binp, np.float32).reshape(-1, 4)
    info, prov = {}, None
    t0 = time.perf_counter()
    if source == 'truth':
        if labp is None:
            raise ValueError(f'no ground-truth labels published for sequence {seq}')
        _, lab, _ = kitti.load(str(binp), str(labp))
    else:
        import predict
        m_, cfg = model()
        lab, info, prov = predict.predict(pts4, m_, cfg, with_prov=True)
    t_label = time.perf_counter() - t0

    x, y, z = (pts4[:, i].astype(float) for i in range(3))
    k = np.hypot(x, y) < g.maxrange
    x, y, z, lab = x[k], y[k], z[k], lab[k].astype(np.int64)

    t0 = time.perf_counter()
    m = g.build(np.stack([x, y, z], 1), lab)
    t_grid = time.perf_counter() - t0

    extra = {}
    if prov is not None:
        extra['det'] = cell_provenance(m, x, y, prov[k])

    t0 = time.perf_counter()
    srf = surface_json(m, x, y, z, lab, mult=SURF_MULT, quiet=True, extra=extra)
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
        provcount=info.get('provcount', {}),
        clusters=info.get('clusters', 0),
        cars=info.get('counts', {}).get('Car', 0),
        vru=(info.get('counts', {}).get('Pedestrian', 0)
             + info.get('counts', {}).get('Cyclist', 0)),
        ms=dict(fetch=round(t_fetch*1000), label=round(t_label*1000),
                grid=round(t_grid*1000), surface=round(t_surf*1000)),
        fetched=fetched, cached=False,
        v=CACHE_V,
        cam=_safe_calib(seq),
        proj=_safe_project(seq, np.stack([x, y, z], 1), lab),
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
