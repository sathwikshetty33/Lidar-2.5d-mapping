"""ground-truth validation for the adaptive 2.5d grid builder."""

import time
import numpy as np
import scene
import grid25 as g

pts, lab = scene.make()
m = g.build(pts, lab)

ok = lambda a, b, t: 'ok' if abs(a - b) < t else 'FAIL'

print('=' * 58)
print('recovered geometry vs known truth')
print('=' * 58)

k = (np.abs(m['cy'] - 4.0) < 0.6) & (np.abs(m['cx']) < 8)
v = np.percentile(m['step'][k], 90)
print(f'kerb height       {v:+.3f}  truth {scene.kerb_h:+.3f}  '
      f'{ok(v, scene.kerb_h, 0.04)}')

v = m['dip'][np.hypot(m['cx'] - 12, m['cy']) < 0.3].min()
print(f'pothole depth     {v:+.3f}  truth {-scene.hole_d:+.3f}  '
      f'{ok(v, -scene.hole_d, 0.06)}')

k = (np.abs(m['cx'] - 30) < 0.6) & (np.abs(m['cy']) < 6) & np.isfinite(m['clear'])
v = np.median(m['clear'][k])
print(f'gantry clearance  {v:+.3f}  truth {scene.gantry_z:+.3f}  '
      f'{ok(v, scene.gantry_z, 0.15)}')

print()
print('=' * 58)
print('traversability  (expected in brackets)')
print('=' * 58)
cases = [
    ('open road', (np.abs(m['cy']) < 3.5) & (m['cx'] > 2) & (m['cx'] < 60) &
     (np.hypot(m['cx'] - 12, m['cy']) > 1.5), 'high'),
    ('under gantry', (np.abs(m['cx'] - 30) < 0.6) & (np.abs(m['cy']) < 3), 'high'),
    ('pothole', np.hypot(m['cx'] - 12, m['cy']) < 0.3, 'zero'),
    ('pedestrian', (np.abs(m['cx'] - 40) < 0.5) & (np.abs(m['cy'] - 1.5) < 0.5), 'zero'),
    ('parked car', (np.abs(m['cx'] - 22) < 1) & (np.abs(m['cy'] + 8) < 1), 'zero'),
    ('wall', (np.abs(np.abs(m['cy']) - 14) < 0.15) & (m['ng'] < 3), 'low'),
]
for nm, msk, exp in cases:
    print(f'{nm:<14} {100 * m["trav"][msk].mean():5.1f} %   ({exp}, '
          f'{msk.sum()} cells)')

print()
print('=' * 58)
print('free space  (a ray got under the cell and carried on)')
print('=' * 58)
swept = (m['zray'] - m['zg']) < 2.2
for nm, msk, exp in [
    ('under gantry', (np.abs(m['cx'] - 30) < 0.6) & (np.abs(m['cy']) < 3), 'all'),
    ('tall wall', (np.abs(np.abs(m['cy']) - 14) < 0.15) & (m['obsh'] > 2.5), 'none'),
]:
    print(f'{nm:<14} {100 * swept[msk].mean():5.1f} % swept   ({exp}, '
          f'{msk.sum()} cells)')

print()
print('=' * 58)
print('resolution tiers')
print('=' * 58)
for lv in range(4):
    k = m['lvl'] == lv
    if not k.any():
        continue
    r = np.hypot(m['cx'][k], m['cy'][k])
    print(f'level {lv}  {g.res0 * 2 ** lv:5.2f} m  {k.sum():7d} cells  '
          f'range {r.min():5.1f} - {r.max():5.1f} m')

s = g.memstats(m)
print()
print('=' * 58)
print('memory  (be honest about which factor does the work)')
print('=' * 58)
print(f"uniform dense {g.res0 * 100:.0f}cm grid   {s['uniform']:>10d} cells")
print(f"sparse, occupied only    {s['fine']:>10d} cells   "
      f"{s['uniform'] / s['fine']:6.0f}x  <- from sparsity")
print(f"adaptive (this)          {s['cells']:>10d} cells   "
      f"{s['fine'] / s['cells']:6.2f}x  <- from foveation")
print(f"combined                                       {s['ratio']:6.0f}x")

g.build(pts, lab)
t = []
for _ in range(5):
    t0 = time.perf_counter()
    g.build(pts, lab)
    t.append(time.perf_counter() - t0)
print()
print(f'build {min(t) * 1000:.0f} ms for {len(pts)} points '
      f'({1 / min(t):.0f} Hz, pure numpy)')
