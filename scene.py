"""
synthetic lidar scene with known ground truth, for checking the converter.

contains, at measured heights:
  - sloped ground, 1% grade
  - a 15cm kerb at y = +4
  - a 25cm deep pothole at (12, 0), radius 0.8
  - building walls at y = +/-14
  - poles every 20m
  - a pedestrian at 40m
  - an overhanging gantry at x = 30, 4.0m clear
"""

import numpy as np
from grid25 import gnd, road, bldg, pole, veg, car, ped

rng = np.random.default_rng(0)

kerb_h = 0.15
hole_d = 0.25
gantry_z = 4.0
ped_x = 40.0


def _n(k, s=0.01):
    return rng.normal(0, s, k)


def make(npts=180000):
    x, y, z, l = [], [], [], []

    def add(px, py, pz, cls):
        x.append(px); y.append(py); z.append(pz)
        l.append(np.full(len(px), cls))

    # ---- road surface, 1% grade, denser near the sensor
    k = int(npts * 0.55)
    r = np.abs(rng.normal(0, 28, k)) + 1.0
    r = r[r < 95]
    th = rng.uniform(-np.pi, np.pi, len(r))
    rx, ry = r * np.cos(th), r * np.sin(th)
    m = np.abs(ry) < 14
    rx, ry = rx[m], ry[m]
    rz = 0.01 * rx + _n(len(rx))

    # kerb: everything beyond y=+4 sits 15cm higher
    rz = rz + np.where(ry > 4.0, kerb_h, 0.0)

    # pothole
    d = np.hypot(rx - 12.0, ry - 0.0)
    rz = rz - np.where(d < 0.8, hole_d * (1 - d / 0.8), 0.0)

    add(rx, ry, rz, np.where(np.abs(ry) < 4.0, road, gnd))

    # ---- walls
    k = int(npts * 0.2)
    wx = rng.uniform(-95, 95, k)
    wy = np.where(rng.random(k) < 0.5, -14.0, 14.0) + _n(k, 0.05)
    wz = rng.uniform(0, 8, k) + 0.01 * wx
    add(wx, wy, wz, bldg)

    # ---- poles
    px_, py_, pz_ = [], [], []
    for cx in np.arange(10, 95, 20):
        for cy in (-6.0, 6.0):
            k = 700
            a = rng.uniform(0, 2 * np.pi, k)
            px_.append(cx + 0.12 * np.cos(a))
            py_.append(cy + 0.12 * np.sin(a))
            pz_.append(rng.uniform(0, 5.5, k) + 0.01 * cx)
    add(np.concatenate(px_), np.concatenate(py_), np.concatenate(pz_), pole)

    # ---- pedestrian at 40m, ~1.7m tall, 0.4m wide
    k = 260
    add(ped_x + rng.normal(0, 0.18, k), rng.normal(1.5, 0.18, k),
        rng.uniform(0, 1.7, k) + 0.01 * ped_x, ped)

    # ---- overhanging gantry: 4.0m clear, spanning the road at x=30
    k = 1400
    gx = 30.0 + rng.normal(0, 0.25, k)
    gy = rng.uniform(-8, 8, k)
    gz = gantry_z + rng.uniform(0, 0.5, k) + 0.01 * 30.0
    add(gx, gy, gz, bldg)

    # ---- a parked car
    k = 1800
    add(rng.uniform(20, 24, k), rng.uniform(-9, -7, k),
        rng.uniform(0, 1.5, k) + 0.01 * 22, car)

    # ---- roadside vegetation
    k = int(npts * 0.06)
    vx = rng.uniform(-90, 90, k)
    vy = np.where(rng.random(k) < 0.5, -1, 1) * rng.uniform(10, 13, k)
    add(vx, vy, rng.uniform(0.3, 3.5, k) + 0.01 * vx, veg)

    pts = np.stack([np.concatenate(x), np.concatenate(y),
                    np.concatenate(z)], 1).astype(np.float32)
    return pts, np.concatenate(l).astype(np.int64)
