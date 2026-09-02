# SIH 2026 — Adaptive Variable-Resolution 2.5D LiDAR Mapping

## What this is

Smart India Hackathon 2026, problem statement **26053**, organisation **DRDO**
(Dept. of Defence Production / IDEX), theme Transportation & Logistics.

Build a pipeline that turns raw LiDAR point clouds into a **variable-resolution
2.5D grid** — an elevation map with semantic layers, where cell size grows with
range from the sensor ("foveated" mapping). Three required capabilities:

1. **Terrain analysis** — drivable vs non-drivable
2. **Object detection** — static (walls, poles) vs dynamic (pedestrians, vehicles)
3. **Adaptive spatial representation** — non-uniform grid, high resolution near,
   coarse far, without alignment errors or data loss during 3D→2.5D projection

Deliverables the PS asks for: the DL model, the grid engine, a real-time
dashboard showing memory reduction, and evidence of low latency / high FPS.

---

## Hardware reality

| Machine | Spec | Used for |
|---|---|---|
| Local | WSL, Ubuntu 26.04 (Resolute), 18 GB RAM, **no GPU** | grid engine dev, ROS nodes, dashboard |
| Remote | Ubuntu + GPU | CARLA server, model training |
| Colab / Kaggle | free GPU | training fallback |

ROS was installed locally (ROS 2 **Lyrical Luth**, matching Ubuntu 26.04) and
then removed. If reinstalling: Lyrical for 26.04, Jazzy only on 24.04. Third-party
packages (`foxglove_bridge`, `grid_map`) may lag Lyrical — build from source or
run in a Jazzy container if a package is missing.

**CARLA cannot run locally** (needs a dedicated 6 GB+ GPU). Run the server on the
remote box in **off-screen** mode (`-RenderOffScreen`, *not* `--no-rendering`,
which returns empty GPU-sensor data). Clients connect over TCP 2000/2001, so ROS
nodes can run anywhere.

---

## Pipeline

```
raw points (N,3)
  → preprocess        deskew, range filter, ego-motion via TF
  → scan buffer       ~5-10 frames, needed for dynamic detection
  → segmentation      per-point class labels
  → labeled points (N,4)      <- SAME geometry, +1 label column
  → grid builder      adaptive 2.5D grid  [grid25.py]
  → AdaptiveGrid msg
       ├→ dashboard (Foxglove)
       └→ planner (thin hybrid A* over traversability)
```

**One stream, not two branches.** The model annotates the points; it does not
consume them. Geometry and labels are aggregated together in a single pass in the
grid builder. Do not re-join labels to points downstream.

**Semantics are data, not decoration.** The cell's class is computed during
aggregation from a class histogram. Colour is only a dashboard lookup at the very
end; the planner reads the class field directly.

### Node boundaries

Preprocess / segmentation / grid builder are logically separate nodes but should
be **composed into one process** (`rclcpp` components, intra-process comms) —
a 2 MB point cloud at 10 Hz cannot afford serialise/deserialise at every hop.
Note `rclpy` does not participate in zero-copy; Python nodes always serialise.

Dashboard and planner stay as separate processes. If the dashboard hangs, the map
must keep building.

### QoS

Heavy topics use **best-effort, keep-last, depth 1** (`qos_profile_sensor_data`).
Reliable + depth 10 silently builds a backlog: every stage reports healthy
per-frame times while the map falls seconds behind. Publish dropped-frame count
to the dashboard — "12 Hz, zero dropped" is a stronger claim than raw FPS.

### Timestamps

**Every stage copies the input header stamp forward, unchanged.** Never restamp.
End-to-end latency = `now - grid.header.stamp` at the dashboard. Restamping
produces a plausible-looking number that is wrong and unfalsifiable.

---

## The grid algorithm (`grid25.py`)

Two passes. This ordering is the whole design, not an implementation detail.

**Pass 1** — quantise every point at the finest resolution (5 cm). **No range
logic at all.**

**Pass 2** — assign each *cell* a level from its centre's range, then merge
children into parents.

### Why per-cell and not per-point

Deciding the tier per point is the obvious approach and it is broken. Two points
1 cm apart straddling the 10 m boundary land in a 5 cm cell and a 40 cm cell that
physically **overlap**. The coarse cell then holds only some of the points in its
own footprint — measured on KITTI frame 0: 80 cells short of their own contents,
the worst seeing 11% of them, and 358 points in the wrong cell. That is exactly
the "alignment errors or data loss" the PS warns about.

### And the tier must come from the BLOCK, not the fine cell

Quantising first fixes the point-level loss but **not** the footprint overlap.
The tier boundary is a circle (`hypot`), the grid is square, so a parent block
can have some children inside the ring and some outside. Take the tier from each
fine cell's own centre and one block splits across two tiers: the inside children
stay fine, the outside ones merge into a parent whose footprint *contains* them
but holds none of their points. That was **82 overlapping footprints** on KITTI
frame 0 — a planner asking "what is at (x,y)?" gets two cells and no rule.

`blocklevel()` decides from the block instead: coarsen only if the **whole**
block lies beyond the boundary; if any part reaches into the finer ring, the
block stays fine. Constant per block by construction, so nesting cannot occur.
Verified 0 overlaps on 7 frames. Costs +0.15% cells (48,837 -> 48,909) and
resolution never degrades across a boundary, which is the safe direction.

Note points were always partitioned correctly — total `n` equals the input count
either way. The overlap was between *footprints*, not point sets.

### Power-of-two tiers

5 / 10 / 20 / 40 cm, not the PS's literal 5 → 50 cm. The PS says "e.g.", and 10×
does not nest. With powers of two, the parent index is an arithmetic right shift
(`ix >> lvl`), which is exact and works for negative coordinates. **Being able to
explain this deviation is a scoring point, not a deduction.**

### Mergeable accumulators

Every per-cell field must combine under `+`, `min` or `max`, so pass 2 is the
same code as pass 1 and any level can be rebuilt from any finer level:

```
n, zmin, zmax, zomin, zsum, zsq, ng, gmin, gsum, gsq, hist[nclass]
```

Store `zsum`/`zsq`, **never mean/variance** — sums merge, means do not.

### Answering "without alignment errors or data loss"

The PS sentence is self-contradictory read literally — it asks for coarser cells
far away and for no loss of resolution. It is scoped *"during the projection
from 3D to 2.5D"*, so it means the four binning failures, not resolution:
points dropped, points double-counted, a cell not holding all the points in its
own footprint, and two cells claiming the same ground. All four are what the
two-pass + `blocklevel` design exists to prevent. Provable claims:

- **No point lost or duplicated** — `sum(n) == len(points)` exactly (124,668 on
  frame 0, across 48,909 cells). One line to check.
- **No overlapping footprints** — by construction, verified 0 on 7 frames.
- **Merging is exact** — merging 5 cm cells up three levels gives *bit-for-bit*
  the same 9,027 cells as binning at 40 cm directly, on every field including
  all eight class tallies. Only `zsq`/`gsq` differ, at 5.6e-16 relative, from
  summation order.

Concede two things before a judge finds them:

- **Sub-cell position** is gone. Fine: at 62 m this velodyne's beams land ~43 cm
  apart vertically, ~18 cm horizontally — the 40 cm cell *is* the sampling gap.
  We decline to store precision the sensor never had.
- **Stacking above the lowest obstacle** is gone — that is 2.5D itself, not
  foveation, and true of any uniform 2.5D grid. Measured frame 0: 32% of cells
  hold something above ground; 8.6% of all cells compress >0.5 m of vertical
  extent into `zomin`/`zmax`, 0.75% compress >2 m.

`Notes.md` section 14 is the long-form version.

---

## Bugs already found and fixed — do not reintroduce

These were caught by a synthetic scene with known geometry (`scene.py`).
Each read as plausible output while being wrong.

1. **Roughness must be terrain-only.** Over all points in a column, a cell under
   an overhang mixes ground at 0.3 m with steel at 4.3 m and reads as impassably
   rough. Hence the separate `ng/gsum/gsq` ground-only accumulators. Symptom:
   every overhang untraversable.

2. **Kerbs and potholes need two different reference surfaces.** A kerb is *part
   of* the ground, so the ground raster climbs it and `zmin - zg` reads ~0. A kerb
   is a **discontinuity**: measure against a local minimum over ~1 m. A pothole is
   a deviation from a **coarse trend** over ~4 m. One raster detects neither.
   Measured 0.017 m instead of 0.150 m before the split.

3. **Box blur destroys kerbs.** `uniform_filter` smears a 15 cm step across the
   whole window. Use `median_filter` on the ground raster — edge-preserving.

4. **Absence of ground return is not clearance.** A cell catching the upper half of
   a wall looks identical to a gantry. But requiring per-cell ground returns is
   *also* wrong — at 30 m a 20 cm cell often catches the obstacle and no ground
   purely from sampling density. Test against **distance to nearest real ground
   return in the raster** (`gdist < gnear`). Unknown space is never traversable.

5. **Absence of a ray is not free space, and neither is a neighbour's ray.**
   Free space is inferred by suffix-minimum over elevation tangent per azimuth
   bin (`visibility`), not by stepping along rays. Two things break it:
   smoothing across azimuth (the neighbouring ray passes *beside* a grazing
   wall, not through it), and a wall seen edge-on spreading over range inside
   one azimuth bin so its own returns look like they are "beyond" the cell.
   Hence no azimuth filter and the `rgap` standoff. Wall traversability went
   26.2% -> 14.2% (raw) -> 8.6% (no azimuth blur) -> 4.4% (with `rgap`).

6. **Class aggregation must not be a majority vote.** 195 road + 5 pedestrian
   points in a cell must resolve to *pedestrian*. Histogram plus priority
   override for safety-critical classes. This is where an averaging bug becomes a
   safety bug.

7. **`known` gates the ground surface, not the top surface.** Requiring
   `gdist < gnear` before drawing anything culled 480 observed surface nodes
   (9.7%, median +1.77 m) — on a roof there is no ground return nearby. Sides
   survived (a quad is tested on its origin corner, which sits at the base), so
   the symptom was tops missing while walls stood. Top surface draws on
   `flag & 5` — real cell **or** terrain known; ground surface stays `flag & 4`.

8. **Hole-filled height must carry its neighbour's class and traversability.**
   `ztop` is inherited from the nearest real observation within ~0.6 m; `cls`
   and `trav` were left at `other`/blocked. A car roof then took its height from
   the car and its class from nowhere — grey tops on purple cars, and the drive
   view reading *blocked* across the top of everything. Fill all three from the
   same `distance_transform_edt` indices.

9. **A quad's class comes from its tallest corner, not its origin.** A vertical
   face spanning pavement to roof took the road's colour. Height can be averaged
   over four corners; a class cannot, and the taller thing owns the face.
   Car-coloured faces 263 -> 414. (Both 8 and 9 are in `export_viewer.py` /
   `web/app.js`, not `grid25.py` — the map data was right, the picture was not.)

---

## Validated numbers (synthetic scene, 116k points)

| Quantity | Recovered | Truth |
|---|---|---|
| Kerb height | 0.150 m | 0.150 m |
| Pothole depth | −0.230 m | −0.250 m |
| Gantry clearance | 4.091 m | 4.000 m |

Build time ~85 ms pure numpy (12 Hz), of which free space is ~20 ms.
Cells 62,583.

### Be honest about the memory claim

```
uniform dense 5cm     12,566,370 cells
sparse, occupied only     89,014 cells    141x   <- from SPARSITY
adaptive (ours)           62,583 cells   1.42x   <- from FOVEATION
combined                                  201x
```

**Most of the 201× is sparsity, not foveation.** Do not present 201× as the
foveation benefit — a judge will catch it. Defensible framings: foveation's gain
grows with tier ratio and far-field density; and the real benefit is *statistics
per cell* (merging four cells holding one point each gives one cell with four),
not raw count. Baseline is a uniform 5 cm **2.5D** grid, never an imaginary dense
3D voxel grid — that is a rigged denominator.

---

## Real data — SemanticKITTI

Frame `kitti/000000` (sequence 00), ground-truth labels, no model:

| | |
|---|---|
| points | 124,668 |
| fine cells @ 5 cm | 66,878 |
| adaptive cells | 48,837 |
| build | 95 ms (**~10 Hz**, right at the sensor rate — no headroom for a net) |
| ground under sensor | −1.81 m (velodyne is ~1.73 m up) |
| memory | 188× sparsity, 1.37× foveation, 257× combined |

`fetch_kitti.py` pulls frames without downloading the 80 GB archive: `zipfile`
works on any seekable object, so a small HTTP-range reader lets it inflate single
members out of the remote zip. A few MB per frame instead of 80 GB.

`kitti.py` maps SemanticKITTI's ids onto our 8. Sidewalk and terrain map to
**ground, not road**, so the kerb between them stays a step rather than a cliff
at the edge of the known world.

**The `moving-*` ids (252–259) are the only `is_dynamic` ground truth anywhere in
this project.** `kitti.load` returns them as a third array. Frame 0 has 88 moving
points, all of them one **moving motorcyclist** (id 255, which `kitti.py` folds
onto `ped`). This is what capability 2 should be built against.

`kitti.load(bin)` with no label path returns **everything as `other`** — silent,
and it makes ground vanish (`ng == 0`, `gmin` all `inf`). Always pass the
`.label`.

### Datasets with 360 imagery — surveyed, don't redo it

KITTI's camera is 82 degrees, so the photo panel covers one quadrant of the map.

- **KITTI-360 does not fix this.** Its semantics are painted on *accumulated
  world-frame clouds*, not per sweep. We need a labelled single sweep.
- **PandaSet is the one that fits** — 64-beam spinning lidar, six cameras
  covering the full circle, per-point labels on every sweep, public and
  range-readable. **Blocked on one unresolved question:** its extrinsics say
  `main_pandar64` is at the vehicle origin (roof), but after the devkit's own
  `lidar_points_to_ego` the road sits at −0.12 m, as if the sensor were at
  ground level. Every height here is sensor-relative, so settle that before
  writing a loader. Parked, not rejected.

---

## The model in the loop — measured, 11 frames

Swapping ground truth for `predict.py` over 11 frames (1.3 M points, 442 k cells),
on the six-class `best_5classes.pt`:

| | |
|---|---|
| ground | precision **0.756**, recall 0.975 |
| car | precision 0.907, recall **0.649** |
| pedestrian + cyclist | precision 0.469, recall 0.425 |
| traversability vs ground truth | agrees on **88.8 %** of cells |
| model drivable, truth NOT | **2.09 %** of cells — the unsafe direction |
| truth drivable, model NOT | 9.12 % — conservative, harmless |
| speed | network ~230–330 ms + grid 70 ms = **2.5–3.3 Hz** on this CPU |

The older four-class `best.pt` is still in `trail/`. It differs only in the
semantic layer: car P 0.939 / R 0.635 (F1 0.758 against 0.756 — a wash), and
pedestrian + cyclist P **0.328** / R 0.425, so the six-class model makes a third
fewer false VRU calls at the same recall. **Traversability is bit-identical
between the two** — same 9,270 unsafe cells, same class breakdown — because
drivability never reads the class and the ground split comes from
`remove_ground`, which is unchanged. Van and truck fold onto `car` in
`predict.DET2GRID6`.

**The network is not the problem.** Cluster classification is 97.5 % accurate.
Three things around it are:

1. **`remove_ground` absorbs obstacles.** Over 11 frames it calls 172,149
   non-ground points ground — including **29.6 % of all car points**. A car point
   labelled `other` still blocks the cell geometrically; one labelled `ground`
   joins the terrain raster, raises `zg`, and stops blocking. That is the whole
   of the 2.09 %: 47 % vegetation, 17 % building, **15 % car**.
2. **Clustering discards 87.4 % of clusters** (`min_cluster_pts = 20`) before the
   network ever runs. Worst for distant pedestrians, which *are* a few points.
3. **Out of its training envelope** — trained front-camera-only within 50 m, run
   here 360 degrees to 100 m, so `range/50` exceeds 1.0 on 13 % of clusters.

Neither 1 nor 2 needs retraining; both are one constant.

---

## Free space (`visibility` / `raylow`)

An overhang claim needs **positive evidence** that the space under the cell was
swept, not just an absence of returns — a cell holding the upper half of a wall
has no low returns either. Before this, wall cells read 26% traversable.

No rays are stepped. A return at (range `R`, height `Z`) means the sensor saw
clean through every column in front of it at that azimuth: at range `r < R` the
ray was at height `Z·r/R`. So a whole ray is **one number**, its elevation
tangent `t = Z/R`, and the lowest ray over a column is the smallest `t` among
returns beyond it, times that column's range. That is a suffix-minimum over
range within each azimuth bin — one sort, ~15 ms, no ragged per-ray arrays.

`swept = (zray - zg) < vh` gates the overhang rescue. Failing it is always the
safe direction: a cell with no obstacle is traversable regardless, so a missing
ray can only ever make the map more conservative. Set `naz` to the sensor's own
azimuth resolution, no finer.

The sensor-height caveat is now **settled on real data**. In `scene.py` the
sensor sits at ground level, so `zray - zg` is ~0 under the gantry for free. On
SemanticKITTI seq 00 frame 0, with the velodyne 1.73 m up, it measures **median
+0.23 m, 90th percentile +1.04 m**, and 93% of cells have a ray at all. That is
comfortably under `vh = 2.2`, so the overhang rescue survives a roof-mounted
sensor with room to spare. Re-check it if the tier ratio or `naz` changes.

Result: under the gantry 100% swept, tall wall cells 2.8% swept. Wall
traversability 26.2% -> 4.4%. The 4.4% residual is *not* a free-space failure —
it is 130 cells holding a single wall return at ground level, with no obstacle
above ground to reject. Only temporal fusion or the semantic layer catches those.

---

## Not built yet

- **Temporal fusion.** Everything is single-frame. Per-cell Kalman height with
  variance, as in ANYbotics `elevation_mapping`.
- **Dynamic flag.** `is_dynamic` cannot come from single-frame segmentation.
  Needs the scan buffer + ego-motion compensation + a moving-object head or
  residual-occupancy check. SemanticKITTI has MOS labels for this.
- **ROS 2 node wrapper**, `sih_msgs`, planner. The dashboard exists as `server/`
  + `web/` (offline, replaying frames); what is missing is the live ROS-side one
  showing dropped frames and end-to-end latency off the header stamp.
- **Content-aware refinement** — refine back to high-res on dynamic objects at
  range, so a pedestrian at 40 m doesn't vanish into a 40 cm cell. Differentiator.

---

## Model

**Not PointNet++** — it will not hit real-time at 100 m; sampling/grouping cost is
brutal at ~120k points. Use a range-image net (SalsaNext) or sparse-conv
(Cylinder3D, SPVNAS). Keep PointNet++ in the report as "considered and rejected,
here is the latency table" — that reads better than not having considered it.

Train **outside ROS**, on the remote GPU box or Colab. Plain Python reading files
off disk. Teams lose weeks trying to make training ROS-native.

**Develop the grid engine against SemanticKITTI ground-truth labels**, so it never
blocks on the model. Swap to predicted labels in week 4.

---

## Consumers (drives the message schema)

Local planner (cost per cell), clearance checking (`z_ground` **and** `z_obst_min`
separately — that is what 2.5D buys over 2D), mobility governor (slope/roughness →
speed), terrain-relative localisation (GPS-denied — strong DRDO angle), remote
operator display.

**The bandwidth argument is the pitch.** A 3D cloud at 10 Hz is ~20 MB/s and
cannot cross a tactical radio link. A sparse 2.5D grid can — so vehicle A drives a
route, transmits its map, and vehicle B arrives already knowing the terrain.
Impossible with raw point clouds.

Include `point_count` / confidence so a planner can distinguish *empty* from
*unobserved*.

---

## Existing libraries — what to take, what not to

Nothing off-the-shelf does adaptive-resolution 2.5D semantic mapping. That is the
contribution. Say so explicitly in the presentation.

- **grid_map** (ANYbotics) — multi-layer 2.5D, ROS 2 branch. Take the **data
  model**. Do **not** try to extend it: it stores Eigen matrices in a circular
  buffer and is fast *because* it is fixed-stride. Variable cell size breaks it at
  the foundation.
- **elevation_mapping / _cupy** (ETH) — take the per-cell Kalman temporal fusion.
- **OctoMap / wavemap** — adaptive, but 3D volumetric, not 2.5D. Take the octree
  indexing and ray-casting ideas.
- **grid_map_pcl** — lowest-cluster trick for ground height.
- **MLS maps** (Triebel 2006) — multiple surfaces per cell, for overhangs.

---

## Code style

- Short lowercase names, no capitals, no semicolon-stacked one-liners
- Minimal data structures; prefer flat numpy arrays over classes
- Vectorised numpy — **never** loop over points in Python
- `read_points_numpy`, never `read_points` (the iterating version is slower than
  the neural network)
- Honest assessment over optimistic framing; read code carefully before making
  claims about its logic

## Files

- `grid25.py` — the converter. Pure numpy, no ROS dependency, unit-testable.
- `scene.py` — synthetic scan with known kerb / pothole / gantry / pedestrian.
- `check.py` — validates recovered geometry against those truths.
- `predict.py` + `trail/` — the PointNet detector from MadhankumarAI/trail,
  producing per-point labels. It is a **cluster detector, not a segmenter**:
  6 classes (Background/Car/Pedestrian/Cyclist/Van/Truck), one label per
  cluster, from `trail/best_5classes.pt`. Ground comes from its geometric
  `remove_ground`, not from the network, and building / vegetation / pole have
  no class at all and land in `other`. Van and truck fold onto `car`; cyclist
  onto `ped`, so it inherits the priority override. `predict._tables(cfg)` picks
  the mapping from the checkpoint's `num_classes`, so the older four-class
  `best.pt` still loads — `predict.load('trail/best.pt')`, or
  `evaluate_model.py trail/best.pt`.
  Needs torch (cpu wheel), numba, pyyaml — all in `.venv`.
  **Only the checkpoint was taken from the upstream update.** That repo also now
  carries an `integration/` tree (temporal accumulation, dense numba tiers,
  grid_map emission) written against `grid25.py` — deliberately not adopted;
  its dense robot-centric grid trades away the sparsity the memory claim rests
  on.
- `evaluate_model.py` — scores predicted labels against ground truth at point
  level AND at map level. The map-level column is the one that matters.
- `export_sequence.py` → `sequence.json` → `player_tpl.html` → `player.html`
  — every frame in `kitti/` through the model and the grid, played back as one
  surface per frame. Surface base is 4x coarser (20/40/80/160 cm) purely to fit
  11 frames in one page; still an exact coarsening of the same grid.
- `fetch_kitti.py` — range-reads SemanticKITTI frames out of the remote zips.
  `python fetch_kitti.py 00 000000 000100`
- `kitti.py` — loads a frame as `(points Nx3, labels N, moving N)` in our classes.
- `export_viewer.py` — `(points, labels) -> viewer_data.json + viewer_surface.json`
  for any frame. `python export_viewer.py kitti/000000.bin`, or `scene` for the
  synthetic one. Prints the free-space margin as a check.
- `viewer_tpl.html` + `viewer_data.json` + `viewer_surface.json` → `viewer.html`
  — the demo viewer. Regenerate both JSONs after any `grid25.py` change and
  splice them into the template at `/*__DATA__*/` and `/*__SURF__*/`.
  Three views: **Surface** (a dense elevation mesh on the same foveated tiers,
  shaded — this is the one that reads as 2.5D at a glance), **Cells** (the
  sparse cell set as tiles and prisms), **Plan + section**.
  - The sparse cell set is what you store and transmit. The surface is the
    dense field it describes — a node wherever a cell of that tier *could*
    exist, ~441k nodes. Do not quote the surface node count as the map size.
  - Unobserved surface nodes are filled from the nearest real observation
    within ~0.6 m, then fall back to the terrain raster. Without that a wall
    alternates between 8 m and ground and the surface reads as a comb.
  - The display stride must be **one power of two shared by all four tiers**.
    A per-tier stride drives every tier to the same on-screen size at any zoom
    that fits the scene, which hides the only thing the view exists to show.
  - Obstacles hang from `zg + clear`, never from the ground, or the picture
    asserts the opposite of what the clearance field says.
  - Shading normals are taken over **one display step**, divided by the metres
    actually spanned (grid_map's `NormalVectorsFilter` idea: a fixed radius, not
    a fixed neighbour count). Per-cell normals make every foveated cell its own
    facet at its own size. **No height is altered** — only the lighting.
  - Surface base is a job parameter (`detail` = 4/2/1 -> 20/10/5 cm), measured
    on 00/000000: 48k nodes / 0.39 MB, 190k / 1.52 MB, 756k / 6.05 MB. A car is
    ~3 nodes across at the 40 cm tier, which is why it read as a slab at the
    coarse default. The stored map is 48,909 cells at every setting.
- `server/` + `web/` + `run_server.sh` — **the pipeline as a service.** Nothing
  precomputed: a job names a sequence and a frame selection, and a worker runs
  fetch -> label -> grid -> surface per frame, streaming each one to the browser
  over SSE as it lands. `./run_server.sh`, then http://127.0.0.1:8011.
  - Four colourings: height, class, drivable, and **detector**. The last one is
    the useful one with model labels, because the plain class view collapses to
    73% ground / 24% other / 3% car. It splits `other` into *examined and
    rejected* (10.7% of points) versus *never clustered* (22.5%) — the second
    is the network's blind spot and was previously invisible. `predict.predict`
    supplies it via `with_prov=True`; it rides to the browser as one extra byte
    per surface node.
  - `POST /api/jobs {seq, mode, start, count, stride, source, seed, camera,
    detail}` — mode is `sequential` (consecutive sweeps, real motion) or
    `random` (scattered); source is `model` or `truth`.
  - `GET /api/jobs/{id}/events` — SSE, one event per finished frame.
  - `GET /api/jobs/{id}/image/{frame}` — the matching camera photo, range-read
    out of `data_odometry_color.zip` the same way. **Forward-facing 82 degrees
    against the map's 360** — there is no 360 photo in KITTI, so stop looking
    for one. Fetched on its own lower-priority lane so a slow photo never delays
    a map. First one costs ~50 s (that archive's index is 87,215 entries), then
    ~5 s each. Frame 00/000000 has the moving motorcyclist visible in it.
  - `pipeline.project()` puts the points **on** the photo (`Tr` then `P2` from
    `calib.zip`, keep `z > 0.1` and inside the frame — ~15% of a sweep). Real
    projection, class-coloured; the fastest way to confirm a lump in the map is
    the car in the picture.
  - The UI state is in the query string, so a run is a shareable link:
    `?seq=00&mode=sequential&count=8&auto=1`.
  - Left-drag orbits; **everything else pans or zooms** — arrow keys and the pad,
    `+`/`-` and the wheel (at the cursor), right/middle/shift-drag, `f` to fit,
    space to play, `,`/`.` to step. The camera is kept across frames so you can
    park it on one car and watch it move.
  - **Fetching dominates a cold run** — ~40 s a frame, because each one is
    range-read out of the 80 GB remote zip. Three downloads overlap, so 4 cold
    frames take ~50 s rather than ~150 s. Cached frames are instant.
  - `cache/raw/` holds the .bin/.label, `cache/frames/` the built JSON. Both are
    caches, not inputs — delete them and the pipeline refills them.
- `.venv/` — numpy + scipy + torch(cpu) + numba + fastapi. The PATH python is
  another project's venv and has none of it, so run `.venv/bin/python check.py`,
  not `python3 check.py`.
- `Notes.md` — the same pipeline in plain English, 18 sections, no jargon: what
  goes in, what each step does and why, what we got wrong, the compression
  claim, the "no data loss" reading, and the honest limits. Keep it in step with
  this file.
- `requirements.txt` — carries `--extra-index-url .../whl/cpu`. Plain
  `pip install torch` pulls ~2.5 GB of CUDA wheels onto a machine with no GPU.

Keep the grid builder **outside ROS**. It is a pure function
`(points Nx3, labels N) -> grid`. Develop it in a script with a two-second edit
loop, wrap it in a ~20-line node once correct.

## Commands

```bash
python3 check.py          # validation suite — run after every grid change
```