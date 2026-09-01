# How the 2.5D map is made

Plain-English notes on the whole pipeline: what goes in, what happens to it,
why each step is the way it is, and what we got wrong on the way.

---

## 1. The problem

A laser on the roof spins round and fires about 120,000 times per sweep, ten
times a second. Every shot that hits something records the spot it hit. So what
arrives is a pile of 120,000 loose dots floating in space.

A pile of dots is heavy and hard to reason with. You cannot ask it "can I drive
there?". It is about 2 MB per sweep, 20 MB a second, which is too much to store,
too much to send over a radio link, and too much for a route planner to chew on.

**What we want instead:** look straight down, chop the ground into squares, and
for each square remember a few summary numbers. That is a *map*, and you can ask
a map questions.

The catch the problem statement adds: the squares should not all be the same
size. They should be small close to the vehicle and large far away.

---

## 2. Why the squares change size

Close to the vehicle the laser dots land packed tightly together, so small
squares are worth having — there is genuinely fine detail to record.

Far away the same number of shots is spread over a much wider area. A tiny
square out at 60 m catches one lonely dot, and one dot tells you almost nothing:
no sense of how rough the ground is, no confidence, no way to tell a real
surface from a stray reflection.

So the squares grow with distance: palm-sized nearby, dinner-plate-sized far
off. Same idea as your eyesight — sharp where you are looking, rough at the
edges.

We use four sizes, each double the last:

| distance from the sensor | square size |
|---|---|
| under 10 m | 5 cm |
| 10 – 25 m | 10 cm |
| 25 – 50 m | 20 cm |
| beyond 50 m | 40 cm |

The doubling matters and is not cosmetic — see section 4.

---

## 3. Step one: every dot into a 5 cm square

Divide each dot's position by 0.05 and round down. That gives a column number
and a row number, and the dot belongs to that square. Done.

**Important: distance plays no part here.** Every dot, near or far, goes into a
5 cm square. The size decision comes later, and that ordering is the whole
design (section 5).

### What each square remembers

Once the dots are grouped, each square throws the dots away and keeps a short
list of numbers. Two separate sets of books:

**Everything that landed here**
- how many dots
- the lowest one, the highest one
- the lowest one that was *not* ground
- the sum of the heights, and the sum of the heights squared

**Only the dots labelled ground or road**
- how many
- the lowest one
- their sum, and their sum of squares

**And a tally** — of the eight possible labels (ground, road, building, pole,
vegetation, car, pedestrian, other), how many dots of each.

### Why sums instead of the average

You would think to store the average height and how spread out the heights are.
You cannot, and the reason is section 4.

**Averages do not combine.** If one square averages 2 m and its neighbour
averages 4 m, the pair does not average 3 m unless they hold the same number of
dots. Sums do combine: add the sums, add the counts, divide once at the very
end.

Worked example from a real cell — four squares merging:

    merged mean = total sum / total count = -89.3606 / 122 = -0.732464
    same figure from the 122 raw dots                       = -0.732464
    averaging the four squares' own means                   = -0.747505   wrong

The four held 17, 60, 15 and 30 dots. Averaging the averages weights the
15-dot square the same as the 60-dot one.

The spread comes out of the two sums as well:

    average  = sum / count
    spread   = sqrt( sumOfSquares/count  -  average² )

Checked against a direct calculation on a real merged cell: they agree to 15
decimal places.

### How much this compresses

124,668 dots become 66,878 squares. And a fact that drives everything after
this: **61% of those squares hold exactly one dot.**

---

## 4. Step two: gluing squares together

### The groups are fixed, not searched

There is no "find similar cells and merge them". Four 5 cm squares sitting in a
2x2 block occupy exactly the same ground as one 10 cm square, so those four are
the only combination ever possible. Sixteen make a 20 cm square, sixty-four make
a 40 cm one.

Finding which block a square belongs to is division by 2, 4 or 8, rounding down.
In the code that is a "bit shift" (`>>`), which is the same thing done directly
on the binary digits — exact, no rounding error, and it behaves correctly for
negative coordinates (everything behind and to the right of the vehicle).

So the only decision left is **how far up to climb**: stay put, or merge with 4,
16 or 64 neighbours.

### The merge itself

Every stored number combines with either addition, a minimum, or a maximum:

    count     17 + 60 + 15 + 30                        = 122      add
    lowest    min(-1.593, -1.593, -1.396, -1.600)      = -1.600   min
    highest   max( 0.399,  0.337, -0.207,  0.201)      = +0.399   max
    sum       -9.955 + -39.345 + -12.400 + -27.661     = -89.361  add
    tally     [2,0,0,15,...] + [5,0,0,55,...] + ...    = [11,0,0,111,...]  add

Nothing in that list needs to know it is a merge. That is why merging reuses the
exact same code as step one, just pointed at a coarser grouping. It is also why
the list of stored numbers is what it is — anything that could not be combined
this way was not allowed on the list.

### What merging actually gains

A 40 cm square *could* swallow 64 small ones. In practice it swallows **1.9 on
average**, because the other 62 had no dots at all and never existed.

    tier      cells   most it could hold   held exactly 1   average held
    5 cm     25,590            1               25,590           1.0
    10 cm    16,791            4                8,247           1.7
    20 cm     5,486           16                2,922           1.9
    40 cm     1,042           64                  569           1.9

A real 40 cm square: 15 of its 64 possible small squares had dots, fourteen of
them holding a single dot each. Merged, it holds 17 dots spanning 3.77 m of
height — enough to say "something tall stands here", which none of the fifteen
could say alone.

What survives the merge: the full height range, the count, the class tally, the
ground-only figures. What is lost: *where inside the 40 cm square* each height
was. At 54 m, where the laser beams are metres apart, that position was never
measured to 5 cm anyway.

---

## 5. Why the size is decided per square and not per dot

The obvious approach is: look at each dot, see how far away it is, and put it
straight into a small or a large square. **That is broken**, and it is worth
understanding exactly how, because the failure is silent.

### The failure

A cell has two halves that must agree: *how big it is* (its tier) and *which
patch of ground it covers* (its footprint). If the tier comes from a dot's own
distance while the footprint comes from the grid, nothing forces them to match.

Take two dots 1 cm apart, either side of the 10 m line. One goes into a 5 cm
square, the other into a 10 cm square — and because the sizes nest, that 10 cm
square **physically contains** the 5 cm one. Both end up in the output. The same
patch of ground is claimed twice, and each cell holds a different subset of the
dots standing on it.

Measured on a real frame with the broken version:

    cells that do not contain all the dots inside their own footprint     80
    dots landing in the wrong cell                                       358
    the median affected cell is missing 50% of its contents; the worst 89%
    patches of ground claimed by two cells at once                        83

The worst one: a 10 cm cell whose footprint held 53 dots but which aggregated
only 6. It reported the tallest thing at +0.441 m; the truth was +0.551 m. Its
class tally said "building x6" when the truth was "building x53".

**Nothing about the output looks wrong.** It produces 48,846 cells against the
correct 48,909 — a perfectly plausible number, no error, no warning.

### The fix

Put every dot in a 5 cm square first, with no distance test at all. *Then* look
at each square and decide its size. Now a dot has already gone into exactly one
place and cannot be double-counted, whatever the size decision turns out to be.

---

## 6. The second thing we got wrong, and fixed later

Quantising first fixes the dot-level problem. It does **not**, on its own, fix
the overlapping-ground problem, and we missed that for a while.

### Why the boundary is a circle

Two different things are laid on top of each other:

- The **grid** is square — dividing by 0.05 makes square cells.
- The **size rule** is radial — it uses straight-line distance from the sensor,
  `sqrt(x² + y²)`. "Within 10 m" is therefore a *disc*, and its edge is a circle.

The rule is radial because that is the physics: laser density falls off with
distance from the sensor, equally in every direction.

A circle cannot follow square edges, so squares near the boundary have some
corners inside and some outside:

    #  every corner inside 10 m     .  every corner outside
    /  the 10 m circle cuts through it

    y= 7.90  /  /  .  .  .  .  .
    y= 7.80  #  /  /  .  .  .  .
    y= 7.70  #  #  /  /  .  .  .
    y= 7.60  #  #  #  /  /  .  .
             6  6  6  6  6  7  7    <- x

### What went wrong

We asked each *5 cm square* for its own distance. So one block could split: the
squares inside the ring stayed at 5 cm, the ones outside merged into a 10 cm
parent — whose footprint contains the 5 cm squares while holding none of their
dots. **82 overlapping footprints** on a real frame.

Note what was *not* wrong: dots were still partitioned perfectly. The total
count matched the input exactly on every frame. Nothing was lost or
double-counted. The overlap was between *footprints*, so a planner asking "what
is at this spot?" got two cells and no rule for choosing.

### The fix

Ask the **block**, not the individual square:

> Merge a block only if **the whole of it** lies beyond the ring. If any part
> reaches inside, the whole block stays where it is.

The test measures the *closest point of the block* to the sensor. If even that
is past the line, nothing in the block reaches inside.

It has to be the closest point, not the block's centre. A real example:

    block  x [-10.00, -9.90]  y [-1.10, -1.00]
      closest point to sensor   9.9504 m   -> reaches inside the 10 m circle
      block centre             10.0052 m   -> looks like it is outside

The circle genuinely cuts that block, but its centre sits just past the line.
**78 of the 183 straddling blocks** in that frame are like this — the centre
test gets them all wrong.

Because every square in a block computes the same block position, they all reach
the same verdict. The block can never split. Verified: **0 overlapping
footprints** on all seven test frames, dots still perfectly partitioned.

Cost: **+0.15% more cells** (48,837 to 48,909). Worth it, and it errs the safe
way — resolution never gets coarser across a boundary.

### An equivalent way to say it

Grow the cells outward one ring at a time, and anything a ring cuts through
stays where it is:

    start: everything 5 cm
    ring at 10 m:  blocks wholly outside -> 10 cm.  straddlers stay 5 cm.
    ring at 25 m:  blocks wholly outside -> 20 cm.  straddlers stay 10 cm.
    ring at 50 m:  blocks wholly outside -> 40 cm.  straddlers stay 20 cm.

This gives *identical* results to what the code does, verified cell-for-cell on
seven frames. It works because the blocks nest: a block wholly beyond 50 m is
automatically wholly beyond 25 m and 10 m too, so passing the far test implies
passing all the near ones.

One clarification: being refused at a ring costs you *that one step*, not
everything. A block refused at the 50 m ring stays at 20 cm, not 5 cm — it keeps
what it already earned at the rings further in. Checked: 320 such cells in a
frame, all at 20 cm.

---

## 7. Step three: finding the ground

Now a problem. A square that caught only a wall has no idea where the floor is —
and it needs to, because everything useful is measured *relative to the ground*.
How tall is this thing? How much headroom is under it? Is this a step? All of
those are "minus the ground height".

So a **second, completely separate structure** is built: a plain grid of 25 cm
patches, all the same size, using **only the dots labelled ground or road**.

It has to be a uniform grid because the next step slides windows over it
(minimum over a window, average over a window), and you cannot slide a window
over cells of different sizes.

A real patch of it, in front of the vehicle:

    -1.723 -1.715 -1.711 -1.702 -1.702 -1.702 -1.702
    -1.715 -1.707 -1.698 -1.694 -1.688 -1.688 -1.688
    -1.710 -1.697 -1.694 -1.686 -1.686 -1.683 -1.683
    -1.710 -1.699 -1.694 -1.683 -1.683 -1.678 -1.675

Around -1.69 m because the sensor sits about 1.7 m above the road. Gently
sloped — that is the crown of the road.

Three details that matter:

**Take a low percentile, not the minimum.** One bad reflection below the surface
would drag the whole patch down and invent a hole that is not there.

**Fill the gaps, and record that you did.** Only **9%** of the 105,324 patches
actually had a ground dot in them; the rest borrow the height from the nearest
patch that did. A second grid stores *how far* each value had to travel. That is
the honesty channel — without it the map cannot tell a measurement from a guess,
and later on "I do not know" has to be a possible answer.

**Smooth with a median, not an average.** An average smears a 15 cm kerb across
the whole window until the kerb stops existing. A median removes speckle and
leaves the step intact.

### Two reference surfaces, because a kerb and a pothole are opposites

**A kerb is part of the ground.** The ground surface climbs the kerb along with
it, so measuring a kerb against a smoothed version of the ground reads roughly
zero. A kerb is a *sudden change*. So: compare each patch against the **lowest
point within about a metre**. Flat road gives zero; a kerb gives the full 15 cm.

**A pothole is a hole in the ground.** Any reference that follows the road
closely falls into the hole with it and also reads zero. So: compare against a
**smooth average over about four metres**, coarse enough not to fall in.

One reference surface detects neither. This is measured, not theoretical:
before splitting them, a known 15 cm kerb measured **1.7 cm**.

---

## 8. Step four: proving that space is empty

### The trap

A square hidden behind a wall has no dots in it. So does an empty stretch of
road. **In the data they are identical.**

That matters for overhangs. A gantry 4 m up is drivable — you go under it. But a
square that caught only the *upper half* of a distant wall also has nothing
recorded below 2.2 m. Treat those the same and you drive into walls. Before this
step existed, **26% of wall squares read as drivable.**

So the map needs *positive evidence* that something swept the low space, not
merely an absence of returns.

### The trick

The textbook method is to walk along every laser beam step by step, marking what
it passed through. That is about 18 million steps per sweep and would cost more
than everything else in the pipeline combined.

You do not have to walk. The sensor is at a fixed point, so every beam is a
straight line from it. A reflection at distance R and height Z means the beam
was at height Z x r/R at every closer distance r — similar triangles. So **an
entire beam is described by one number: its slope.**

The question "what is the lowest anything passed over this square?" becomes
"among the reflections in the same compass direction that are *further away*,
which has the smallest slope?" — multiplied by this square's distance.

A worked example from a real cell 7.38 m away:

    45 reflections lie in the same thin direction slice, more than 0.5 m beyond it

      distance   height    slope     beam's height over our square
        7.89     -1.940   -0.2459            -1.814 m
        8.21     -1.938   -0.2362            -1.742 m
        8.63     -1.954   -0.2264            -1.669 m

    lowest beam  -1.814 m      ground here  -1.947 m
    so a beam cleared the ground by 0.21 m -- under the 2.2 m vehicle roof
    the low space really was swept

No beam was walked. It is a running minimum over a precomputed table, about
**11 ms** for 124,668 dots.

### Two traps inside the trick, both closed

**Do not borrow the neighbouring direction slice.** We originally smoothed
across neighbouring slices, worried thin ones would come up empty. That is wrong
for a wall seen almost edge-on: the neighbouring beam slides *past* the wall
rather than through it, and the wall reads as see-through. Removing it took wall
drivability from 14.2% to 8.6%.

**Skip the first half metre.** A wall seen edge-on spans half a metre of
*distance* within a single direction slice, so the wall's own reflections look
as though they got past it. Requiring a reflection to be at least 0.5 m further
out took it from 8.6% to **4.4%**.

The remaining 4.4% is not a free-space failure. It is squares holding a single
wall reflection right at ground level, with no obstacle above ground to reject.
Only combining several sweeps over time, or the class label, catches those.

### One caveat that turned out fine

In our synthetic test scene the sensor sits at ground level, so a beam aimed at
distant ground stays at ground level and this test passes almost for free. On
real data with the sensor 1.73 m up we measured the margin properly: **median
0.23 m, 90th percentile 1.04 m**, and 93% of squares have a beam at all.
Comfortably under the 2.2 m threshold, so it survives a roof-mounted sensor.

---

## 9. Step five: subtracting

Every square asks the ground surface four questions at its own position, reading
smoothly between the four surrounding patches:

- how high is the ground here
- how far away was the nearest *real* ground dot
- how much does the ground rise above its local low point (kerbs)
- what is the large-scale road level here (potholes)

Then it is arithmetic:

    obstacle height = highest dot           - ground
    headroom        = lowest non-ground dot - ground
    pothole depth   = lowest ground dot     - large-scale road level
    bumpiness       = spread of the GROUND dots only
    class           = the tally's winner, with an override

Two of those need explaining.

**Bumpiness uses the ground dots only.** A square under a tree branch holds road
at -0.2 m and branch at 3.7 m. Using all its dots the spread is **1.57 m**
against a limit of 0.08 m, so the square reads as impassably rough purely
because something hangs over it. Using ground dots only: **0.016 m**. This is
the entire reason two separate sets of books are kept in step one.

**Class is not a plain majority.** A handful of pedestrian dots must not be
outvoted by a road-dominated square. Three pedestrian dots override the winner.

---

## 10. Step six: the decision

One yes/no per square:

    known    = the nearest real ground dot is within 2 m
    swept    = a beam actually passed through below roof height
    overhang = headroom > 2.2 m  AND known  AND swept
    solid    = something taller than 12 cm stands here, and it is not an overhang

    drivable = known AND not solid
                     AND kerb step  < 12 cm
                     AND pothole    < 10 cm deep
                     AND bumpiness  < 8 cm

`known` comes first for a reason: **unknown space is never drivable.** Guessed
ground gets no vote. About 90% of squares qualify.

And notice this never looks at the class. It is pure geometry, deliberately, so
that a hazard the network has never seen is still caught by being tall, steep,
or a hole.

Two real squares, side by side:

                            drivable one        blocked one
    ground here               -1.964 m            -1.488 m
    nearest real ground        0.000 m             3.000 m   <- 3 m away
    obstacle height           -0.003 m            +1.686 m
    headroom                  nothing above       +0.978 m
    lowest beam over it       -1.619 m            none passed
    ---------------------------------------------------------------
    known                     yes                 no
    swept                     yes                 no
    solid                     no                  yes
    DRIVABLE                  yes                 no

The blocked one fails three separate ways at once, and that redundancy is the
point.

---

## 11. Cases worth thinking about

### A pothole and a tree branch in the same square

They do not interfere, because the square keeps two separate sets of books.

Real square: 4 road dots down in the pothole, 1 branch dot at 3.7 m.

- **Pothole**, measured from the ground dots only: floor at -0.208 m against a
  smooth road level of -0.012 m. A dip of -0.196 m. Too deep. **Blocks.**
- **Branch**, measured from the non-ground dots only: 3.84 m above the ground,
  more than the 2.2 m roof. **Drive under.** Does not block.
- **Bumpiness** from road dots alone: 0.016 m, fine. From all dots it would be
  **1.57 m** and the square would read impassable for no reason.

Answer: not drivable, because of the pothole. The branch is correctly ignored.
Lower the branch to 1.5 m and it blocks too — two independent reasons, each
measured from its own dots.

### A car and a building in the same square

The tallest thing wins, so the car's own height is not kept separately. But:

- Mixing is rare. Over 11 frames, **1.37%** of squares hold two different
  non-ground things, and car+building never once occurred. What actually mixes
  is building+vegetation, i.e. trees against walls.
- The class tally keeps both regardless — nothing is discarded.
- **Mixing can never make a square look safer.** Obstacle height is a maximum,
  so adding anything can only raise it. Headroom is a minimum, so adding
  anything can only lower it. Both move toward *blocked*. Borne out: 99.2% of
  mixed squares are non-drivable anyway.

What you cannot get from the map is per-object properties — "how tall is that
car". It is a field, not an object list. That is the detector's job.

### Multiple labels in one square

Every square keeps the **full tally of all eight labels**, and tallies add when
squares merge, so no label is ever thrown away. Only the single "class" field
collapses it to one winner.

97% of squares are pure — one label only. Purity falls as squares get bigger,
exactly as expected.

An honest finding: **the pedestrian override has never actually fired** — not on
real data, not on the synthetic scene. Pedestrian squares turn out to be
naturally pure, because a person standing on the road *blocks the beam from
reaching the ground beneath them*, so their square gets body reflections and
little else. The override is untriggered insurance, not a fix for an observed
problem. It would start to matter with larger squares or a lower sensor.

The real loss is the opposite one: 9 squares across 11 frames held only one or
two pedestrian dots and were outvoted, because the override needs three.

---

## 12. Where the labels come from

Everything above needs to know which dots are ground. That comes either from
SemanticKITTI's hand-made ground truth, or from a model.

The model we wired in is a **cluster detector, not a segmenter**. It does not
label individual dots. It takes a clump of dots and says which of four things it
is: background, car, pedestrian, cyclist. Per-dot labels are assembled from
three stages, only one of which is the network:

    remove ground   ground / not ground per dot     geometry, no network
    clustering      group the leftovers into clumps geometry, no network
    the network     one label per clump             <- the network

So the network contributes only the car / pedestrian / cyclist split. Buildings,
vegetation and poles have no class to go to and land in "other".

### What that costs, measured over 11 frames

    ground          precision 0.756   recall 0.975
    car             precision 0.939   recall 0.635
    pedestrian      precision 0.328   recall 0.425

    the finished map agrees with the ground-truth version on 88.8% of squares
      model says drivable, truth says not :  2.09%   <- the unsafe direction
      truth says drivable, model says not :  9.13%   (over-cautious, harmless)

**The network itself is not the problem** — its clump classification is 97.5%
accurate. Three things around it are:

1. **The ground remover swallows obstacles.** Across 11 frames it calls 172,149
   non-ground dots "ground", including **29.6% of all car dots**. This matters
   specifically because a car dot mislabelled "other" still blocks the square
   geometrically, but one labelled "ground" joins the terrain surface, raises
   the ground height, and stops blocking at all. That is where the 2.09% comes
   from: 47% vegetation, 17% building, **15% car**.
2. **Clustering throws away 87.4% of clumps** before the network ever sees them,
   because they have fewer than 20 dots. Worst for distant pedestrians, which
   *are* a handful of dots.
3. **It is being run outside its training conditions** — trained front-camera
   only within 50 m, run here all round to 100 m.

Neither 1 nor 2 needs retraining. Both are a single constant.

It is also now the slow part: **428 ms for the network** against 80 ms for the
grid, so about 2 Hz against a 10 Hz sensor.

### Colouring the map by what the classifier said

The map can be coloured four ways: by height, by class, by drivable, and by
**detector**.

The plain class view is nearly useless when the labels come from the model,
because the model only has four categories. Measured over 14 frames:

    ground 73.2%   car 3.1%   pedestrian 0.0%   other 23.6%

Four of the eight legend entries are permanently empty, and that 23.6% "other"
hides the interesting part, because it merges two completely different things.

So there is a separate **detector view** that colours by what the model actually
did with each point:

    ground (geometric)     the ground remover found it. the network had no say.
    examined, rejected     the network looked at this clump and said "nothing".
    car / pedestrian / cyclist    the network's actual output.
    never clustered        the network NEVER SAW this. dropped before inference.
    no cell here           no reflections at all.

On one real frame:

    ground                79,502   63.8%
    examined, rejected    13,307   10.7%
    car                    3,774    3.0%
    pedestrian                 0
    never clustered       28,085   22.5%   <-

That last line is the point. **22.5% of the sweep never reached the network at
all** — more than twice what it examined and rejected — because clustering
throws away any clump with fewer than 20 points. Both used to be painted the
same grey and were indistinguishable. Now the blind spot is visible on the map.

---

## 13. Being honest about the compression

    uniform 5 cm grid over the same area   12,566,370 squares
    only the occupied ones                     66,878    188x   <- from sparsity
    with variable sizes on top                 48,909   1.37x   <- from foveation
    combined                                            257x

**Most of that factor is sparsity, not the variable sizing.** 98% of the ground
around the vehicle produced no reflections at all, so it has no squares. Saying
"257x from adaptive resolution" would be wrong and a reviewer would catch it.

The variable sizing is worth 1.37x on its own, and the reason it is not more is
back in section 4: there is usually only one child to combine. Its real benefit
is not the count — it is **statistics per square**. Merging four squares holding
one dot each gives one square holding four, which can say things none of the
four could.

The comparison is always against a uniform 5 cm *2.5D* grid. Comparing against
an imaginary dense 3D cube grid would be a rigged denominator.

---

## 14. Running it as a workflow

Rather than keeping files on disk and rebuilding by hand, the whole thing runs
as a service. `./run_server.sh`, then open http://127.0.0.1:8011.

You choose a sequence and a set of frames, press Run, and a worker does
**fetch -> label -> grid -> surface** for each frame, streaming each one to the
browser as it finishes. Frames become playable as they arrive; you do not wait
for the whole run.

- **Sequential** frames are consecutive sweeps — real vehicle motion.
- **Random** frames are scattered across the sequence — unrelated scenes.
- Labels come from either the detector or the ground truth.

Nothing is precomputed. Frames are pulled out of the public SemanticKITTI
archives on demand. Those archives are 80 GB, so instead of downloading them we
read only the few megabytes belonging to one frame, using ordinary HTTP range
requests. A few MB per frame instead of 80 GB.

Measured costs per frame:

    fetching a frame not seen before     ~40 s     <- dominates a cold run
    labels, from the detector            ~305 ms
    labels, from ground truth            ~2 ms     (a file read and a lookup)
    building the grid                    ~90 ms
    reducing it to a drawable surface    ~35 ms
    a frame already fetched              instant

Fetching is by far the slowest part and is pure waiting on the network, so three
downloads run at once: four cold frames take about 50 seconds instead of 150.

The files on disk under `cache/` are a cache, not an input. Delete them and the
pipeline refills them.

---

## 15. What this honestly does not do

- **One sweep at a time.** No combining across frames, no vehicle motion. Each
  frame is an independent map with the sensor at the origin.
- **One surface per square.** Ground and obstacle-bottom, so a gantry works —
  but a bridge you drive *on top of* cannot be expressed at all. That is the
  classic limit of this kind of map and we have it in full.
- **Bumpiness is thin on real data.** Only 19.5% of squares have more than two
  ground dots, and beyond 50 m none do. It carries far less weight on real data
  than on the synthetic scene.
- **No moving/stationary distinction.** A parked car and a walking pedestrian
  look the same. That needs several sweeps plus motion compensation.
- **Labels are an input, not an output.** Swap ground truth for the detector and
  the geometry is byte-identical; only the class and terrain layers move.
