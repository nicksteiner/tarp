# TARP — Method notes

Development history and rationale for the coverage-adequacy metric. This is the
method-paper source document. v0 ships without any of this; v1 is the research
thread.

## Problem statement

For a moving ground-based LiDAR in forest, "coverage adequacy" per voxel must
answer four questions:

1. **Spatial coverage** — did the beam reach this location at all?
2. **Density sufficiency** — enough returns to resolve structure?
3. **Angular diversity** — observed from multiple viewpoints? (occlusion)
4. **Range adequacy** — returns close enough to be reliable?

Most real-time tools do 1 and 2 only. 3 is where the scientific novelty lives
for canopy / understory work.

## Tier ladder considered

| Tier | Method | Verdict |
|---|---|---|
| 1 | Ground-plane hex density (deck.gl `HexagonLayer`) | v0 — ships, proves UI + pipeline |
| 2 | 3D voxel density, range-weighted | ignores viewpoint; subsumed by Tier 3 |
| 3 | Viewpoint diversity per voxel | **v1 — method paper** |
| 4 | Occlusion-aware ray tracing | too expensive for field hardware |

## Viewpoint-diversity metric — derivation

### First candidate: range-weighted mean pairwise angle

$$D_w(V) = \frac{\sum_{i<j} w_i w_j \arccos(\mathbf{d}_i \cdot \mathbf{d}_j)}
                {\sum_{i<j} w_i w_j}, \quad w_i = \exp(-\rho_i / \rho_0)$$

where $\mathbf{d}_i$ is the unit vector from voxel to sensor at return $i$,
$\rho_i$ is the range.

**Pressure-tested** against six canonical trajectories (see
[sim/diversity_sim.py](../sim/diversity_sim.py)). Failed on three counts:

1. `D_min = 17°` threshold useless — even a single straight pass scores ~57°
   because incidence sweeps a wide arc as the sensor moves past.
2. **Bimodality failure:** opposite parallel passes score 90.7° — *higher* than
   full loop (89.2°). Two antipodal clusters maximize mean pairwise angle but
   represent fewer distinct viewpoints than a loop.
3. Parallel passes (53°) score lower than single pass (57°) — more sampling on
   the same side concentrates the distribution, making the metric non-monotone
   in a counterintuitive way.

Mean pairwise angle is a second-order moment. It confuses *spread* with
*diversity*. Abandoned.

### Second candidate: solid-angle coverage (adopted for v1)

Bin the viewpoint directions onto a tessellation of $S^2$. Count occupied cells.

$$k(V) = \#\{c : \sum_{i \in c} w_i \ge W_{\min}\}$$

**Why this is the right metric:**

- Directly answers "from how many distinct directions was this voxel observed?"
- $O(N)$ per voxel — matches real-time budget
- Interpretable: "seen from 14 of 48 directions" is a field-operator statement
- No bimodality pathology: two antipodal clusters correctly register as two
  cells, not as "maximum diversity"

**Tessellation choice — pressure-tested in
[sim/solid_angle_sim.py](../sim/solid_angle_sim.py):**

| case           | icosa20 | hp12 | hp48  | az16   |
|----------------|---------|------|-------|--------|
| single pass    | 5/20    | 5/12 | 7/48  | 8/16   |
| parallel       | 5/20    | 5/12 | 7/48  | 8/16   |
| opposite       | 10/20   | 8/12 | 14/48 | 16/16  |
| crossing       | 8/20    | 7/12 | 11/48 | 12/16  |
| half loop      | 6/20    | 5/12 | 9/48  | 9/16   |
| full loop      | 10/20   | 8/12 | 16/48 | 16/16  |

Coarse tessellations (icosa20, hp12) *tie* opposite with full loop — the
viewpoint-diversity signal we need is below their resolution. **hp48 is the
smallest tessellation that separates them** (14 vs 16). It's also density-
invariant: going from 40 to 200 returns doesn't change $k$, which is correct —
more samples from the same angle isn't new information. This means $k$ must be
gated alongside a density count $N$, not used as a replacement.

### Stem case — azimuth-only coverage

For vertical stems, the relevant viewpoint geometry is the **horizontal plane**,
not full $S^2$. A stem observed from front and back is radially constrained;
full $S^2$ penalizes this as "only two cells" but azimuth coverage correctly
rewards it (opposite = 16/16 on az16).

This gives us **object-class-dependent adequacy** — detect verticality via local
PCA, switch tessellation accordingly. This is likely the strongest IP claim in
the method. *Patent review required before public release of v1 code.*

## v1 adequacy function (locked for the research thread)

$$A(V) = \mathbb{1}[N \ge N_{\min}] \cdot
        \mathbb{1}[k \ge k_{\min}] \cdot
        \mathbb{1}[\bar{\rho}_w \le \rho_{\max}]$$

Three-state field display:

- **Red:**    $N < N_{\min}$ OR $k < k_{\text{low}}$ — undersampled or one-sided
- **Orange:** $N \ge N_{\min}$ AND $k_{\text{low}} \le k < k_{\text{high}}$ — multi-angle but partial
- **Green:**  $N \ge N_{\min}$ AND $k \ge k_{\text{high}}$ — adequate

### v1 default parameters

| parameter | value | rationale |
|-----------|-------|-----------|
| voxel $\ell$ | 0.5 m | understory structural scale |
| $N_{\min}$ | 10 | enough for meaningful coverage |
| $\rho_{\max}$ | 20 m | Fairy reliable range |
| $\rho_0$ | 10 m | characteristic weighting scale |
| tessellation | HEALPix nside=2 (48 cells) | smallest that separates opposite/loop |
| $k_{\text{low}}$ (hp48) | 10 | bottom of orange band |
| $k_{\text{high}}$ (hp48) | 14 | threshold for green |
| stem tessellation | 16 azimuth bins | radial constraint only |
| $k_{\min}$ (az16) | 12 | green threshold for stems |
| $W_{\min}$ per cell | 0.1 | glancing-return floor |

All thresholds are config-exposed; field operator can tune.

## Open work for v1

- Noisy-trajectory robustness pass: Gaussian jitter, variable speed, occlusion
  dropouts. Does hp48 still separate, or do we need to drop to hp24?
- Stem detection: verticality gate on local point PCA. What eigenvalue ratio?
- Harvard Forest validation: real rosbags, plot-level coverage maps, correlate
  with downstream biomass/structure metrics.
- Voxel-store data structure: sparse hash-map vs. octree under the field compute
  budget.
- Publication prior-art clock: AGPL release starts prior-art timer. Talk to CCNY
  tech transfer about a provisional on object-class-dependent adequacy before
  first public commit of v1.
