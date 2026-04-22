# TARP — Claude context

## What this is

Real-time LiDAR collection monitor. Field operator sees live point cloud,
trajectory, and ground-plane coverage heatmap on a tablet. Answers: have I
collected enough to trust the output from this plot?

## v0 scope (locked)

**In:**
- Synthetic replay node (publishes PointCloud2 + Odometry from a file) — lets UI
  work be developed without a rig
- Rosbridge WebSocket bridge on port 9090
- deck.gl SPA: PointCloudLayer (decimated), HexagonLayer (ground-plane count),
  PathLayer (trajectory)
- Touch-first controls: record start/stop, waypoint, anomaly flag
- No external network required at runtime. All JS/CSS bundled locally.
  No CDN fetches, no telemetry, no Google Fonts, no analytics.

**Out (do not implement without asking):**
- v1 angular-diversity / solid-angle adequacy metric — research thread, fall/winter 2026
- Object-class-dependent adequacy (stems vs voxels)
- Post-processing, cloud upload, multi-rig fusion
- Any rendering path that isn't deck.gl

## v1 research thread (separate)

Method paper target. See [docs/METHOD_NOTES.md](docs/METHOD_NOTES.md) for the
full derivation + simulation results. Short version:

- Adequacy = voxel density gate AND viewpoint-coverage gate
- Coverage = occupied cells on a HEALPix nside=2 (48-cell) tessellation of the
  unit sphere, range-weighted
- Stem-class voxels use 16-bin azimuth coverage instead of full S^2
- v1 thresholds (defaults, all tunable): N≥10, k_hp48≥14, k_az16≥12

v1 validates against Harvard Forest. Do not merge v1 into the v0 field tool
without an explicit green-light from Nick.

## Platform conventions

- Python for v0 bridge / orchestration / replay / I/O (`rclpy`, rosbridge)
- v1 adequacy engine is Rust, exposed to Python via PyO3 (crate name
  `tarp-core`). Voxel store, HEALPix binning, and adequacy gates live there —
  not in Python. Python calls into the crate from a `rclpy` node.
- `sim/` stays Python/numpy for research prototyping. Port to Rust only after
  the algorithm is validated.
- Do not add Rust to the tree until v1 work starts. No empty placeholder crate.
- ROS2 frames per REP-105: `map` → `odom` → `base_link`
- Client display budget: ~50k points accumulated in view, decimated from
  sensor raw (~600k pts/sec). Per-message size target: ~5k points at 10 Hz.
- WebSocket bandwidth budget: assume tethered hotspot, not gigabit
- deck.gl on WebGL2. WebGPU deferred until HexagonLayer and picking land
  in deck.gl (not v0 scope). See docs/METHOD_NOTES.md for decision log.


## Working rules for Claude

- Don't add v1 metric code to `bridge/` or `web/`. Algorithm work lives in `sim/`
  until validated.
- Don't add dependencies without asking — v0 should stay light (numpy, roslibjs,
  deck.gl, that's ~it).
- Don't create planning/status docs unless asked. Method derivations belong in
  `docs/METHOD_NOTES.md`; everything else is in code or PR descriptions.
- If something needs a ROS2 host to verify, say so — don't claim "tested" when
  you only ran syntax checks.
  - Scope creep is the main risk. If a v0 task suggests v1 metric work
  would help, flag it and ask. Do not pre-implement v1 in v0 files.

