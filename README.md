# TARP — Topographic Adequacy Real-time Profiler

Web-first, mobile-friendly real-time LiDAR collection monitoring. Answers the
field question: **"have I collected enough point cloud data in this region to
trust the output?"** — before leaving the plot.

## Status

**v0 — field tool.** Synthetic-replay scaffolding, deck.gl viewer, hex coverage
heatmap over a trajectory. Does not compute the v1 adequacy metric yet.

## Architecture

- **Backend:** ROS2 host running `rosbridge_suite` (WebSocket, port 9090)
- **Frontend:** single-page web app, vanilla JS + deck.gl, served from the SLAM host
- **Transport:** roslibjs over rosbridge
- **Target:** iPad/tablet browser on local hotspot — no internet required
- **Primary sensor:** RoboSense Fairy on a FAST-LIO2 / similar stack

## Repo layout

```
bridge/            Python replay + bridge nodes (rosbridge publishers)
web/               deck.gl viewer (SPA)
sim/               Algorithm simulations (solid-angle coverage, diversity)
docs/              Method notes and design docs
scripts/           Dev helpers
```

## License

AGPL-3.0-or-later. See [LICENSE](LICENSE).

## Ecosystem

Companion tool to SARdine (SAR visualization). Steiner Lab / Earth Big Data LLC.
