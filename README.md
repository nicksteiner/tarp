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
docker/            Container image for the bridge stack
compose.yml        Docker Compose service definitions
```

## Bring-up (Jetson)

The SLAM container (`lidar_slam`) uses `network_mode: host`. The TARP bridge
container does too, so DDS discovery works without extra config.

```bash
docker compose build
docker compose up            # rosbridge + replay + web server
# then, on the tablet: http://<jetson-ip>:8000
```

See [docker/README.md](docker/README.md) for the swap-in path once real
hardware replaces the synthetic replay.

## License

AGPL-3.0-or-later. See [LICENSE](LICENSE).

## Ecosystem

Companion tool to SARdine (SAR visualization). Steiner Lab / Earth Big Data LLC.
