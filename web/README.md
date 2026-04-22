# web/

deck.gl single-page viewer. No build step. Third-party libraries are bundled
in [vendor/](vendor/) so the tool needs no internet at runtime — the whole
thing is static files served from the ROS2 host.

## Layers

- `PointCloudLayer` — live decimated returns (cap `MAX_LIVE_POINTS` in main.js)
- `HexagonLayer`    — ground-plane coverage heatmap, density only (Tier 1)
- `PathLayer`       — odometry trajectory

## Dev loop

```
# from repo root
python3 scripts/serve.py         # serves web/ on :8000
python3 bridge/replay_node.py --synth --loop
# open http://localhost:8000 on the iPad on the same hotspot
```

Rosbridge must be running on port 9090 on the same host that serves the page
(the viewer derives the WebSocket URL from `location.hostname`).

## Not in v0

The v1 adequacy metric — solid-angle coverage on HEALPix nside=2 — does **not**
run here. Algorithm work is in [../sim/](../sim/) and the research-thread notes
are in [../docs/METHOD_NOTES.md](../docs/METHOD_NOTES.md).
