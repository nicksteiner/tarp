# docker/

Container image for the TARP bridge stack (rosbridge + replay + static web
server). Designed to run alongside the existing `lidar_slam` container on a
Jetson, both on `network_mode: host` so they share the DDS graph.

## Why a separate container

- Keeps SLAM and TARP deploys decoupled (different upgrade cadences).
- Avoids bloating the `lidar_slam` image with a websocket dependency it doesn't
  need.
- Same base image (`dustynv/ros:humble-ros-base-l4t-r35.4.1`) as lidar_slam, so
  the big L4T layer is already on disk — the TARP image is effectively the
  delta.

## What's in the image

The dustynv base runs Humble **built from source** on top of Ubuntu 20.04
(focal). The OSRF apt repo doesn't ship Humble debs for focal — Humble only
targets Jammy (22.04) — so we can't `apt install ros-humble-rosbridge-suite`.
Instead:

1. Refresh the ROS2 apt signing key (rotated 2025-05; the base predates it).
2. Install the Python runtime deps rosbridge needs from focal universe: bson,
   ujson, tornado, pil, numpy. One package — cbor2 — isn't in focal apt
   (jammy-only), so it comes from pip (same answer rosdep gives on focal).
3. Clone `rosbridge_suite` tag `2.0.1` into `/ws/src`,
   `colcon build --packages-up-to rosbridge_suite --merge-install
   --cmake-args -DBUILD_TESTING=OFF`. Pinned to 2.0.1 because 2.0.2+
   introduced Python 3.10-only syntax and the dustynv base is on Python 3.8.
4. Entrypoint sources the base `/opt/ros/humble/install/setup.bash` and then
   the overlay `/ws/install/setup.bash`.

First build on the Jetson is ~3–4 min (colcon compile). Cached after that.

`topic_tools` is intentionally **not** in the image for v0 bring-up — the
replay node publishes directly on `/tarp/points` and `/tarp/odom`, no relay
needed. When real hardware lands and we want to remap sensor topics, add
`ros2/topic_tools` to the source clone in the Dockerfile.

## Usage

From the repo root:

```bash
docker compose build
docker compose up              # all three services
# or run them individually:
docker compose up rosbridge    # port 9090
docker compose up replay
docker compose up webserver    # port 8000
```

The compose file bind-mounts the repo into `/tarp`, so edits on the host are
live inside the container — no rebuild needed for Python or web changes.

## When real hardware lands

Replace the `replay` service with `topic_tools relay` calls that remap the real
sensor topics into the canonical `/tarp/points` and `/tarp/odom`. This requires
adding `topic_tools` to the Dockerfile's source clone:

```dockerfile
# in the git clone block:
 && git clone --depth 1 --branch humble https://github.com/ros2/topic_tools.git
# and to the colcon build:
 --packages-up-to rosbridge_suite topic_tools
```

Then in `compose.yml`, replace the `replay` service with:

```yaml
relay-points:
  <<: *tarp-common
  command: ros2 run topic_tools relay /rslidar_points /tarp/points
relay-odom:
  <<: *tarp-common
  command: ros2 run topic_tools relay /Odometry      /tarp/odom
```

The web client code stays untouched — the whole point of the canonical
`/tarp/*` topic names.

## Files

- `Dockerfile` — dustynv base + apt key refresh + focal-side Python deps +
  source-built rosbridge_suite overlay. Non-root `tarp` user; override UID/GID
  via build args to match the host user.
- `entrypoint.sh` — sources `/opt/ros/humble/install/setup.bash` then
  `/ws/install/setup.bash`, execs the compose command.
