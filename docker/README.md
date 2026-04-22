# docker/

Container image for the TARP bridge stack (rosbridge + replay + static web
server). Designed to run alongside the existing `lidar_slam` container on a
Jetson, both on `network_mode: host` so they share the DDS graph.

## Why a separate container

- Keeps SLAM and TARP deploys decoupled (different upgrade cadences).
- Avoids bloating the 14 GB `lidar_slam` image with a ~30 MB websocket
  dependency.
- Same base image (`dustynv/ros:humble-ros-base-l4t-r35.4.1`) as lidar_slam, so
  the big L4T layer is already on disk — the TARP image is effectively the
  delta.

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
sensor topics into the canonical `/tarp/points` and `/tarp/odom`. The web
client code stays untouched. Example for a Fairy + FAST-LIO2 stack:

```yaml
# in compose.yml, replacing replay:
relay-points:
  <<: *tarp-common
  command: ros2 run topic_tools relay /rslidar_points /tarp/points
relay-odom:
  <<: *tarp-common
  command: ros2 run topic_tools relay /Odometry      /tarp/odom
```

## Files

- `Dockerfile` — base + rosbridge_suite + topic_tools + numpy. Non-root `tarp`
  user; override UID/GID via build args to match the host user (prevents
  root-owned files from sneaking into bind-mounted edits).
- `entrypoint.sh` — sources `/opt/ros/humble/setup.bash`, execs the compose
  command.
