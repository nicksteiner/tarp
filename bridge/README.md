# bridge/

Python nodes for the ROS2 host side of TARP. Three nodes today:
`replay_node.py` (synthetic), `slam_bridge.py` (real SLAM → viewer), and
`m5_bridge.py` (USB serial link to the M5Stack Core2).

## `replay_node.py`

Synthetic replay. Publishes:

- `/tarp/points`  `sensor_msgs/PointCloud2`   (xyz float32)
- `/tarp/odom`    `nav_msgs/Odometry`

Runs from either a canned synthetic scene or a `.npz` recording with `points`
(T,N,3) and `poses` (T,7) arrays.

```
# synthetic walk past 5 trees
python3 replay_node.py --synth --rate 10 --loop

# file-backed replay
python3 replay_node.py --input /path/to/recording.npz --rate 10
```

On a dev machine without ROS2 installed, it falls back to a stdout dry-run so
syntax / data loading can be tested.

## `slam_bridge.py`

Real-data path. Subscribes to whichever SLAM stack the lidar_slam container is
running, decimates the cloud to the web viewer's per-message budget (5k pts
default), and republishes on the same `/tarp/points` + `/tarp/odom` topics the
viewer already speaks. Keeping the renaming inside this node lets `web/` stay
sensor-agnostic.

Source aliases:

| `--source`        | Cloud topic                                       | Odom topic                                  |
|-------------------|---------------------------------------------------|---------------------------------------------|
| `fastlio_fairy`   | `/fastlio_fairy/cloud_registered`                 | `/fastlio_fairy/Odometry`                   |
| `liosam`          | `/liosam/lio_sam/mapping/cloud_registered`        | `/liosam/lio_sam/mapping/odometry`          |

The `liosam` source becomes useful once `lidar_slam` D102 (Dockerfile.liosam)
and D103 (Fairy patches) land — see
`~/lidar_slam/docs/LIO_SAM_INTEGRATION_PLAN.md`.

```
python3 bridge/slam_bridge.py --source fastlio_fairy --max-points 5000
```

`replay_node` and `slam_bridge` are mutually exclusive — both publish on
`/tarp/points` and `/tarp/odom`. Compose runs them under separate profiles
(`synth` and `live`) for that reason.

## `m5_bridge.py`

USB-CDC serial bridge between TARP and the M5Stack Core2 (`~/m5tough`). Lets
the operator keep the tablet in a pocket and still see pose / path / recording
state on a wrist-mounted Core2 with chunky buttons that survive cold + gloves.

```
python3 bridge/m5_bridge.py --port /dev/ttyACM0
```

If the serial device is missing the node logs and keeps retrying every 5 s, so
the operator can plug the Core2 in mid-mission without restarting the bridge.

### Wire protocol

ASCII, `\n`-terminated, lossy-tolerant. The Core2 firmware ignores unknown
lines so the same stream can carry free-form debug.

**Host → device** (this node emits):

| Line                   | Meaning                                        |
|------------------------|------------------------------------------------|
| `P <x> <y> <z>`        | pose update in metres (throttled to 10 Hz)     |
| `S <path_m> <map_pts>` | path length + map cardinality (~1 Hz)          |
| `R <0\|1>`             | recording state                                |
| `L online`             | link heartbeat (~0.5 Hz)                       |

**Device → host** (Core2 firmware emits):

| Line          | Meaning                                       |
|---------------|-----------------------------------------------|
| `booted`      | firmware just (re)started — host re-syncs     |
| `BTN START`   | operator pressed START                        |
| `BTN STOP`    | operator pressed STOP                         |
| `WPT`         | waypoint requested (firmware not wired yet)   |
| `FLAG`        | anomaly flagged (firmware not wired yet)      |

### Republished topics

| Topic              | Type            | Notes                                       |
|--------------------|-----------------|---------------------------------------------|
| `/tarp/cmd/record` | `std_msgs/Bool` | flips on `BTN START` / `BTN STOP`           |
| `/tarp/cmd/event`  | `std_msgs/String` | `"waypoint"` or `"flag"` on the equivalent device line |

`rosbag record` triggering hangs off `/tarp/cmd/record`; that wiring lives in
a follow-on directive (no work yet).

## Running with rosbridge

**Docker (Jetson / preferred):** see [../docker/README.md](../docker/README.md)
and the top-level `compose.yml`. Use the compose profiles:

```
# synthetic data, dev box
docker compose --profile synth up

# real SLAM + Core2
TARP_M5_PORT=/dev/ttyACM0 docker compose --profile live up
```

The `rosbridge` and `webserver` services have no profile and come up in either
mode.

**Bare metal (dev box with ROS2 installed):**

```
# one terminal
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# another terminal — pick one
python3 bridge/replay_node.py --synth --loop
python3 bridge/slam_bridge.py --source fastlio_fairy
# optional
python3 bridge/m5_bridge.py --port /dev/ttyACM0
```

The deck.gl viewer in [../web/](../web/) connects to `ws://<host>:9090` and
subscribes to `/tarp/points` and `/tarp/odom`.
