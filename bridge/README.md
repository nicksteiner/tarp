# bridge/

Python nodes for the ROS2 host side of TARP.

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

## `m5_bridge.py`

Host side of the M5Stack Core2 tactile companion display protocol — see
firmware at `~/m5tough/src/main.cpp` for the device-side line-format docs.

The bridge auto-detects the Core2 USB-CDC port by VID:PID, drives
`P / S / R / L / U / D` lines from rclpy subscriptions on `/tarp/odom`,
`/tarp/points`, and `/tarp/recording_state`, and re-publishes button events
+ session ids onto `/tarp/recording_request` (Bool) and
`/tarp/session_id` (UInt32).

`BTN DUMP` from the device rsyncs the most-recently-modified subdir of
`--bags-root` into `<usb_mount>/tarp_session_<n>_<bag-name>/`, sending
`D busy` → `D ok | D fail` back to the device. USB drive fill is reported
every 5 s as `U <pct>` (or `U -1` when no drive is mounted under
`/media`, `/mnt`, or `/run/media`).

```
# typical: rclpy + autodetected port + default ~/.ros/bags as bag root
python3 bridge/m5_bridge.py

# explicit port
python3 bridge/m5_bridge.py --port /dev/ttyACM0

# bench bring-up: stdout-only (no Core2, no ROS2 — just prints what would
# go to the device, useful to confirm sim cadence and USB polling)
python3 bridge/m5_bridge.py --no-port --no-ros

# real device, no ROS yet (still drives Lissajous sim into the Core2)
python3 bridge/m5_bridge.py --no-ros
```

Tests (no hardware required): `python3 -m pytest bridge/test_m5_bridge.py -v`

## Running with rosbridge

**Docker (Jetson / preferred):** see [../docker/README.md](../docker/README.md)
and the top-level `compose.yml`. `docker compose up` brings rosbridge, the
replay node, and the static web server up together.

**Bare metal (dev box with ROS2 installed):**

```
# one terminal
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# another terminal
python3 bridge/replay_node.py --synth --loop
```

The deck.gl viewer in [../web/](../web/) connects to `ws://<host>:9090` and
subscribes to the two topics above.
