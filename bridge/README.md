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

## Running with rosbridge

```
# one terminal
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# another terminal
python3 bridge/replay_node.py --synth --loop
```

The deck.gl viewer in [../web/](../web/) connects to `ws://<host>:9090` and
subscribes to the two topics above.
