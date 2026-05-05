"""SLAM → TARP bridge node.

Subscribes to whichever SLAM stack the lidar_slam container is running and
republishes its cloud + odometry on the two viewer-facing topics
(`/tarp/points`, `/tarp/odom`). The deck.gl viewer is hardcoded to those two
topics on purpose — keeping the renaming inside this node lets the front-end
stay sensor-agnostic.

Sources (selectable with --source):
  fastlio_fairy  — `/fastlio_fairy/cloud_registered`, `/fastlio_fairy/Odometry`
  liosam         — `/liosam/lio_sam/mapping/cloud_registered`,
                   `/liosam/lio_sam/mapping/odometry`

Cloud decimation: the lidar_slam stack publishes ~600k points/sec on the
RoboSense Fairy. The web viewer budget (per CLAUDE.md) is ~5k pts per message
at 10 Hz. We stride-sample to that budget and strip everything except xyz to
match the viewer's PointCloud2 decoder (`web/src/ros.js`).

Run separately on a dev box:
    python3 bridge/slam_bridge.py --source fastlio_fairy --max-points 5000

Inside docker compose, see compose.yml service `slam_bridge`.
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from nav_msgs.msg import Odometry
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Node = object


VIEWER_FRAME = "map"
VIEWER_BASE = "base_link"


@dataclass(frozen=True)
class Source:
    name: str
    cloud_topic: str
    odom_topic: str


SOURCES: dict[str, Source] = {
    "fastlio_fairy": Source(
        "fastlio_fairy",
        "/fastlio_fairy/cloud_registered",
        "/fastlio_fairy/Odometry",
    ),
    "liosam": Source(
        "liosam",
        "/liosam/lio_sam/mapping/cloud_registered",
        "/liosam/lio_sam/mapping/odometry",
    ),
}


def decimate_cloud_xyz(msg: "PointCloud2", max_points: int) -> bytes:
    """Stride-sample a PointCloud2 down to xyz-only float32 at <= max_points.

    The upstream cloud may carry intensity/ring/timestamp fields, but the web
    viewer only reads xyz. Returning a flat little-endian float32 xyz buffer
    lets us reuse the existing decoder without per-source branching.
    """
    n_in = msg.width * msg.height
    if n_in == 0:
        return b""

    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return b""
    fx, fy, fz = fields["x"], fields["y"], fields["z"]

    raw = bytes(msg.data)
    step = msg.point_step
    stride = max(1, (n_in + max_points - 1) // max_points)

    out = bytearray()
    n_out = 0
    little = "<" if not msg.is_bigendian else ">"
    fmt_x = f"{little}f"
    fmt_y = f"{little}f"
    fmt_z = f"{little}f"
    for i in range(0, n_in, stride):
        base = i * step
        x = struct.unpack_from(fmt_x, raw, base + fx.offset)[0]
        y = struct.unpack_from(fmt_y, raw, base + fy.offset)[0]
        z = struct.unpack_from(fmt_z, raw, base + fz.offset)[0]
        out += struct.pack("<fff", x, y, z)
        n_out += 1
        if n_out >= max_points:
            break
    return bytes(out)


def make_viewer_pointcloud2(node, raw_xyz: bytes, n: int, stamp) -> "PointCloud2":
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = VIEWER_FRAME
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.is_dense = True
    msg.data = raw_xyz
    return msg


class SlamBridgeNode(Node):
    def __init__(self, source: Source, max_points: int):
        super().__init__("tarp_slam_bridge")
        self.source = source
        self.max_points = max_points

        # Sensor-data QoS for the cloud (matches FAST-LIO/LIO-SAM publishers).
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Reliable for odometry (matches typical SLAM publishers).
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pc_sub = self.create_subscription(
            PointCloud2, source.cloud_topic, self._on_cloud, cloud_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry, source.odom_topic, self._on_odom, odom_qos,
        )
        self.pc_pub = self.create_publisher(PointCloud2, "/tarp/points", 10)
        self.odom_pub = self.create_publisher(Odometry, "/tarp/odom", 10)

        self.get_logger().info(
            f"slam_bridge source={source.name} "
            f"cloud={source.cloud_topic} odom={source.odom_topic} "
            f"max_points={max_points}"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        raw = decimate_cloud_xyz(msg, self.max_points)
        n = len(raw) // 12
        if n == 0:
            return
        out = make_viewer_pointcloud2(self, raw, n, msg.header.stamp)
        self.pc_pub.publish(out)

    def _on_odom(self, msg: Odometry) -> None:
        # Frame-rewrite so the viewer always sees `map`. The SLAM stack
        # already publishes pose in its own `odom`/`map`-equivalent frame,
        # so we only relabel — no transform applied.
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = VIEWER_FRAME
        out.child_frame_id = VIEWER_BASE
        out.pose = msg.pose
        out.twist = msg.twist
        self.odom_pub.publish(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--source", choices=sorted(SOURCES.keys()), default="fastlio_fairy",
        help="which SLAM stack to bridge from",
    )
    ap.add_argument(
        "--max-points", type=int, default=5000,
        help="upper bound on points per republished cloud (web viewer budget)",
    )
    args = ap.parse_args()

    if not HAS_ROS:
        print(
            "[slam_bridge] rclpy not importable — this node only runs in the "
            "tarp container or on a host with ROS2 Humble installed.",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = SlamBridgeNode(SOURCES[args.source], args.max_points)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
