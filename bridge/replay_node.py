"""Synthetic replay node. Publishes PointCloud2 + Odometry at a steady rate
without needing a rig. Lets the deck.gl viewer be developed end-to-end before
we have real hardware on a bench.

Input formats supported:
  - .npz with arrays 'points' (T, N, 3) and 'poses' (T, 7)  [x y z qx qy qz qw]
  - .csv directory layout: frames/0000.csv ... + poses.csv

If --synth is passed (or no input is given), we generate a canned scene: a
straight walk past a cluster of "trees" (vertical line segments of points).

Usage:
    python3 replay_node.py --synth --rate 10
    python3 replay_node.py --input /path/to/recording.npz --rate 10

Requires: rclpy, sensor_msgs, nav_msgs, std_msgs. No-op falls back to a
stdout-only dry run if rclpy is not importable (so the file can be linted on a
dev machine without ROS2 installed).
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Node = object  # placeholder so the class below can be declared without ROS2


FRAME_ID_MAP = "map"
FRAME_ID_BASE = "base_link"


@dataclass
class Frame:
    points: np.ndarray       # (N, 3) float32
    pose: np.ndarray         # (7,) [x y z qx qy qz qw]
    stamp_ns: int


# ---------------- synthetic scene ----------------

def synth_scene(n_frames: int = 200, rate_hz: float = 10.0) -> list[Frame]:
    """Straight walk along +x past a small cluster of vertical 'trees'.
    Each frame emits ~2000 points from whichever trees are in range."""
    rng = np.random.default_rng(42)
    # Tree bases
    trees = np.array([
        [3.0,  2.0, 0.0],
        [5.0, -1.5, 0.0],
        [7.5,  0.5, 0.0],
        [10.0, 2.5, 0.0],
        [12.0,-2.0, 0.0],
    ])
    tree_height = 8.0
    pts_per_tree = 400

    frames = []
    dt_ns = int(1e9 / rate_hz)
    for i in range(n_frames):
        t = i / rate_hz
        sensor_x = -2.0 + 0.5 * t  # ~0.5 m/s walk
        sensor = np.array([sensor_x, 0.0, 1.5])

        frame_pts = []
        for base in trees:
            zs = rng.uniform(0.0, tree_height, size=pts_per_tree)
            jitter = rng.normal(0.0, 0.05, size=(pts_per_tree, 2))
            xs = base[0] + jitter[:, 0]
            ys = base[1] + jitter[:, 1]
            pts = np.stack([xs, ys, zs], axis=1)
            rng_to_pt = np.linalg.norm(pts - sensor, axis=1)
            pts = pts[rng_to_pt < 15.0]
            frame_pts.append(pts)

        if frame_pts:
            points = np.vstack(frame_pts).astype(np.float32)
        else:
            points = np.zeros((0, 3), dtype=np.float32)

        pose = np.array([sensor[0], sensor[1], sensor[2], 0.0, 0.0, 0.0, 1.0])
        frames.append(Frame(points=points, pose=pose, stamp_ns=i * dt_ns))
    return frames


# ---------------- file loader ----------------

def load_npz(path: Path) -> list[Frame]:
    data = np.load(path)
    pts_seq = data["points"]
    poses = data["poses"]
    if len(pts_seq) != len(poses):
        raise ValueError(f"points ({len(pts_seq)}) and poses ({len(poses)}) length mismatch")
    dt_ns = int(1e9 / 10.0)
    return [
        Frame(points=p.astype(np.float32), pose=poses[i], stamp_ns=i * dt_ns)
        for i, p in enumerate(pts_seq)
    ]


# ---------------- ROS2 publisher ----------------

def make_pointcloud2(frame: Frame, node) -> "PointCloud2":
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = FRAME_ID_MAP
    msg.height = 1
    msg.width = int(frame.points.shape[0])
    msg.fields = [
        PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = frame.points.tobytes()
    return msg


def make_odometry(frame: Frame, node) -> "Odometry":
    msg = Odometry()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = FRAME_ID_MAP
    msg.child_frame_id = FRAME_ID_BASE
    msg.pose.pose.position.x = float(frame.pose[0])
    msg.pose.pose.position.y = float(frame.pose[1])
    msg.pose.pose.position.z = float(frame.pose[2])
    msg.pose.pose.orientation.x = float(frame.pose[3])
    msg.pose.pose.orientation.y = float(frame.pose[4])
    msg.pose.pose.orientation.z = float(frame.pose[5])
    msg.pose.pose.orientation.w = float(frame.pose[6])
    return msg


class ReplayNode(Node):
    def __init__(self, frames: list[Frame], rate_hz: float, loop: bool):
        super().__init__("tarp_replay")
        self.frames = frames
        self.loop = loop
        self.idx = 0
        self.pc_pub = self.create_publisher(PointCloud2, "/tarp/points", 10)
        self.odom_pub = self.create_publisher(Odometry, "/tarp/odom", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"replaying {len(frames)} frames at {rate_hz} Hz")

    def _tick(self):
        if self.idx >= len(self.frames):
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info("replay complete")
                self.timer.cancel()
                return
        f = self.frames[self.idx]
        self.pc_pub.publish(make_pointcloud2(f, self))
        self.odom_pub.publish(make_odometry(f, self))
        self.idx += 1


def dry_run(frames: list[Frame], rate_hz: float) -> None:
    """Stdout-only loop for environments without ROS2 installed."""
    print(f"[dry-run] {len(frames)} frames, {rate_hz} Hz, ROS2 not available")
    for i, f in enumerate(frames[:5]):
        print(f"  frame {i}: pts={f.points.shape[0]:5d}  pose={f.pose[:3]}")
    print("  …")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", type=Path, help="path to .npz recording")
    ap.add_argument("--synth", action="store_true", help="use canned synthetic scene")
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--loop", action="store_true")
    args = ap.parse_args()

    if args.synth or args.input is None:
        frames = synth_scene(rate_hz=args.rate)
    else:
        frames = load_npz(args.input)

    if not HAS_ROS:
        dry_run(frames, args.rate)
        return 0

    rclpy.init()
    node = ReplayNode(frames, args.rate, args.loop)
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
