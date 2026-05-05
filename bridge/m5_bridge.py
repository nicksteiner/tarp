"""TARP <-> M5Stack Core2 (m5tough) serial bridge.

The Core2 acts as a tactile, glanceable companion to the deck.gl viewer:
the operator can keep the tablet in a pocket and still see pose/path/recording
state on the wrist-mounted Core2, plus toggle record/waypoint/anomaly with
chunky physical buttons that survive cold + gloves.

Wire: USB-CDC, 115200-8N1. The Core2's `Serial` is exposed on the same USB
cable used to flash it. On the Jetson we expect `/dev/ttyACM0` or similar.

Wire protocol (ASCII, '\\n'-terminated, lossy-tolerant):

  Host -> device:
    P <x> <y> <z>            pose (m), updated at odom rate
    S <path_m> <map_pts>     stats, updated at ~1 Hz
    R <0|1>                  recording state
    L <online|offline>       link/health indicator for the header

  Device -> host:
    BTN START                operator pressed START
    BTN STOP                 operator pressed STOP
    WPT                      waypoint requested (future)
    FLAG                     anomaly flagged (future)
    booted                   firmware reset notification

The protocol is ASCII on purpose — the M5 firmware already prints `BTN START`
verbatim, and the Jetson side stays debuggable with `cat /dev/ttyACM0` while
both sides are being brought up.

This node:
  * subscribes to /tarp/odom (republished by slam_bridge or replay_node);
  * computes path length and publishes stats at ~1 Hz;
  * sends pose at the odom rate (10 Hz typical, throttled if higher);
  * publishes /tarp/cmd/record (std_msgs/Bool) on BTN START/STOP so the
    rest of the stack can react (rosbag triggers will hang off this later).

Run separately:
    python3 bridge/m5_bridge.py --port /dev/ttyACM0

If the serial port is missing, the node logs a warning every 5 s and keeps
trying — this matches how the operator may plug the Core2 in mid-mission.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Node = object

try:
    import serial  # pyserial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


POSE_THROTTLE_HZ = 10.0
STATS_HZ = 1.0


class M5Serial:
    """Best-effort serial transport. Re-opens the port on failure rather than
    crashing the node — the Core2 may be unplugged during a mission."""

    def __init__(self, port: str, baud: int, logger):
        self.port = port
        self.baud = baud
        self.logger = logger
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.last_open_attempt = 0.0
        self.reopen_interval = 5.0

    def _try_open(self) -> bool:
        now = time.monotonic()
        if now - self.last_open_attempt < self.reopen_interval:
            return False
        self.last_open_attempt = now
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.logger.info(f"m5 serial open: {self.port} @ {self.baud}")
            return True
        except (serial.SerialException, OSError) as e:
            self.logger.warning(f"m5 serial open failed ({self.port}): {e}")
            self.ser = None
            return False

    def write_line(self, line: str) -> None:
        with self.lock:
            if self.ser is None and not self._try_open():
                return
            try:
                assert self.ser is not None
                self.ser.write((line + "\n").encode("ascii", errors="replace"))
            except (serial.SerialException, OSError) as e:
                self.logger.warning(f"m5 serial write failed: {e}; will reopen")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def readline(self) -> Optional[str]:
        with self.lock:
            if self.ser is None and not self._try_open():
                return None
            try:
                assert self.ser is not None
                data = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.logger.warning(f"m5 serial read failed: {e}; will reopen")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return None
        if not data:
            return None
        try:
            return data.decode("ascii", errors="replace").strip()
        except Exception:
            return None


class M5BridgeNode(Node):
    def __init__(self, port: str, baud: int):
        super().__init__("tarp_m5_bridge")
        self.serial = M5Serial(port, baud, self.get_logger())

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.last_pose: Optional[tuple[float, float, float]] = None
        self.path_m = 0.0
        self.map_pts = 0
        self.recording = False
        self.last_pose_send = 0.0

        self.create_subscription(Odometry, "/tarp/odom", self._on_odom, odom_qos)
        self.create_subscription(
            PointCloud2, "/tarp/points", self._on_cloud, cloud_qos,
        )

        self.cmd_record_pub = self.create_publisher(Bool, "/tarp/cmd/record", 10)
        self.cmd_event_pub = self.create_publisher(String, "/tarp/cmd/event", 10)

        self.create_timer(1.0 / STATS_HZ, self._stats_tick)
        self.create_timer(0.05, self._serial_pump)
        self.create_timer(2.0, self._link_tick)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        cur = (float(p.x), float(p.y), float(p.z))
        if self.last_pose is not None:
            dx = cur[0] - self.last_pose[0]
            dy = cur[1] - self.last_pose[1]
            dz = cur[2] - self.last_pose[2]
            self.path_m += math.sqrt(dx * dx + dy * dy + dz * dz)
        self.last_pose = cur

        now = time.monotonic()
        if now - self.last_pose_send >= 1.0 / POSE_THROTTLE_HZ:
            self.last_pose_send = now
            self.serial.write_line(f"P {cur[0]:.3f} {cur[1]:.3f} {cur[2]:.3f}")

    def _on_cloud(self, msg: PointCloud2) -> None:
        # Best-effort: accumulate map cardinality. The decimated cloud carries
        # the per-frame point budget, not the full map count, but it's the
        # only signal available to this node and is good enough to drive the
        # "map ___ pts" indicator on the Core2.
        self.map_pts += int(msg.width * msg.height)

    def _stats_tick(self) -> None:
        self.serial.write_line(f"S {self.path_m:.2f} {self.map_pts}")

    def _link_tick(self) -> None:
        # Heartbeat — keeps the Core2 header label fresh even if odom drops.
        self.serial.write_line("L online")

    def _serial_pump(self) -> None:
        line = self.serial.readline()
        if not line:
            return
        if line == "BTN START":
            self.recording = True
            self.cmd_record_pub.publish(Bool(data=True))
            self.serial.write_line("R 1")
        elif line == "BTN STOP":
            self.recording = False
            self.cmd_record_pub.publish(Bool(data=False))
            self.serial.write_line("R 0")
        elif line == "WPT":
            self.cmd_event_pub.publish(String(data="waypoint"))
        elif line == "FLAG":
            self.cmd_event_pub.publish(String(data="flag"))
        elif line == "booted":
            # Core2 just (re)started — push current state so the screen
            # isn't left showing zeros.
            self.serial.write_line(f"R {1 if self.recording else 0}")
            self.serial.write_line(f"S {self.path_m:.2f} {self.map_pts}")
            self.serial.write_line("L online")
        # Unknown lines are ignored — the Core2 also prints free-form debug
        # to Serial, and we don't want to crash on it.


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0", help="USB-CDC serial port")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    if not HAS_ROS:
        print("[m5_bridge] rclpy not importable — needs ROS2 Humble.",
              file=sys.stderr)
        return 2
    if not HAS_SERIAL:
        print("[m5_bridge] pyserial not importable — `pip3 install pyserial`.",
              file=sys.stderr)
        return 2

    rclpy.init()
    node = M5BridgeNode(args.port, args.baud)
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
