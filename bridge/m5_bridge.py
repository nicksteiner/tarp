#!/usr/bin/env python3
"""Host-side bridge for the M5Stack Core2 tactile companion display.

Line-based ASCII protocol over USB-CDC at 115200 baud (see firmware
~/m5tough/src/main.cpp for the device side):

  Host -> device:
    P <x> <y> <z>         pose (m), high-rate (10 Hz)
    S <path_m> <map_pts>  stats (~1 Hz)
    R <0|1>               recording state
    L online              link heartbeat (~2 Hz)
    D <state>             dump status: idle | busy | ok | fail
    U <pct>               USB-drive fill 0-100 (or -1 for no drive)

  Device -> host:
    booted              once on reset
    BTN START           instant on press
    BTN STOP            on hold-to-confirm
    BTN DUMP            on hold-to-confirm
    SESS <n>            after each START — persisted session id

The bridge:
  * Auto-detects the Core2 USB-CDC port by VID:PID (CH9102F / CP210x).
  * Subscribes to /tarp/odom (nav_msgs/Odometry) and /tarp/points
    (sensor_msgs/PointCloud2) to drive P / S out to the device.
  * Subscribes to /tarp/recording_state (std_msgs/Bool) to drive R.
  * Publishes /tarp/recording_request (Bool) and /tarp/session_id
    (UInt32) so a recording node can react to button presses.
  * Sends an "L online" heartbeat at 2 Hz so the device shows LNK.
  * On BTN DUMP: locates a USB drive via /proc/mounts, rsyncs the
    most-recently-modified subdir of --bags-root into
    <mount>/tarp_session_<n>_<bag-name>/, echoes D busy -> ok | fail.
  * Emits U <pct> with df fill of the USB drive every 5 s
    (U -1 when no drive is mounted).

Dependencies (per ~/tarp/CLAUDE.md): pyserial added; rclpy optional.
With --no-ros, runs a Lissajous sim pose loop so the device can be
exercised on a bench without ROS2 installed.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, UInt32
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Node = object  # placeholder so the class can be declared

LOG = logging.getLogger("tarp.m5_bridge")


# ---------------- port autodetect ----------------

# USB-serial chips that ship on M5Stack Core2 hardware. The Core2 V1.1 uses
# CH9102F; older units use CP2102. CH340 covers ESP32 dev boards if someone
# bridges through one for testing.
M5_VID_PID = [
    (0x1A86, 0x55D4),  # CH9102F (current Core2)
    (0x1A86, 0x55D2),  # CH9102F (variant)
    (0x1A86, 0x7523),  # CH340
    (0x10C4, 0xEA60),  # CP2102 (older Core2)
]


def find_m5_port() -> Optional[str]:
    """First attached Core2 device path, by USB VID:PID, or None."""
    if list_ports is None:
        return None
    for p in list_ports.comports():
        if p.vid is None or p.pid is None:
            continue
        for vid, pid in M5_VID_PID:
            if p.vid == vid and p.pid == pid:
                return p.device
    return None


# ---------------- USB drive discovery ----------------

# Mountpoint roots where removable media typically appears on Linux. Anything
# mounted directly under one of these is treated as a candidate USB drive.
USB_MOUNT_ROOTS = ("/media", "/mnt", "/run/media")


def parse_mounts(mounts_text: str,
                 roots: Iterable[str] = USB_MOUNT_ROOTS) -> list[str]:
    """Return mountpoints under one of `roots` from /proc/mounts text.

    /proc/mounts escapes spaces as \\040 and tabs as \\011 in its
    space-separated fields, so we decode those before matching.
    """
    out = []
    rs = [r.rstrip("/") for r in roots]
    for line in mounts_text.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        mp = parts[1].replace("\\040", " ").replace("\\011", "\t")
        for root in rs:
            if mp.startswith(root + "/") and mp != root:
                out.append(mp)
                break
    return out


def find_usb_drive(roots: Iterable[str] = USB_MOUNT_ROOTS) -> Optional[str]:
    """First mounted USB drive path, or None."""
    try:
        text = Path("/proc/mounts").read_text()
    except OSError:
        return None
    drives = parse_mounts(text, roots)
    return drives[0] if drives else None


def usb_fill_pct(mountpoint: Optional[str]) -> int:
    """Drive fill % (0-100), or -1 on error / no drive."""
    if not mountpoint:
        return -1
    try:
        st = os.statvfs(mountpoint)
    except OSError:
        return -1
    if st.f_blocks <= 0:
        return -1
    used = st.f_blocks - st.f_bfree
    pct = int(round(100.0 * used / st.f_blocks))
    return max(0, min(100, pct))


# ---------------- bag discovery ----------------

def latest_bag_dir(bags_root: Path) -> Optional[Path]:
    """Most-recently-modified child directory of `bags_root`, or None.

    We dump the latest dir rather than tracking "the active bag" because the
    bridge has no insight into rosbag2's internals — when the operator hits
    DUMP after a stop, the just-closed bag is the youngest dir. If recording
    is still in progress rsync's --partial keeps the copy resumable.
    """
    if not bags_root.is_dir():
        return None
    children = [p for p in bags_root.iterdir() if p.is_dir()]
    if not children:
        return None
    return max(children, key=lambda p: p.stat().st_mtime)


# ---------------- protocol parser / formatters ----------------

@dataclass
class DeviceEvent:
    kind: str  # booted | btn_start | btn_stop | btn_dump | session | raw
    value: int = 0
    raw: str = ""


_SESS_RE = re.compile(r"^SESS\s+(\d+)$")


def parse_device_line(line: str) -> Optional[DeviceEvent]:
    """Parse one device-side protocol line. None for empty input."""
    s = line.strip()
    if not s:
        return None
    if s == "booted":
        return DeviceEvent(kind="booted", raw=s)
    if s == "BTN START":
        return DeviceEvent(kind="btn_start", raw=s)
    if s == "BTN STOP":
        return DeviceEvent(kind="btn_stop", raw=s)
    if s == "BTN DUMP":
        return DeviceEvent(kind="btn_dump", raw=s)
    m = _SESS_RE.match(s)
    if m:
        return DeviceEvent(kind="session", value=int(m.group(1)), raw=s)
    return DeviceEvent(kind="raw", raw=s)


def fmt_pose(x: float, y: float, z: float) -> str:
    return f"P {x:.3f} {y:.3f} {z:.3f}\n"


def fmt_stats(path_m: float, map_pts: int) -> str:
    return f"S {path_m:.3f} {int(map_pts)}\n"


def fmt_recording(on: bool) -> str:
    return f"R {1 if on else 0}\n"


def fmt_link_online() -> str:
    return "L online\n"


def fmt_dump(state: str) -> str:
    if state not in ("idle", "busy", "ok", "fail"):
        raise ValueError(f"invalid dump state: {state!r}")
    return f"D {state}\n"


def fmt_usb(pct: int) -> str:
    if pct < -1:
        pct = -1
    if pct > 100:
        pct = 100
    return f"U {pct}\n"


# ---------------- serial transport ----------------

class SerialLink:
    """Thread-safe wrapper around a pyserial port. Reads run in a daemon
    thread; writes from any thread are mutex-guarded."""

    def __init__(self, port: str, baud: int = 115200, read_timeout: float = 0.2):
        if serial is None:
            raise RuntimeError("pyserial not installed; pip install pyserial")
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baudrate=baud, timeout=read_timeout)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._on_line: Optional[Callable[[str], None]] = None
        self._reader = threading.Thread(
            target=self._read_loop, name="m5-serial-reader", daemon=True
        )

    def start(self, on_line: Callable[[str], None]) -> None:
        self._on_line = on_line
        self._reader.start()

    def close(self) -> None:
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass

    def write(self, data: str) -> None:
        if self._stop.is_set():
            return
        with self._lock:
            try:
                self.ser.write(data.encode("ascii", errors="replace"))
            except Exception as e:
                LOG.warning("serial write failed: %s", e)

    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(64)
            except Exception as e:
                if self._stop.is_set():
                    return
                LOG.warning("serial read error: %s", e)
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                idx = buf.find(b"\n")
                if idx < 0:
                    break
                line = buf[:idx].decode("ascii", errors="replace").rstrip("\r")
                del buf[:idx + 1]
                cb = self._on_line
                if cb is not None:
                    try:
                        cb(line)
                    except Exception:
                        LOG.exception("on_line callback failed")


# ---------------- dump worker ----------------

@dataclass
class DumpRequest:
    src: Path
    dest: Path


class DumpWorker:
    """Single-flight rsync runner. Reports state via the supplied callback —
    'busy' on start, 'ok' or 'fail' on completion."""

    def __init__(self, send_state: Callable[[str], None]):
        self._send = send_state
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    def busy(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def submit(self, req: DumpRequest) -> bool:
        """Start a dump if none is running. Returns False if already busy."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            t = threading.Thread(
                target=self._run, args=(req,),
                name="m5-dump-worker", daemon=True,
            )
            self._thread = t
        self._send("busy")
        t.start()
        return True

    def _run(self, req: DumpRequest) -> None:
        try:
            req.dest.mkdir(parents=True, exist_ok=True)
            cmd = [
                "rsync", "-a", "--partial",
                str(req.src) + "/", str(req.dest) + "/",
            ]
            LOG.info("dump: %s", " ".join(cmd))
            res = subprocess.run(cmd, capture_output=True, text=True)
            if res.returncode == 0:
                LOG.info("dump ok: %s -> %s", req.src, req.dest)
                self._send("ok")
            else:
                LOG.error("rsync failed (%d): %s",
                          res.returncode, res.stderr.strip())
                self._send("fail")
        except FileNotFoundError as e:
            LOG.error("rsync binary missing: %s", e)
            self._send("fail")
        except Exception:
            LOG.exception("dump failed")
            self._send("fail")


# ---------------- bridge runtime ----------------

class M5Bridge:
    """Glue between device events and the rest of the system. Owns no ROS
    state directly — callbacks hooked in by M5BridgeNode (or left None for
    --no-ros mode)."""

    def __init__(
        self,
        link: "SerialLink | _DryLink",
        bags_root: Path,
        usb_roots: Iterable[str] = USB_MOUNT_ROOTS,
    ):
        self.link = link
        self.bags_root = bags_root
        self.usb_roots = tuple(usb_roots)

        self.session_id: int = 0
        self.dump = DumpWorker(self._send_dump_state)

        self.on_btn_start: Optional[Callable[[], None]] = None
        self.on_btn_stop: Optional[Callable[[], None]] = None
        self.on_session: Optional[Callable[[int], None]] = None

    # ---- outbound ----
    def send_pose(self, x: float, y: float, z: float) -> None:
        self.link.write(fmt_pose(x, y, z))

    def send_stats(self, path_m: float, map_pts: int) -> None:
        self.link.write(fmt_stats(path_m, map_pts))

    def send_recording(self, on: bool) -> None:
        self.link.write(fmt_recording(on))

    def send_link_online(self) -> None:
        self.link.write(fmt_link_online())

    def send_usb(self, pct: int) -> None:
        self.link.write(fmt_usb(pct))

    def _send_dump_state(self, state: str) -> None:
        self.link.write(fmt_dump(state))

    # ---- inbound ----
    def handle_line(self, line: str) -> None:
        ev = parse_device_line(line)
        if ev is None:
            return
        if ev.kind == "raw":
            LOG.debug("device raw: %s", ev.raw)
            return
        LOG.info("device: %s", ev.raw)
        if ev.kind == "booted":
            # Re-announce link freshness so the device shows LNK promptly
            # without waiting for the next heartbeat tick.
            self.send_link_online()
            # And clear any stale dump indicator left over from a prior run.
            self._send_dump_state("idle")
        elif ev.kind == "btn_start":
            cb = self.on_btn_start
            if cb is not None:
                cb()
        elif ev.kind == "btn_stop":
            cb = self.on_btn_stop
            if cb is not None:
                cb()
        elif ev.kind == "btn_dump":
            self._handle_btn_dump()
        elif ev.kind == "session":
            self.session_id = ev.value
            cb = self.on_session
            if cb is not None:
                try:
                    cb(ev.value)
                except Exception:
                    LOG.exception("on_session callback failed")

    # ---- dump request ----
    def _handle_btn_dump(self) -> None:
        if self.dump.busy():
            LOG.info("dump already running, ignoring BTN DUMP")
            return
        usb = find_usb_drive(self.usb_roots)
        if usb is None:
            LOG.warning("BTN DUMP: no USB drive mounted under %s",
                        self.usb_roots)
            self._send_dump_state("fail")
            return
        src = latest_bag_dir(self.bags_root)
        if src is None:
            LOG.warning("BTN DUMP: no bag dir found under %s", self.bags_root)
            self._send_dump_state("fail")
            return
        if self.session_id > 0:
            dest_name = f"tarp_session_{self.session_id:04d}_{src.name}"
        else:
            # No session id yet (host-driven start hasn't fired) — fall back
            # to a timestamp tag so dest still survives multiple dumps in a
            # session.
            dest_name = f"tarp_dump_{int(time.time())}_{src.name}"
        dest = Path(usb) / dest_name
        if not self.dump.submit(DumpRequest(src=src, dest=dest)):
            LOG.warning("BTN DUMP: dump submit lost race; already busy")

    # ---- periodic USB poll ----
    def usb_poll_once(self) -> int:
        usb = find_usb_drive(self.usb_roots)
        pct = usb_fill_pct(usb) if usb else -1
        self.send_usb(pct)
        return pct


# ---------------- ROS2 node ----------------

class M5BridgeNode(Node):
    """rclpy node owning the M5 bridge.

    Subscribes:  /tarp/odom (Odometry), /tarp/points (PointCloud2),
                 /tarp/recording_state (Bool)
    Publishes:   /tarp/recording_request (Bool), /tarp/session_id (UInt32)
    """

    def __init__(self, bridge: M5Bridge):
        super().__init__("m5_bridge")
        self.bridge = bridge

        self.path_m: float = 0.0
        self.map_pts: int = 0
        self._last_pos: Optional[tuple[float, float, float]] = None

        self.create_subscription(Odometry, "/tarp/odom", self._on_odom, 10)
        self.create_subscription(PointCloud2, "/tarp/points", self._on_pc, 10)
        self.create_subscription(
            Bool, "/tarp/recording_state", self._on_rec_state, 10
        )

        self.req_pub = self.create_publisher(Bool, "/tarp/recording_request", 10)
        self.sess_pub = self.create_publisher(UInt32, "/tarp/session_id", 10)

        bridge.on_btn_start = self._fire_request_start
        bridge.on_btn_stop = self._fire_request_stop
        bridge.on_session = self._publish_session

        self.create_timer(1.0, self._tick_stats)
        self.create_timer(0.5, self.bridge.send_link_online)
        self.create_timer(5.0, self._tick_usb)

        self.get_logger().info(
            f"m5_bridge attached; bags_root={bridge.bags_root}"
        )

    def _on_odom(self, msg) -> None:
        p = msg.pose.pose.position
        x, y, z = float(p.x), float(p.y), float(p.z)
        self.bridge.send_pose(x, y, z)
        if self._last_pos is not None:
            dx = x - self._last_pos[0]
            dy = y - self._last_pos[1]
            dz = z - self._last_pos[2]
            self.path_m += math.sqrt(dx * dx + dy * dy + dz * dz)
        self._last_pos = (x, y, z)

    def _on_pc(self, msg) -> None:
        self.map_pts += int(msg.width) * int(msg.height)

    def _on_rec_state(self, msg) -> None:
        self.bridge.send_recording(bool(msg.data))

    def _tick_stats(self) -> None:
        self.bridge.send_stats(self.path_m, self.map_pts)

    def _tick_usb(self) -> None:
        self.bridge.usb_poll_once()

    def _fire_request_start(self) -> None:
        m = Bool()
        m.data = True
        self.req_pub.publish(m)

    def _fire_request_stop(self) -> None:
        m = Bool()
        m.data = False
        self.req_pub.publish(m)

    def _publish_session(self, n: int) -> None:
        m = UInt32()
        m.data = int(n)
        self.sess_pub.publish(m)


# ---------------- dry-run runtime ----------------

class _DryLink:
    """Stand-in for SerialLink that prints to stdout. Lets the bridge be
    smoke-tested without a Core2 attached."""

    def __init__(self) -> None:
        self._lock = threading.Lock()

    def write(self, data: str) -> None:
        with self._lock:
            sys.stdout.write(f"-> {data}")
            sys.stdout.flush()

    def close(self) -> None:
        pass


def run_dry_sim(bridge: M5Bridge, sim_rate_hz: float = 10.0) -> None:
    """Stdout sim runtime that drives sim pose + periodic timers without
    rclpy. Lets the device be exercised on a bench. Ctrl-C to exit."""
    LOG.info("dry-run: driving Lissajous sim at %.1f Hz "
             "(no rclpy or no --port = --no-port mode)", sim_rate_hz)
    t0 = time.time()
    next_link = 0.0
    next_stats = 0.0
    next_usb = 0.0
    path_m = 0.0
    last_xy: Optional[tuple[float, float]] = None
    map_pts = 0
    dt = 1.0 / sim_rate_hz
    while True:
        now = time.time() - t0
        x = 6.0 * math.sin(now * 0.25)
        y = 4.0 * math.sin(now * 0.4 + 0.7)
        z = 1.5
        bridge.send_pose(x, y, z)
        if last_xy is not None:
            path_m += math.hypot(x - last_xy[0], y - last_xy[1])
        last_xy = (x, y)
        map_pts += 200
        if now >= next_stats:
            bridge.send_stats(path_m, map_pts)
            next_stats = now + 1.0
        if now >= next_link:
            bridge.send_link_online()
            next_link = now + 0.5
        if now >= next_usb:
            bridge.usb_poll_once()
            next_usb = now + 5.0
        time.sleep(dt)


# ---------------- entry point ----------------

def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="TARP <-> M5Stack Core2 bridge")
    ap.add_argument(
        "--port", type=str, default=None,
        help="serial device path (default: autodetect by VID:PID)",
    )
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument(
        "--bags-root", type=Path,
        default=Path.home() / ".ros" / "bags",
        help="root dir holding rosbag2 subdirs (most-recent is dumped)",
    )
    ap.add_argument(
        "--usb-roots", type=str, nargs="*", default=list(USB_MOUNT_ROOTS),
        help="mount-point prefixes considered USB drives",
    )
    ap.add_argument(
        "--no-ros", action="store_true",
        help="skip rclpy init; drive sim pose / heartbeat only",
    )
    ap.add_argument(
        "--no-port", action="store_true",
        help="skip serial open; print device traffic to stdout (debugging)",
    )
    ap.add_argument(
        "--sim-rate", type=float, default=10.0,
        help="sim pose rate when running dry (default 10 Hz)",
    )
    ap.add_argument("--log", type=str, default="INFO")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    link: "SerialLink | _DryLink"
    if args.no_port:
        link = _DryLink()
    else:
        port = args.port or find_m5_port()
        if port is None:
            LOG.error(
                "no Core2 found by VID:PID; pass --port or use --no-port "
                "to drive a stdout sim"
            )
            return 2
        LOG.info("opening %s @ %d", port, args.baud)
        try:
            link = SerialLink(port, args.baud)
        except Exception as e:
            LOG.error("failed to open %s: %s", port, e)
            return 2

    bridge = M5Bridge(
        link=link, bags_root=args.bags_root, usb_roots=args.usb_roots,
    )
    if isinstance(link, SerialLink):
        link.start(bridge.handle_line)

    use_ros = HAS_ROS and not args.no_ros
    try:
        if use_ros:
            rclpy.init()
            node = M5BridgeNode(bridge)
            try:
                rclpy.spin(node)
            finally:
                node.destroy_node()
                rclpy.shutdown()
        else:
            if not HAS_ROS:
                LOG.warning("rclpy not importable — running in --no-ros mode")
            run_dry_sim(bridge, sim_rate_hz=args.sim_rate)
    except KeyboardInterrupt:
        LOG.info("interrupted")
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
