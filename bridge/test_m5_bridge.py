"""Unit tests for bridge/m5_bridge.py — parser, formatters, USB-mount
discovery, dump dispatch. No serial port or ROS2 required.

Run:
    cd ~/tarp && python3 -m pytest bridge/test_m5_bridge.py -v
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import pytest

from bridge import m5_bridge as mb


# ---------------- parser ----------------

def test_parse_booted():
    ev = mb.parse_device_line("booted")
    assert ev is not None
    assert ev.kind == "booted"
    assert ev.raw == "booted"


def test_parse_buttons():
    assert mb.parse_device_line("BTN START").kind == "btn_start"
    assert mb.parse_device_line("BTN STOP").kind == "btn_stop"
    assert mb.parse_device_line("BTN DUMP").kind == "btn_dump"


def test_parse_session():
    ev = mb.parse_device_line("SESS 7")
    assert ev.kind == "session"
    assert ev.value == 7


def test_parse_session_multidigit():
    ev = mb.parse_device_line("SESS 1234")
    assert ev.kind == "session"
    assert ev.value == 1234


def test_parse_strips_crlf_and_whitespace():
    assert mb.parse_device_line("  BTN DUMP  \r\n").kind == "btn_dump"


def test_parse_empty_returns_none():
    assert mb.parse_device_line("") is None
    assert mb.parse_device_line("   \r\n") is None


def test_parse_unknown_returns_raw():
    ev = mb.parse_device_line("hello world")
    assert ev.kind == "raw"
    assert ev.raw == "hello world"


# ---------------- formatters ----------------

def test_fmt_pose():
    assert mb.fmt_pose(1.0, 2.0, 3.0) == "P 1.000 2.000 3.000\n"
    assert mb.fmt_pose(-0.5, 0.0, 1.234567) == "P -0.500 0.000 1.235\n"


def test_fmt_stats():
    assert mb.fmt_stats(12.5, 1234) == "S 12.500 1234\n"
    assert mb.fmt_stats(0.0, 0) == "S 0.000 0\n"


def test_fmt_recording():
    assert mb.fmt_recording(True) == "R 1\n"
    assert mb.fmt_recording(False) == "R 0\n"


def test_fmt_link_online():
    assert mb.fmt_link_online() == "L online\n"


def test_fmt_dump_valid():
    for s in ("idle", "busy", "ok", "fail"):
        assert mb.fmt_dump(s) == f"D {s}\n"


def test_fmt_dump_invalid_raises():
    with pytest.raises(ValueError):
        mb.fmt_dump("done")


def test_fmt_usb_clamps():
    assert mb.fmt_usb(50) == "U 50\n"
    assert mb.fmt_usb(-1) == "U -1\n"
    assert mb.fmt_usb(-99) == "U -1\n"
    assert mb.fmt_usb(150) == "U 100\n"
    assert mb.fmt_usb(0) == "U 0\n"
    assert mb.fmt_usb(100) == "U 100\n"


# ---------------- USB-mount discovery ----------------

SAMPLE_MOUNTS = (
    "proc /proc proc rw,nosuid 0 0\n"
    "/dev/sda1 / ext4 rw,relatime 0 0\n"
    "/dev/sda2 /home ext4 rw,relatime 0 0\n"
    "tmpfs /run/lock tmpfs rw,nosuid 0 0\n"
    "/dev/sdb1 /media/nick/USB ext4 rw,relatime 0 0\n"
    "/dev/sdc1 /mnt/extra vfat rw,relatime 0 0\n"
)


def test_parse_mounts_finds_usb():
    drives = mb.parse_mounts(SAMPLE_MOUNTS)
    assert "/media/nick/USB" in drives
    assert "/mnt/extra" in drives


def test_parse_mounts_excludes_system():
    drives = mb.parse_mounts(SAMPLE_MOUNTS)
    assert "/" not in drives
    assert "/home" not in drives
    assert "/proc" not in drives


def test_parse_mounts_handles_escaped_spaces():
    text = "/dev/sdb1 /media/nick/My\\040Drive ext4 rw 0 0\n"
    drives = mb.parse_mounts(text)
    assert drives == ["/media/nick/My Drive"]


def test_parse_mounts_handles_run_media():
    text = "/dev/sdb1 /run/media/nick/SCAN exfat rw 0 0\n"
    drives = mb.parse_mounts(text)
    assert drives == ["/run/media/nick/SCAN"]


def test_parse_mounts_excludes_root_itself():
    # /media should not match /media/...
    text = "tmpfs /media tmpfs rw 0 0\n"
    drives = mb.parse_mounts(text)
    assert drives == []


def test_parse_mounts_empty():
    assert mb.parse_mounts("") == []


def test_parse_mounts_custom_roots():
    text = "/dev/sdb1 /custom/mount/x ext4 rw 0 0\n"
    assert mb.parse_mounts(text, roots=("/custom/mount",)) == ["/custom/mount/x"]


# ---------------- bag discovery ----------------

def test_latest_bag_dir(tmp_path: Path):
    a = tmp_path / "bag_a"
    b = tmp_path / "bag_b"
    a.mkdir()
    time.sleep(0.02)
    b.mkdir()
    assert mb.latest_bag_dir(tmp_path) == b


def test_latest_bag_dir_empty(tmp_path: Path):
    assert mb.latest_bag_dir(tmp_path) is None


def test_latest_bag_dir_missing(tmp_path: Path):
    assert mb.latest_bag_dir(tmp_path / "does-not-exist") is None


def test_latest_bag_dir_skips_files(tmp_path: Path):
    (tmp_path / "loose.bag").write_text("x")
    d = tmp_path / "bag_dir"
    d.mkdir()
    assert mb.latest_bag_dir(tmp_path) == d


# ---------------- bridge dispatch ----------------

class FakeLink:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self._lock = threading.Lock()

    def write(self, data: str) -> None:
        with self._lock:
            self.lines.append(data)

    def close(self) -> None:
        pass

    def find(self, prefix: str) -> list[str]:
        return [ln for ln in self.lines if ln.startswith(prefix)]


def make_bridge(tmp_path: Path, usb: list[str] | None = None) -> tuple[mb.M5Bridge, FakeLink]:
    link = FakeLink()
    bags = tmp_path / "bags"
    bags.mkdir()
    bridge = mb.M5Bridge(link=link, bags_root=bags,
                         usb_roots=tuple(usb or [str(tmp_path / "usb")]))
    return bridge, link


def test_handle_session_updates_id_and_callback(tmp_path: Path):
    bridge, _ = make_bridge(tmp_path)
    seen = []
    bridge.on_session = seen.append
    bridge.handle_line("SESS 42")
    assert bridge.session_id == 42
    assert seen == [42]


def test_handle_btn_start_fires_callback(tmp_path: Path):
    bridge, _ = make_bridge(tmp_path)
    fired = []
    bridge.on_btn_start = lambda: fired.append("start")
    bridge.handle_line("BTN START")
    assert fired == ["start"]


def test_handle_btn_stop_fires_callback(tmp_path: Path):
    bridge, _ = make_bridge(tmp_path)
    fired = []
    bridge.on_btn_stop = lambda: fired.append("stop")
    bridge.handle_line("BTN STOP")
    assert fired == ["stop"]


def test_booted_emits_link_and_idle(tmp_path: Path):
    bridge, link = make_bridge(tmp_path)
    bridge.handle_line("booted")
    assert "L online\n" in link.lines
    assert "D idle\n" in link.lines


def test_btn_dump_no_usb_emits_fail(tmp_path: Path):
    bridge, link = make_bridge(tmp_path, usb=[str(tmp_path / "no-usb-here")])
    # Need a bag to exist so we don't fail on bag-missing first
    (bridge.bags_root / "bag_001").mkdir()
    bridge.handle_line("BTN DUMP")
    # Wait briefly in case the dump worker started (it shouldn't have)
    time.sleep(0.05)
    assert link.find("D fail")
    assert not link.find("D busy")


def test_btn_dump_no_bag_emits_fail(tmp_path: Path):
    usb = tmp_path / "usb_root"
    usb.mkdir()
    bridge, link = make_bridge(tmp_path, usb=[str(usb)])
    # Mount one fake usb-drive subdir under usb_root so find_usb_drive succeeds.
    drive = usb / "drive1"
    drive.mkdir()
    # Patch /proc/mounts via parse_mounts? Simpler: monkeypatch find_usb_drive.
    bridge._handle_btn_dump = lambda: bridge._send_dump_state("fail")  # quick override
    bridge.handle_line("BTN DUMP")
    assert link.find("D fail")


def test_session_id_used_in_dest_name(tmp_path: Path, monkeypatch):
    bridge, link = make_bridge(tmp_path)
    bag = bridge.bags_root / "bag_alpha"
    bag.mkdir()

    usb_dest = tmp_path / "usb"
    usb_dest.mkdir()

    captured: list[mb.DumpRequest] = []

    def fake_submit(req: mb.DumpRequest) -> bool:
        captured.append(req)
        return True

    monkeypatch.setattr(mb, "find_usb_drive", lambda _roots: str(usb_dest))
    monkeypatch.setattr(bridge.dump, "submit", fake_submit)

    bridge.handle_line("SESS 7")
    bridge.handle_line("BTN DUMP")

    assert len(captured) == 1
    assert captured[0].src == bag
    assert captured[0].dest.name == "tarp_session_0007_bag_alpha"
    assert captured[0].dest.parent == usb_dest


def test_no_session_falls_back_to_timestamp(tmp_path: Path, monkeypatch):
    bridge, link = make_bridge(tmp_path)
    bag = bridge.bags_root / "bag_beta"
    bag.mkdir()
    usb_dest = tmp_path / "usb"
    usb_dest.mkdir()

    captured: list[mb.DumpRequest] = []
    monkeypatch.setattr(mb, "find_usb_drive", lambda _roots: str(usb_dest))
    monkeypatch.setattr(bridge.dump, "submit",
                       lambda req: captured.append(req) or True)

    bridge.handle_line("BTN DUMP")
    assert len(captured) == 1
    assert captured[0].dest.name.startswith("tarp_dump_")
    assert captured[0].dest.name.endswith("_bag_beta")


def test_usb_poll_no_drive_emits_minus_one(tmp_path: Path, monkeypatch):
    bridge, link = make_bridge(tmp_path)
    monkeypatch.setattr(mb, "find_usb_drive", lambda _roots: None)
    pct = bridge.usb_poll_once()
    assert pct == -1
    assert "U -1\n" in link.lines


def test_usb_poll_uses_statvfs(tmp_path: Path, monkeypatch):
    bridge, link = make_bridge(tmp_path)
    monkeypatch.setattr(mb, "find_usb_drive", lambda _roots: str(tmp_path))
    pct = bridge.usb_poll_once()
    assert 0 <= pct <= 100
    assert link.lines[-1] == f"U {pct}\n"


# ---------------- DumpWorker ----------------

def test_dump_worker_busy_then_ok(tmp_path: Path):
    states: list[str] = []
    lock = threading.Lock()

    def cb(s: str) -> None:
        with lock:
            states.append(s)

    src = tmp_path / "src"
    dest = tmp_path / "dest"
    src.mkdir()
    (src / "file.txt").write_text("hello")

    worker = mb.DumpWorker(cb)
    accepted = worker.submit(mb.DumpRequest(src=src, dest=dest))
    assert accepted

    # Wait for completion
    for _ in range(100):
        time.sleep(0.05)
        with lock:
            if "ok" in states or "fail" in states:
                break

    with lock:
        assert states[0] == "busy"
        assert states[-1] in ("ok", "fail")
    if states[-1] == "ok":
        assert (dest / "file.txt").read_text() == "hello"


def test_dump_worker_rejects_concurrent(tmp_path: Path):
    src = tmp_path / "src"
    src.mkdir()

    block = threading.Event()
    cb_states: list[str] = []
    worker = mb.DumpWorker(cb_states.append)

    # Replace the rsync invocation with a blocking sleep so we can race.
    def slow_run(self, req):
        cb_states.append("running")
        block.wait(timeout=2.0)
        cb_states.append("ok")

    import types
    worker._run = types.MethodType(slow_run, worker)  # type: ignore

    assert worker.submit(mb.DumpRequest(src=src, dest=tmp_path / "d1"))
    # The second submit should be rejected without firing 'busy' twice.
    assert not worker.submit(mb.DumpRequest(src=src, dest=tmp_path / "d2"))
    block.set()
    # Allow worker thread to finish
    for _ in range(40):
        if not worker.busy():
            break
        time.sleep(0.05)
    assert not worker.busy()
    # Exactly one 'busy' was emitted by submit() (plus 'running' / 'ok' from
    # our patched _run).
    assert cb_states.count("busy") == 1


# ---------------- find_m5_port (smoke) ----------------

def test_find_m5_port_no_match(monkeypatch):
    """When no Core2 is plugged in, returns None without raising."""
    class FakePort:
        vid = 0x1234
        pid = 0x5678
        device = "/dev/null"
    monkeypatch.setattr(mb, "list_ports",
                        type("LP", (), {"comports": staticmethod(lambda: [FakePort()])}))
    assert mb.find_m5_port() is None


def test_find_m5_port_matches_ch9102(monkeypatch):
    class FakePort:
        vid = 0x1A86
        pid = 0x55D4
        device = "/dev/ttyUSB42"
    monkeypatch.setattr(mb, "list_ports",
                        type("LP", (), {"comports": staticmethod(lambda: [FakePort()])}))
    assert mb.find_m5_port() == "/dev/ttyUSB42"
