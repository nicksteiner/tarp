"""Minimal static server for the web/ SPA during dev. Binds to all interfaces
so the iPad on the same hotspot can reach it.

    python3 scripts/serve.py [--port 8000]
"""

import argparse
import http.server
import socketserver
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent / "web"


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()
    with socketserver.TCPServer(("0.0.0.0", args.port), Handler) as httpd:
        print(f"serving {ROOT} on 0.0.0.0:{args.port}")
        httpd.serve_forever()


if __name__ == "__main__":
    main()
