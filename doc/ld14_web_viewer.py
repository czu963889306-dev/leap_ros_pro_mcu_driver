#!/usr/bin/env python3
"""
LD14 web point-cloud viewer.

Reads LD14 frames from /dev/ttyUSB0 at 230400 baud and serves a local web page
that renders the latest point cloud on a canvas.

Dependencies:
    python3 -m pip install pyserial

Run:
    python3 doc/ld14_web_viewer.py
    open http://127.0.0.1:8000
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Deque, Dict, Iterable, List, Optional

try:
    import serial
except ImportError:  # pragma: no cover - runtime environment dependent
    serial = None


HEADER = 0x54
VER_LEN = 0x2C
POINTS_PER_PACK = 12
FRAME_LEN = 47
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 230400
DEFAULT_HTTP_HOST = "0.0.0.0"
DEFAULT_HTTP_PORT = 8000
MAX_POINTS = 4000

CRC_TABLE = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25,
    0x8B, 0xC6, 0x11, 0x5C, 0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07,
    0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5, 0x1F, 0x52, 0x85, 0xC8,
    0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93,
    0x3D, 0x70, 0xA7, 0xEA, 0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90,
    0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62, 0x97, 0xDA, 0x0D, 0x40,
    0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04,
    0xAA, 0xE7, 0x30, 0x7D, 0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26,
    0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4, 0x7C, 0x31, 0xE6, 0xAB,
    0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0,
    0x5E, 0x13, 0xC4, 0x89, 0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD,
    0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F, 0xCA, 0x87, 0x50, 0x1D,
    0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67,
    0xC9, 0x84, 0x53, 0x1E, 0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7, 0x5D, 0x10, 0xC7, 0x8A,
    0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1,
    0x7F, 0x32, 0xE5, 0xA8,
]

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>LD14 Point Cloud</title>
  <style>
    html, body {
      width: 100%;
      height: 100%;
      margin: 0;
      background: #0b0f14;
      color: #d8e0ea;
      font: 13px/1.4 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      overflow: hidden;
    }
    #wrap {
      position: relative;
      width: 100vw;
      height: 100vh;
    }
    canvas {
      display: block;
      width: 100%;
      height: 100%;
      background: #0b0f14;
    }
    #hud {
      position: absolute;
      left: 12px;
      top: 12px;
      z-index: 2;
      padding: 10px 12px;
      background: rgba(8, 12, 18, 0.75);
      border: 1px solid rgba(160, 180, 200, 0.18);
      border-radius: 6px;
      max-width: min(520px, calc(100vw - 24px));
      white-space: pre-wrap;
    }
    #hint {
      position: absolute;
      right: 12px;
      bottom: 12px;
      z-index: 2;
      padding: 8px 10px;
      background: rgba(8, 12, 18, 0.65);
      border: 1px solid rgba(160, 180, 200, 0.18);
      border-radius: 6px;
    }
    .bad { color: #ff9b9b; }
    .good { color: #8fe6b2; }
  </style>
</head>
<body>
  <div id="wrap">
    <canvas id="view"></canvas>
    <div id="hud">loading...</div>
    <div id="hint">LD14 live point cloud</div>
  </div>
  <script>
    const canvas = document.getElementById('view');
    const ctx = canvas.getContext('2d');
    const hud = document.getElementById('hud');
    const maxRangeMm = 12000;
    let latest = null;

    function resize() {
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.floor(window.innerWidth * dpr);
      canvas.height = Math.floor(window.innerHeight * dpr);
      canvas.style.width = window.innerWidth + 'px';
      canvas.style.height = window.innerHeight + 'px';
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    function clear() {
      ctx.fillStyle = '#0b0f14';
      ctx.fillRect(0, 0, window.innerWidth, window.innerHeight);
    }

    function drawGrid(cx, cy, radius) {
      ctx.strokeStyle = 'rgba(160,180,200,0.12)';
      ctx.lineWidth = 1;
      for (let i = 1; i <= 4; i++) {
        ctx.beginPath();
        ctx.arc(cx, cy, radius * i / 4, 0, Math.PI * 2);
        ctx.stroke();
      }
      ctx.beginPath();
      ctx.moveTo(cx - radius, cy);
      ctx.lineTo(cx + radius, cy);
      ctx.moveTo(cx, cy - radius);
      ctx.lineTo(cx, cy + radius);
      ctx.stroke();
    }

    function drawPoints(points, cx, cy, radius) {
      for (const p of points) {
        const x = cx + p[0] * radius;
        const y = cy - p[1] * radius;
        const intensity = Math.max(0, Math.min(255, p[3] || 0));
        const alpha = 0.20 + intensity / 400;
        ctx.fillStyle = `rgba(56, 212, 255, ${Math.min(alpha, 0.95)})`;
        ctx.beginPath();
        ctx.arc(x, y, 1.8, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    function render(data) {
      clear();
      const cx = window.innerWidth / 2;
      const cy = window.innerHeight / 2;
      const radius = Math.min(window.innerWidth, window.innerHeight) * 0.45;
      drawGrid(cx, cy, radius);
      if (data && data.points && data.points.length) {
        drawPoints(data.points, cx, cy, radius);
      }

      const status = data && data.connected ? '<span class="good">connected</span>' : '<span class="bad">disconnected</span>';
      const error = data && data.error ? `\nerror: ${data.error}` : '';
      const stats = data ? (
        `port: ${data.port}  baud: ${data.baudrate}\n` +
        `state: ${status}\n` +
        `frames: ${data.frame_count}  points: ${data.points ? data.points.length : 0}\n` +
        `last frame: ${data.last_frame ? data.last_frame.speed_dps + ' deg/s, ' + data.last_frame.timestamp_ms + ' ms' : '-'}${error}`
      ) : 'waiting...';
      hud.innerHTML = stats;
    }

    async function poll() {
      try {
        const resp = await fetch('/api/state', { cache: 'no-store' });
        latest = await resp.json();
        render(latest);
      } catch (err) {
        hud.innerHTML = '<span class="bad">web request failed</span>';
      }
    }

    window.addEventListener('resize', () => {
      resize();
      render(latest);
    });

    resize();
    poll();
    setInterval(poll, 100);
  </script>
</body>
</html>
"""


@dataclass
class LidarPoint:
    x: float
    y: float
    distance_mm: int
    intensity: int
    angle_deg: float


@dataclass
class FrameSummary:
    speed_dps: int
    start_angle_cdeg: int
    end_angle_cdeg: int
    timestamp_ms: int
    crc8: int


class SharedState:
    def __init__(self, port: str, baudrate: int) -> None:
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.error = ""
        self.frame_count = 0
        self.last_frame: Optional[FrameSummary] = None
        self.points: Deque[LidarPoint] = deque(maxlen=MAX_POINTS)
        self.lock = threading.Lock()

    def set_error(self, message: str) -> None:
        with self.lock:
            self.connected = False
            self.error = message

    def set_connected(self, value: bool) -> None:
        with self.lock:
            self.connected = value
            if value:
                self.error = ""

    def add_frame(self, summary: FrameSummary, points: List[LidarPoint]) -> None:
        with self.lock:
            self.connected = True
            self.error = ""
            self.frame_count += 1
            self.last_frame = summary
            self.points.extend(points)

    def snapshot(self) -> Dict[str, object]:
        with self.lock:
            return {
                "port": self.port,
                "baudrate": self.baudrate,
                "connected": self.connected,
                "error": self.error,
                "frame_count": self.frame_count,
                "last_frame": asdict(self.last_frame) if self.last_frame else None,
                "points": [
                    [p.x, p.y, p.distance_mm, p.intensity, p.angle_deg]
                    for p in self.points
                ],
            }


def crc8(data: Iterable[int]) -> int:
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def parse_frame(frame: bytes) -> Optional[Dict[str, object]]:
    if len(frame) != FRAME_LEN:
        return None
    if frame[0] != HEADER or frame[1] != VER_LEN:
        return None
    if crc8(frame[:-1]) != frame[-1]:
        return None

    speed = int.from_bytes(frame[2:4], "little")
    start_angle = int.from_bytes(frame[4:6], "little")
    idx = 6
    raw_points: List[tuple[int, int]] = []
    for _ in range(POINTS_PER_PACK):
        distance = int.from_bytes(frame[idx:idx + 2], "little")
        intensity = frame[idx + 2]
        raw_points.append((distance, intensity))
        idx += 3
    end_angle = int.from_bytes(frame[idx:idx + 2], "little")
    timestamp = int.from_bytes(frame[idx + 2:idx + 4], "little")

    if POINTS_PER_PACK > 1:
        step = (end_angle - start_angle) / (POINTS_PER_PACK - 1)
    else:
        step = 0.0

    points: List[LidarPoint] = []
    for i, (distance_mm, intensity) in enumerate(raw_points):
        angle_deg = (start_angle + step * i) / 100.0
        angle_rad = math.radians(angle_deg)
        radius = distance_mm / 1000.0
        points.append(
            LidarPoint(
                x=radius * math.cos(angle_rad),
                y=radius * math.sin(angle_rad),
                distance_mm=distance_mm,
                intensity=intensity,
                angle_deg=angle_deg,
            )
        )

    summary = FrameSummary(
        speed_dps=speed,
        start_angle_cdeg=start_angle,
        end_angle_cdeg=end_angle,
        timestamp_ms=timestamp,
        crc8=frame[-1],
    )
    return {"summary": summary, "points": points}


def open_serial(port: str, baudrate: int):
    if serial is None:
        raise RuntimeError("pyserial is not installed. Run: python3 -m pip install pyserial")
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=1,
    )


def serial_worker(state: SharedState, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        ser = None
        try:
            ser = open_serial(state.port, state.baudrate)
            state.set_connected(True)
            buffer = bytearray()
            while not stop_event.is_set():
                chunk = ser.read(256)
                if not chunk:
                    continue
                buffer.extend(chunk)
                while True:
                    header_pos = buffer.find(bytes([HEADER]))
                    if header_pos < 0:
                        buffer.clear()
                        break
                    if header_pos > 0:
                        del buffer[:header_pos]
                    if len(buffer) < FRAME_LEN:
                        break
                    candidate = bytes(buffer[:FRAME_LEN])
                    if candidate[1] != VER_LEN:
                        del buffer[0]
                        continue
                    parsed = parse_frame(candidate)
                    if parsed is None:
                        del buffer[0]
                        continue
                    del buffer[:FRAME_LEN]
                    state.add_frame(parsed["summary"], parsed["points"])
        except Exception as exc:
            state.set_error(str(exc))
            time.sleep(1.0)
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass


class ViewerHandler(BaseHTTPRequestHandler):
    state: SharedState = None  # type: ignore[assignment]

    def do_GET(self) -> None:
        if self.path == "/" or self.path.startswith("/index.html"):
            body = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/api/state"):
            payload = json.dumps(self.state.snapshot(), ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format: str, *args) -> None:  # pragma: no cover - noisy
        return


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="LD14 web point-cloud viewer")
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial port path")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="serial baud rate")
    parser.add_argument("--host", default=DEFAULT_HTTP_HOST, help="HTTP listen host")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT, help="HTTP listen port")
    args = parser.parse_args(argv)

    state = SharedState(args.port, args.baudrate)
    stop_event = threading.Event()
    worker = threading.Thread(target=serial_worker, args=(state, stop_event), daemon=True)
    worker.start()

    ViewerHandler.state = state
    server = ThreadingHTTPServer((args.host, args.http_port), ViewerHandler)
    print(f"web viewer: http://127.0.0.1:{args.http_port}")
    print(f"serial: {args.port} @ {args.baudrate}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.shutdown()
        server.server_close()
        worker.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
