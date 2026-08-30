#!/usr/bin/env python3
"""
LD14 serial test reader.

Reads frames from /dev/ttyUSB0 at 230400 baud and decodes the LD14 packet
format described in ld-stl-14.pdf:

    0x54 + 0x2C + speed(2) + start_angle(2) + 12 points * 3
    + end_angle(2) + timestamp(2) + crc8(1)

This script uses pyserial when available. If pyserial is missing, install it
with:

    python3 -m pip install pyserial
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Iterable, List, Optional

try:
    import serial
except ImportError:  # pragma: no cover - runtime environment dependent
    serial = None


HEADER = 0x54
VER_LEN = 0x2C
POINTS_PER_PACK = 12
FRAME_LEN = 47

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


@dataclass
class LidarPoint:
    distance_mm: int
    intensity: int


@dataclass
class LidarFrame:
    speed_dps: int
    start_angle_cdeg: int
    end_angle_cdeg: int
    timestamp_ms: int
    points: List[LidarPoint]
    crc8: int

    def angle_step(self) -> float:
        if len(self.points) <= 1:
            return 0.0
        return (self.end_angle_cdeg - self.start_angle_cdeg) / (len(self.points) - 1)


def crc8(data: Iterable[int]) -> int:
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def parse_frame(frame: bytes) -> LidarFrame:
    if len(frame) != FRAME_LEN:
        raise ValueError(f"unexpected frame length: {len(frame)}")
    if frame[0] != HEADER or frame[1] != VER_LEN:
        raise ValueError("frame header mismatch")
    if crc8(frame[:-1]) != frame[-1]:
        raise ValueError("crc8 mismatch")

    speed = int.from_bytes(frame[2:4], "little")
    start_angle = int.from_bytes(frame[4:6], "little")
    idx = 6
    points: List[LidarPoint] = []
    for _ in range(POINTS_PER_PACK):
        distance = int.from_bytes(frame[idx:idx + 2], "little")
        intensity = frame[idx + 2]
        points.append(LidarPoint(distance, intensity))
        idx += 3
    end_angle = int.from_bytes(frame[idx:idx + 2], "little")
    timestamp = int.from_bytes(frame[idx + 2:idx + 4], "little")
    return LidarFrame(speed, start_angle, end_angle, timestamp, points, frame[-1])


def open_serial(port: str, baudrate: int):
    if serial is None:
        raise RuntimeError(
            "pyserial is not installed. Install it with: python3 -m pip install pyserial"
        )
    return serial.Serial(port=port, baudrate=baudrate, bytesize=8, parity="N", stopbits=1, timeout=1)


def read_frames(port: str, baudrate: int) -> None:
    ser = open_serial(port, baudrate)
    buffer = bytearray()
    print(f"listening on {port} at {baudrate} baud")
    try:
        while True:
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
                try:
                    frame = parse_frame(candidate)
                except ValueError:
                    del buffer[0]
                    continue
                del buffer[:FRAME_LEN]
                print_frame(frame)
    finally:
        ser.close()


def print_frame(frame: LidarFrame) -> None:
    step = frame.angle_step()
    print(
        f"speed={frame.speed_dps} deg/s "
        f"start={frame.start_angle_cdeg / 100.0:.2f} deg "
        f"end={frame.end_angle_cdeg / 100.0:.2f} deg "
        f"timestamp={frame.timestamp_ms} ms "
        f"crc=0x{frame.crc8:02X}"
    )
    for idx, point in enumerate(frame.points):
        angle = (frame.start_angle_cdeg + step * idx) / 100.0
        print(
            f"  pt{idx:02d} angle={angle:7.2f} deg "
            f"dist={point.distance_mm:5d} mm intensity={point.intensity:3d}"
        )


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="LD14 serial test reader")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port path")
    parser.add_argument("--baudrate", type=int, default=230400, help="baud rate")
    args = parser.parse_args(argv)

    try:
        read_frames(args.port, args.baudrate)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
