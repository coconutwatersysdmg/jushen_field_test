"""Convert the supplied Intelly .itl recording format to a standard binary PCD.

The vendor recorder stores a 1441-range CoLa-B scan in each 2980-byte record,
with its last two bytes repurposed as a signed turntable angle (1/100 degree).
They are NOT a CoLa checksum. The vendor compensation tables and integer
rounding order are required to reproduce the accompanying PCD coordinates.
No executable, device connection, or network access is used here.
"""
from __future__ import annotations

import configparser
import contextlib
import hashlib
import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "itl_conversion.ini"
TABLE_PATH = PROJECT_ROOT / "config" / "itl_sensor_compensation.json"
FRAME_BYTES = 2980
RANGES_PER_FRAME = 1441
VENDOR_PI = 3.141592653


@dataclass(frozen=True)
class ItlCalibration:
    tilt_deg: float
    roll_deg: float
    height_mm: int
    angular_step_deg: float
    lateral_mm: tuple[int, ...]
    radial_mm: tuple[int, ...]
    profile_id: str


def load_calibration(config_path: Path = CONFIG_PATH) -> ItlCalibration:
    config = configparser.ConfigParser()
    with Path(config_path).open(encoding="utf-8-sig") as stream:
        config.read_file(stream)
    table_bytes = TABLE_PATH.read_bytes()
    tables = json.loads(table_bytes)
    tilt = float(np.float32(config.getfloat("Para", "PARA_M")))
    roll = float(np.float32(config.getfloat("Para", "PARA_L")))
    height = config.getint("Para", "PARA_G")
    step = float(np.float32(config.getfloat("Para", "ANGLE_STEP_DEG")))
    if not np.isfinite([tilt, roll, step]).all() or step != .125:
        raise ValueError("ITL calibration requires finite angles and ANGLE_STEP_DEG=0.125")
    lateral = tuple(int(x) for x in tables["lateral_mm"])
    radial = tuple(int(x) for x in tables["radial_mm"])
    if len(lateral) != 1440 or len(radial) != 1440:
        raise ValueError("ITL sensor compensation tables must each contain 1440 entries")
    fingerprint = hashlib.sha256(table_bytes + repr((tilt, roll, height, step)).encode()).hexdigest()
    return ItlCalibration(tilt, roll, height, step, lateral, radial, fingerprint)


@contextlib.contextmanager
def _open_finished_source(path: Path):
    """On Windows, refuse sources still open for writing, even if size is stable."""
    if os.name != "nt":
        with path.open("rb") as stream:
            yield stream
        return
    import ctypes
    import msvcrt

    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    create = kernel.CreateFileW
    create.argtypes = [ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32,
                       ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p]
    create.restype = ctypes.c_void_p
    kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    handle = create(str(path.resolve()), 0x80000000, 1, None, 3, 0x80, None)
    if handle == ctypes.c_void_p(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        descriptor = msvcrt.open_osfhandle(handle, os.O_RDONLY | os.O_BINARY)
    except Exception:
        kernel.CloseHandle(handle)
        raise
    with os.fdopen(descriptor, "rb") as stream:
        yield stream


def _convert_frame(frame: bytes, calibration: ItlCalibration, cos_beam, sin_beam) -> np.ndarray:
    import struct

    if (len(frame) != FRAME_BYTES or frame[:8] != b"\x02\x02\x02\x02\x00\x00\x0b\x9b"
            or frame[8:24] != b"sSN LMDscandata " or frame[64:69] != b"DIST1"):
        raise ValueError("Unsupported or damaged ITL scan record; expected Intelly 2980-byte DIST1 recording")
    metadata = struct.unpack_from(">ffiHH", frame, 69)
    if metadata != (1.0, 0.0, 0, 12500, RANGES_PER_FRAME):
        raise ValueError(f"Unsupported ITL DIST1 channel metadata: {metadata}")
    encoder = struct.unpack_from(">h", frame, FRAME_BYTES - 2)[0]
    if not -8000 <= encoder <= 8000:
        raise ValueError(f"ITL turntable angle out of supported range: {encoder / 100} degrees")
    table_index = (90000 - encoder * 10) // 125
    distances = np.frombuffer(frame, dtype=">u2", count=RANGES_PER_FRAME, offset=85).astype(np.float64)
    horizontal = np.trunc(distances * cos_beam)
    radial = np.trunc(distances * sin_beam) - calibration.radial_mm[table_index]
    lateral = -calibration.lateral_mm[table_index]
    tilt = (-encoder / 100 + calibration.tilt_deg) * VENDOR_PI / 180
    roll = calibration.roll_deg * VENDOR_PI / 180
    y = lateral * np.cos(tilt) + radial * np.sin(tilt)
    z = radial * np.cos(tilt) - lateral * np.sin(tilt)
    # Keep the executable's truncation order and zero-distance returns.
    return np.column_stack((
        np.trunc(horizontal * np.cos(roll) - z * np.sin(roll)),
        -np.trunc(y),
        calibration.height_mm - np.trunc(z * np.cos(roll) + horizontal * np.sin(roll)),
    )).astype("<f4")


def convert_itl_to_pcd(source: Path, destination: Path, *, calibration: ItlCalibration | None = None) -> dict:
    source, destination = Path(source), Path(destination)
    if source.suffix.lower() != ".itl" or destination.suffix.lower() != ".pcd":
        raise ValueError("ITL conversion requires an .itl source and a .pcd destination")
    calibration = calibration or load_calibration()
    beam = np.arange(RANGES_PER_FRAME, dtype=np.float64) * calibration.angular_step_deg * VENDOR_PI / 180
    cos_beam, sin_beam = np.cos(beam), np.sin(beam)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with _open_finished_source(source) as incoming:
            before = os.fstat(incoming.fileno())
            if not before.st_size or before.st_size % FRAME_BYTES:
                raise ValueError(f"ITL file is empty or incomplete: size={before.st_size}; record size={FRAME_BYTES}")
            frames = before.st_size // FRAME_BYTES
            count = frames * RANGES_PER_FRAME
            with tempfile.NamedTemporaryFile(dir=destination.parent, prefix=".itl-", suffix=".tmp", delete=False) as outgoing:
                temporary = Path(outgoing.name)
                header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
                          "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                          f"WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n")
                outgoing.write(header.encode("ascii"))
                for index in range(frames):
                    try:
                        xyz = _convert_frame(incoming.read(FRAME_BYTES), calibration, cos_beam, sin_beam)
                    except ValueError as exc:
                        raise ValueError(f"ITL record #{index + 1}: {exc}") from exc
                    outgoing.write(xyz.tobytes())
                after = os.fstat(incoming.fileno())
                if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                    raise ValueError("ITL source changed during conversion; retry after writing finishes")
            os.replace(temporary, destination)
            temporary = None
        return {"frames": frames, "points": count, "source": str(source), "pcd_path": str(destination)}
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
