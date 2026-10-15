"""Run an explicitly configured vendor converter and publish validated PCD files.

The bundled BZKJDEMO GUI needs separately supplied 32-bit PCL runtime DLLs.
No undocumented ITL coordinates or command-line arguments are inferred here.
"""
from __future__ import annotations

import math
import os
import shutil
import signal
import struct
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import Sequence


VENDOR_CONVERTER_STATUS = (
    "ITL 转换未配置：厂商 EXE 原包缺少 32 位 PCL 运行库；"
    "补齐官方依赖可启动 GUI，但载入配对 ITL 样例退出 0xC0000409，未生成 PCD。"
    "需配置带 {input}/{output} 的已验证转换命令，详见 docs/ITL转换器部署说明.md。"
)


class ConversionError(RuntimeError):
    """The converter failed or did not produce a valid point cloud."""


class ConverterUnavailable(ConversionError):
    """No usable converter command has been configured."""


class ConversionCancelled(ConversionError):
    """The owner requested that conversion stop."""


def _check_cancel(cancel_event: threading.Event | None) -> None:
    if cancel_event is not None and cancel_event.is_set():
        raise ConversionCancelled("ITL 转换已停止")


def _validate_lzf(stream, expected_bytes: int, cancel_event: threading.Event | None) -> None:
    # Check LZF runs/references without allocating the expanded point cloud.
    expanded = 0
    blocks = 0
    while True:
        control = stream.read(1)
        if not control:
            break
        code = control[0]
        if code < 32:
            run = code + 1
            if len(stream.read(run)) != run:
                raise ValueError("PCD LZF 字面数据截断")
        else:
            run = code >> 5
            if run == 7:
                extra = stream.read(1)
                if not extra:
                    raise ValueError("PCD LZF 长度截断")
                run += extra[0]
            offset = stream.read(1)
            if not offset or ((code & 31) << 8) + offset[0] + 1 > expanded:
                raise ValueError("PCD LZF 回溯引用无效")
            run += 2
        expanded += run
        if expanded > expected_bytes:
            raise ValueError("PCD LZF 展开数据超出声明")
        blocks += 1
        if blocks % 1024 == 0:
            _check_cancel(cancel_event)
    if expanded != expected_bytes:
        raise ValueError("PCD LZF 展开数据长度不匹配")


def validate_pcd(path: Path, cancel_event: threading.Event | None = None) -> None:
    """Check PCD structure, declared point counts, and ASCII numeric payload.

    Binary payload lengths and LZF compressed runs are checked. This is
file integrity validation; it does not certify the vendor's coordinate frame.
"""
    try:
        with Path(path).open("rb") as stream:
            header: dict[str, list[str]] = {}
            header_bytes = 0
            while True:
                line = stream.readline(65537)
                header_bytes += len(line)
                if not line or header_bytes > 65536:
                    raise ValueError("PCD 文件头缺失或过长")
                parts = line.decode("ascii").strip().split()
                if not parts or parts[0].startswith("#"):
                    continue
                key = parts[0].upper()
                if key in header:
                    raise ValueError(f"PCD 重复字段 {key}")
                header[key] = parts[1:]
                if key == "DATA":
                    break
            fields = header["FIELDS"]
            sizes = [int(value) for value in header["SIZE"]]
            types = header["TYPE"]
            counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
            width, height, points = (int(header[key][0]) for key in ("WIDTH", "HEIGHT", "POINTS"))
            if not fields or len(set(fields)) != len(fields) or not {"x", "y", "z"}.issubset(fields):
                raise ValueError("PCD 必须包含 x/y/z 字段")
            if not (len(fields) == len(sizes) == len(types) == len(counts)):
                raise ValueError("PCD 字段长度不一致")
            if any(size not in (1, 2, 4, 8) for size in sizes) or any(kind not in ("F", "I", "U") for kind in types):
                raise ValueError("PCD 字段类型无效")
            if any(count <= 0 for count in counts) or width <= 0 or height <= 0 or points != width * height:
                raise ValueError("PCD 点数/尺寸无效")
            if any(counts[fields.index(axis)] != 1 for axis in ("x", "y", "z")):
                raise ValueError("PCD 坐标字段必须为标量")
            encoding = header["DATA"]
            expected_bytes = points * sum(size * count for size, count in zip(sizes, counts))
            remaining = os.fstat(stream.fileno()).st_size - stream.tell()
            _check_cancel(cancel_event)
            if encoding == ["ascii"]:
                row_count = 0
                columns = sum(counts)
                coordinate_columns = [sum(counts[:fields.index(axis)]) for axis in ("x", "y", "z")]
                for line in stream:
                    parts = line.split()
                    if not parts:
                        continue
                    if len(parts) != columns:
                        raise ValueError("PCD 点数据列数不匹配")
                    values = [float(value) for value in parts]
                    if not all(math.isfinite(values[index]) for index in coordinate_columns):
                        raise ValueError("PCD 包含无效坐标")
                    row_count += 1
                    if row_count > points:
                        raise ValueError("PCD 点数超出声明")
                    if row_count % 1024 == 0:
                        _check_cancel(cancel_event)
                if row_count != points:
                    raise ValueError("PCD 点数据不完整")
            elif encoding == ["binary"]:
                if remaining != expected_bytes:
                    raise ValueError("PCD 二进制点数据长度不匹配")
            elif encoding == ["binary_compressed"]:
                prefix = stream.read(8)
                if len(prefix) != 8:
                    raise ValueError("PCD 压缩数据头不完整")
                compressed, uncompressed = struct.unpack("<II", prefix)
                if compressed <= 0 or remaining != compressed + 8 or uncompressed != expected_bytes:
                    raise ValueError("PCD 压缩数据长度不匹配")
                _validate_lzf(stream, expected_bytes, cancel_event)
            else:
                raise ValueError("PCD DATA 编码不支持")
            _check_cancel(cancel_event)
    except (OSError, ValueError, KeyError, IndexError, UnicodeError) as exc:
        raise ConversionError(f"PCD 校验失败：{path}：{exc}") from exc


def _stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    if os.name == "nt":
        try:
            subprocess.run(
                ["taskkill.exe", "/PID", str(process.pid), "/T", "/F"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                creationflags=subprocess.CREATE_NO_WINDOW, timeout=10, check=False,
            )
        except (OSError, subprocess.SubprocessError):
            # Still terminate the direct child if the OS tree helper fails.
            pass
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if process.poll() is None:
        process.kill()
    process.wait(timeout=10)


def convert_itl(
    source: Path,
    destination: Path,
    command_template: Sequence[str] | None,
    timeout_s: float = 120,
    cancel_event: threading.Event | None = None,
) -> Path:
    """Run an argv template containing {input}/{output}, without a shell.

    The command must wait for completion and write the supplied output path.
    Its working directory is the executable directory (for vendor INI/DLLs).
    Output is staged beside the destination and published without overwriting.
    """
    source = Path(source).expanduser().resolve()
    destination = Path(destination).expanduser().resolve()
    if not command_template:
        raise ConverterUnavailable(VENDOR_CONVERTER_STATUS)
    if isinstance(command_template, (str, bytes)) or not all(isinstance(arg, str) for arg in command_template):
        raise ConversionError("转换命令必须是字符串参数列表")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ConversionError("转换超时必须大于零")
    if not source.is_file() or source.suffix.lower() != ".itl" or source.stat().st_size == 0:
        raise ConversionError(f"ITL 输入不存在、为空或扩展名错误：{source}")
    if destination.suffix.lower() != ".pcd":
        raise ConversionError("转换输出必须为 .pcd 文件")
    if destination.exists():
        raise ConversionError(f"PCD 输出已存在，拒绝覆盖：{destination}")
    if not all(any(token in arg for arg in command_template) for token in ("{input}", "{output}")):
        raise ConversionError("转换命令必须包含 {input} 和 {output}")
    _check_cancel(cancel_event)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".itl-convert-", dir=destination.parent) as folder:
        staged = Path(folder) / destination.name
        try:
            command = [arg.format(input=str(source), output=str(staged)) for arg in command_template]
        except (KeyError, ValueError, IndexError) as exc:
            raise ConversionError(f"转换命令占位符无效：{exc}") from exc
        executable = shutil.which(command[0])
        if executable is None:
            raise ConverterUnavailable(f"找不到转换程序：{command[0]}")
        command[0] = str(Path(executable).resolve())
        options = {"creationflags": subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {"start_new_session": True}
        with tempfile.TemporaryFile() as diagnostic:
            try:
                process = subprocess.Popen(
                    command, cwd=str(Path(command[0]).parent), stdin=subprocess.DEVNULL,
                    stdout=diagnostic, stderr=subprocess.STDOUT, shell=False, **options,
                )
            except OSError as exc:
                raise ConverterUnavailable(f"无法启动转换程序：{exc}") from exc
            deadline = time.monotonic() + timeout_s
            try:
                while process.poll() is None:
                    _check_cancel(cancel_event)
                    if time.monotonic() >= deadline:
                        raise ConversionError(f"ITL 转换超时（{timeout_s} 秒）：{source.name}")
                    if cancel_event is None:
                        time.sleep(0.05)
                    else:
                        cancel_event.wait(0.05)
            finally:
                _stop_process(process)
            _check_cancel(cancel_event)
            if process.returncode != 0:
                diagnostic.seek(0, os.SEEK_END)
                diagnostic.seek(max(0, diagnostic.tell() - 4000))
                detail = diagnostic.read().decode("utf-8", errors="replace").strip()
                code = process.returncode & 0xFFFFFFFF
                if code == 0xC0000135:
                    detail = "缺少运行时 DLL；请提供完整、位数匹配的厂商运行包。" + detail
                raise ConversionError(f"ITL 转换失败，退出码 {process.returncode} (0x{code:08X})：{detail}")
        if not staged.is_file() or staged.is_symlink():
            raise ConversionError(f"转换程序未生成指定 PCD 输出：{staged}")
        validate_pcd(staged, cancel_event=cancel_event)
        _check_cancel(cancel_event)
        try:
            # A same-filesystem hard link publishes atomically and fails if the
            # final pathname appeared while conversion was running.
            os.link(staged, destination)
        except OSError as exc:
            raise ConversionError(f"无法发布 PCD（拒绝覆盖已有文件）：{destination}：{exc}") from exc
    return destination
