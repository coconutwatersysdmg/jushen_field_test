# -*- coding: utf-8 -*-
"""Windows Wi-Fi/IP switching. Importing this module never changes the network."""
from __future__ import annotations

import base64
from contextlib import contextmanager
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import traceback


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "network_switch.json"
SCRIPT_PATH = PROJECT_ROOT / "scripts" / "network_switch.ps1"
LOG_PATH = PROJECT_ROOT / "logs" / "network_switch.log"


def _log(message: str) -> None:
    try:
        LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with LOG_PATH.open("a", encoding="utf-8") as stream:
            stream.write(f"[{datetime.now().isoformat(timespec='seconds')}] {message}\n")
    except OSError:
        pass


@contextmanager
def _result_file():
    """Keep result ownership with the GUI and never mask results with cleanup errors."""
    directory = Path(tempfile.mkdtemp(prefix="jushen-network-"))
    try:
        result_path = directory / "result.json"
        # The elevated helper opens this existing file, retaining its original
        # ownership/access rules instead of creating a new elevated-owned file.
        result_path.write_text("", encoding="utf-8")
        yield result_path
    finally:
        # Only clean the exact directory generated for this call inside Temp.
        try:
            folder = directory.resolve()
            temp_root = Path(tempfile.gettempdir()).resolve()
            if folder.parent == temp_root and folder.name.startswith("jushen-network-"):
                shutil.rmtree(directory)
            else:
                _log(f"RESULT_CLEANUP_SKIPPED unexpected_path={folder}")
        except OSError:
            _log("RESULT_CLEANUP_FAILED (network result preserved)\n" + traceback.format_exc())


def _run_powershell(arguments: list[str], *, timeout: float | None) -> subprocess.CompletedProcess:
    if os.name != "nt":
        raise RuntimeError("网络切换仅支持 Windows。")
    return subprocess.run(
        ["powershell.exe", "-NoProfile", "-NonInteractive", *arguments],
        stdin=subprocess.DEVNULL, capture_output=True,
        encoding="utf-8", errors="replace", timeout=timeout,
        creationflags=subprocess.CREATE_NO_WINDOW,
    )


def _encoded_command(command: str) -> list[str]:
    encoded = base64.b64encode(command.encode("utf-16-le")).decode("ascii")
    return ["-EncodedCommand", encoded]


def read_network_state() -> dict:
    result = _run_powershell(
        ["-ExecutionPolicy", "Bypass", "-File", str(SCRIPT_PATH),
         "-Mode", "inspect", "-ConfigPath", str(CONFIG_PATH)],
        timeout=25,
    )
    if result.returncode:
        try:
            detail = json.loads(result.stdout.lstrip("\ufeff").strip()).get("message")
        except ValueError:
            detail = None
        raise RuntimeError(detail or result.stderr.strip() or result.stdout.strip() or "无法读取网络状态。")
    try:
        return json.loads(result.stdout.lstrip("\ufeff").strip())
    except ValueError as exc:
        raise RuntimeError("无法解析 Windows 返回的网络状态。") from exc


def switch_network(target: str) -> dict:
    """Request UAC before any Wi-Fi/IP mutation, then wait for verified settings."""
    if target not in ("radar", "hotspot"):
        raise ValueError("未知的目标网络。")
    # Result files live in a private per-invocation folder; nothing contains passwords.
    with _result_file() as result_path:
        args = ["-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass",
                "-File", str(SCRIPT_PATH), "-Mode", target,
                "-ConfigPath", str(CONFIG_PATH), "-ResultPath", str(result_path)]
        # Start-Process joins ArgumentList with spaces: supply a fully quoted command line.
        command_line = subprocess.list2cmdline(args)
        quoted_args = "'" + command_line.replace("'", "''") + "'"
        command = (
            "$ErrorActionPreference='Stop'; "
            "[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new($false); "
            "try { $p=Start-Process -FilePath "
            "(Join-Path $PSHOME 'powershell.exe') -Verb RunAs -WindowStyle Hidden "
            f"-ArgumentList {quoted_args} -Wait -PassThru; exit $p.ExitCode "
            "} catch { [Console]::Error.WriteLine($_.Exception.Message); exit 1 }"
        )
        # Do not time out the UAC dialog: killing the launcher could leave the elevated
        # helper running and allow a second switch to overlap with the first.
        _log(f"SWITCH_BEGIN target={target}")
        process = _run_powershell(_encoded_command(command), timeout=None)
        _log(f"HELPER_EXIT target={target} code={process.returncode}")
        try:
            content = result_path.read_text(encoding="utf-8-sig").strip()
        except OSError as exc:
            _log("RESULT_READ_FAILED\n" + traceback.format_exc())
            raise RuntimeError(
                "无法读取管理员进程的切换结果，网络可能已发生变化。"
                f"\n请查看按钮提示中的当前网络状态。\n日志：{LOG_PATH}"
            ) from exc
        if not content:
            detail = process.stderr.strip() or process.stdout.strip()
            _log(f"NO_RESULT target={target} detail={detail}")
            raise RuntimeError("管理员授权已取消，或切换进程未能启动。" + (f"\n{detail}" if detail else ""))
        try:
            result = json.loads(content)
        except ValueError as exc:
            _log("RESULT_PARSE_FAILED\n" + traceback.format_exc())
            raise RuntimeError(f"切换结果格式异常，请查看日志：{LOG_PATH}") from exc
        if process.returncode or not result.get("ok"):
            _log(f"SWITCH_FAILED target={target} message={result.get('message')}")
            raise RuntimeError(result.get("message") or "网络切换失败。")
        _log(f"SWITCH_OK target={target} state={result['state']}")
        return result["state"]
