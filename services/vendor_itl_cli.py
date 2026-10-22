"""Offline adapter for the observed Intelly converter controls, Windows only.

Every native message is restricted to windows owned by the executable this
adapter launches. There are no keyboard, mouse, networking, or device actions.
"""
from __future__ import annotations

import argparse
import ctypes
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from services.itl_converter import ConversionError, ConverterUnavailable, _stop_process, validate_pcd


class _OwnedConverterWindow:
    OPEN_FILE = 1007
    SAVE_CLOUD = 1047
    DISPLAY_CLOUD = 1010
    FILENAME = 1148
    OPEN_DIALOG = 1

    def __init__(self, pid: int):
        from ctypes import wintypes as W
        self.pid = pid
        self.W = W
        self.user = ctypes.WinDLL("user32", use_last_error=True)
        self.callback_type = ctypes.WINFUNCTYPE(W.BOOL, W.HWND, W.LPARAM)
        self.user.EnumWindows.argtypes = [self.callback_type, W.LPARAM]
        self.user.EnumChildWindows.argtypes = [W.HWND, self.callback_type, W.LPARAM]
        self.user.GetWindowThreadProcessId.argtypes = [W.HWND, ctypes.POINTER(W.DWORD)]
        self.user.GetDlgCtrlID.argtypes = [W.HWND]
        self.user.GetDlgItem.argtypes = [W.HWND, ctypes.c_int]
        self.user.GetDlgItem.restype = W.HWND
        self.user.GetClassNameW.argtypes = [W.HWND, W.LPWSTR, ctypes.c_int]
        self.user.GetWindowTextW.argtypes = [W.HWND, W.LPWSTR, ctypes.c_int]
        self.user.PostMessageW.argtypes = [W.HWND, W.UINT, W.WPARAM, W.LPARAM]
        self.user.SendMessageTimeoutW.argtypes = [W.HWND, W.UINT, W.WPARAM, W.LPARAM, W.UINT, W.UINT, ctypes.POINTER(ctypes.c_size_t)]
        self.user.ShowWindow.argtypes = [W.HWND, ctypes.c_int]

    def owns(self, window: int) -> bool:
        pid = self.W.DWORD()
        self.user.GetWindowThreadProcessId(window, ctypes.byref(pid))
        return pid.value == self.pid

    def windows(self) -> list[int]:
        result = []
        @self.callback_type
        def collect(window, _):
            if self.owns(window):
                result.append(window)
            return True
        self.user.EnumWindows(collect, 0)
        return result

    def children(self, parent: int) -> list[int]:
        result = []
        @self.callback_type
        def collect(window, _):
            if self.owns(window):
                result.append(window)
            return True
        self.user.EnumChildWindows(parent, collect, 0)
        return result

    def class_name(self, window: int) -> str:
        text = ctypes.create_unicode_buffer(256)
        self.user.GetClassNameW(window, text, len(text))
        return text.value

    def text(self, window: int) -> str:
        text = ctypes.create_unicode_buffer(4096)
        # GetWindowText cannot retrieve an Edit control in another process.
        self.send(window, 0x000D, len(text), ctypes.cast(text, ctypes.c_void_p).value)
        return text.value

    def send(self, window: int, message: int, wparam: int = 0, lparam: int = 0) -> int:
        if not self.owns(window):
            raise ConversionError("转换控件不属于当前转换进程")
        result = ctypes.c_size_t()
        if not self.user.SendMessageTimeoutW(window, message, wparam, lparam, 2, 1500, ctypes.byref(result)):
            raise ConversionError(f"厂商控件响应超时：{self.user.GetDlgCtrlID(window)}")
        return result.value

    def click(self, window: int) -> None:
        if not self.owns(window) or not self.user.PostMessageW(window, 0x00F5, 0, 0):
            raise ConversionError("无法触发当前转换进程的离线按钮")

    def control(self, parent: int, control_id: int) -> int | None:
        window = self.user.GetDlgItem(parent, control_id)
        return window if window and self.owns(window) else None

    def main_window(self) -> int | None:
        return next((window for window in self.windows() if self.control(window, self.OPEN_FILE) and self.control(window, self.SAVE_CLOUD)), None)

    def file_dialog(self) -> tuple[int, int] | None:
        for window in self.windows():
            if not self.control(window, self.OPEN_DIALOG):
                continue
            for child in self.children(window):
                if self.user.GetDlgCtrlID(child) == self.FILENAME and self.class_name(child) == "Edit":
                    return window, child
        return None

    def hide_owned(self) -> None:
        for window in self.windows():
            self.user.ShowWindow(window, 0)

    def diagnostic(self) -> str:
        return "; ".join(
            f"{self.text(window)} [{','.join(str(self.user.GetDlgCtrlID(child)) for child in self.children(window))}]"
            for window in self.windows()
        )[:4000]


def convert_vendor_itl(source: Path, output: Path, runtime_dir: Path, timeout_s: float = 90) -> Path:
    source, output, runtime_dir = (Path(p).expanduser().resolve() for p in (source, output, runtime_dir))
    if output.exists():
        raise ConversionError(f"PCD 输出已存在，拒绝覆盖：{output}")
    executable = runtime_dir / "BZKJDEMO_pcd.exe"
    required = [executable, runtime_dir / "IntellyLaserApp.ini", runtime_dir / "pcl_common_release.dll"]
    if not all(path.is_file() for path in required):
        raise ConverterUnavailable(f"厂商隔离运行包不完整：{runtime_dir}")
    if os.name != "nt":
        raise ConverterUnavailable("厂商 GUI 转换器仅支持 Windows")
    if not source.is_file() or source.suffix.lower() != ".itl" or not source.stat().st_size:
        raise ConversionError(f"ITL 输入无效：{source}")
    if output.suffix.lower() != ".pcd" or timeout_s <= 0:
        raise ConversionError("输出扩展名或转换超时无效")
    deadline = time.monotonic() + timeout_s
    with tempfile.TemporaryDirectory(prefix="vendor-itl-", ignore_cleanup_errors=True) as folder:
        job = Path(folder)
        for path in runtime_dir.iterdir():
            if path.is_file() and path.suffix.lower() in (".exe", ".dll", ".ini"):
                shutil.copy2(path, job / path.name)
        # The supplied vendor package keeps its raw files under wave/. Preserve
        # that offline layout, rather than depend on undocumented path handling.
        (job / "wave").mkdir()
        local_input = job / "wave" / (source.name if source.name.isascii() else "input.itl")
        shutil.copy2(source, local_input)
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = subprocess.SW_HIDE
        process = subprocess.Popen([str(job / executable.name)], cwd=job, startupinfo=startup)
        ui = _OwnedConverterWindow(process.pid)

        def wait_for(condition, label: str):
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise ConversionError(f"厂商转换程序提前退出：0x{process.returncode & 0xffffffff:08X}")
                ui.hide_owned()
                result = condition()
                if result:
                    return result
                time.sleep(0.1)
            raise ConversionError(f"厂商离线转换超时：{label}；{ui.diagnostic()}")

        try:
            main = wait_for(ui.main_window, "主窗口")
            save = ui.control(main, ui.SAVE_CLOUD)
            if ui.send(save, 0x00F0) != 1:
                ui.click(save)
                wait_for(lambda: ui.send(save, 0x00F0) == 1, "保存点云复选框")
            print("vendor: save enabled, opening local ITL", flush=True)
            ui.click(ui.control(main, ui.OPEN_FILE))
            dialog, edit = wait_for(ui.file_dialog, "离线文件对话框")
            filename = ctypes.create_unicode_buffer(str(local_input))
            ui.send(edit, 0x000C, 0, ctypes.cast(filename, ctypes.c_void_p).value)
            if ui.text(edit) != str(local_input):
                raise ConversionError("厂商文件对话框未接受指定本地 ITL 路径")
            ui.click(ui.control(dialog, ui.OPEN_DIALOG))
            wait_for(lambda: ui.file_dialog() is None, "载入 ITL")

            def candidate() -> Path | None:
                paths = [p for p in job.rglob("*.pcd") if p.is_file() and p.stat().st_size > 0]
                return max(paths, key=lambda p: p.stat().st_mtime_ns) if paths else None

            # Opening may convert immediately. Display only when it has not.
            display_after = time.monotonic() + 10
            while time.monotonic() < min(display_after, deadline) and candidate() is None:
                if process.poll() is not None:
                    raise ConversionError(f"厂商载入 ITL 后退出：0x{process.returncode & 0xffffffff:08X}")
                time.sleep(0.1)
            if candidate() is None:
                log = ui.control(main, 1009)
                if log:
                    print("vendor: load log: " + ui.text(log), flush=True)
                print("vendor: ITL loaded, displaying cloud", flush=True)
                ui.click(ui.control(main, ui.DISPLAY_CLOUD))
            generated = wait_for(candidate, "生成 PCD")
            previous = None
            stable_since = time.monotonic()
            while time.monotonic() < deadline:
                current = (generated.stat().st_size, generated.stat().st_mtime_ns)
                if current != previous:
                    previous, stable_since = current, time.monotonic()
                elif time.monotonic() - stable_since >= 1:
                    break
                time.sleep(0.1)
            else:
                raise ConversionError("厂商 PCD 输出写入未完成")
            validate_pcd(generated)
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open("xb") as destination, generated.open("rb") as source_stream:
                shutil.copyfileobj(source_stream, destination)
            print(f"vendor: generated {generated.name}, {output.stat().st_size} bytes", flush=True)
            return output
        finally:
            _stop_process(process)
            # Windows may briefly retain the image mapping after process exit.
            time.sleep(0.2)


def main(argv: list[str] | None = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--runtime-dir", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=90)
    args = parser.parse_args(argv)
    try:
        convert_vendor_itl(args.input, args.output, args.runtime_dir, args.timeout)
    except (ConversionError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
