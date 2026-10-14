"""Independent local ITL watcher. Conversion failures never gate the GUI flow."""
from __future__ import annotations

import json
import configparser
import math
import os
import threading
import time
from pathlib import Path

from utils.itl_to_pcd import CONFIG_PATH, convert_itl_to_pcd, load_calibration

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class ItlConversionWatcher:
    def __init__(self, source_dir: Path = PROJECT_ROOT / "存itl", output_dir: Path = PROJECT_ROOT / "PCD", *,
                 state_path: Path = PROJECT_ROOT / "workdir" / "itl_conversion_state.json",
                 poll_seconds: float = 1, stable_seconds: float = 2, retry_seconds: float = 10,
                 converter=convert_itl_to_pcd, log=None):
        if (not all(math.isfinite(x) for x in (poll_seconds, stable_seconds, retry_seconds))
                or poll_seconds <= 0 or stable_seconds < 0 or retry_seconds < 0):
            raise ValueError("Invalid ITL watcher polling/stability/retry interval")
        self.source_dir, self.output_dir, self.state_path = Path(source_dir), Path(output_dir), Path(state_path)
        self.poll_seconds, self.stable_seconds, self.retry_seconds = poll_seconds, stable_seconds, retry_seconds
        self.converter = converter
        self.log = log or (lambda message: print(message, flush=True))
        self.calibration = load_calibration()
        self._stop = threading.Event()
        self._observed = {}
        self._retry_at = {}
        self._completed = {}
        self.thread = None
        self.source_dir.mkdir(parents=True, exist_ok=True)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        if self.state_path.is_file():
            try:
                state = json.loads(self.state_path.read_text(encoding="utf-8"))
                if state.get("profile") == self.calibration.profile_id:
                    self._completed = dict(state.get("files") or {})
            except Exception as exc:
                self.log(f"[ITL_CONVERT] 转换记录读取失败，将重新检查文件：{exc}")

    @classmethod
    def from_config(cls, *, log=None):
        config = configparser.ConfigParser()
        with CONFIG_PATH.open(encoding="utf-8-sig") as stream:
            config.read_file(stream)
        return cls(poll_seconds=config.getfloat("Watcher", "POLL_SECONDS"),
                   stable_seconds=config.getfloat("Watcher", "STABLE_SECONDS"),
                   retry_seconds=config.getfloat("Watcher", "RETRY_SECONDS"), log=log)

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return
        self._stop.clear()
        self.thread = threading.Thread(target=self._run, name="itl-to-pcd", daemon=True)
        self.thread.start()

    def stop(self):
        self._stop.set()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=3)

    def _save_state(self):
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.state_path.with_suffix(".tmp")
        temporary.write_text(json.dumps({"version": 1, "profile": self.calibration.profile_id,
                                         "files": self._completed}, ensure_ascii=False), encoding="utf-8")
        os.replace(temporary, self.state_path)

    def scan_once(self, *, now=None):
        now = time.monotonic() if now is None else now
        sources = [path for path in self.source_dir.iterdir() if path.is_file() and path.suffix.lower() == ".itl"]
        present = {path.name for path in sources}
        self._observed = {name: state for name, state in self._observed.items() if name in present}
        self._retry_at = {name: stamp for name, stamp in self._retry_at.items() if name in present}
        for source in sources:
            if self._stop.is_set():
                break
            try:
                stat = source.stat()
                signature = [stat.st_size, stat.st_mtime_ns]
                destination = self.output_dir / (source.stem + ".pcd")
                if self._completed.get(source.name) == signature and destination.is_file():
                    continue
                previous = self._observed.get(source.name)
                if previous is None or previous[0] != signature:
                    self._observed[source.name] = (signature, now)
                    self._retry_at.pop(source.name, None)
                    self.log(f"[ITL_CONVERT] 发现新文件或更新：{source.name}；等待写入完成")
                stable_since = self._observed[source.name][1]
                if now - stable_since < self.stable_seconds or now < self._retry_at.get(source.name, 0):
                    continue
                self.log(f"[ITL_CONVERT] 开始转换：{source} -> {destination}")
                result = self.converter(source, destination, calibration=self.calibration)
                self._completed[source.name] = signature
                self._retry_at.pop(source.name, None)
                try:
                    self._save_state()
                except Exception as exc:
                    self.log(f"[ITL_CONVERT] 转换完成，但记录保存失败：{exc}")
                self.log(f"[ITL_CONVERT] 转换成功：{destination.name}；{result['points']:,} 点")
            except Exception as exc:
                self._retry_at[source.name] = now + self.retry_seconds
                self.log(f"[ITL_CONVERT] 转换失败：{source.name}：{exc}；稍后重试，不影响手动选择 PCD")

    def _run(self):
        self.log(f"[ITL_CONVERT] 本地自动转换已启动：{self.source_dir} -> {self.output_dir}")
        while not self._stop.is_set():
            try:
                self.scan_once()
            except Exception as exc:
                self.log(f"[ITL_CONVERT] 本地目录扫描失败：{exc}；稍后重试，不影响手动选择 PCD")
            self._stop.wait(self.poll_seconds)
        self.log("[ITL_CONVERT] 自动转换已停止")
