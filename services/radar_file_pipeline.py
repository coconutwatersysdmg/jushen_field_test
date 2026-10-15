"""Owned background ITL intake and explicitly configured vendor conversion."""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

from services.itl_converter import ConversionCancelled, VENDOR_CONVERTER_STATUS, convert_itl


class RadarFilePipeline:
    def __init__(self, root: Path, config: dict, *, log=None, converter=convert_itl):
        self.root = Path(root).resolve()
        self.config = config
        self.itl_dir = self.root / config.get('local_itl_dir', '存itl')
        self.pcd_dir = self.root / config.get('pcd_dir', 'pcd')
        self.log_path = self.root / 'logs' / 'radar_sync.log'
        self.state_path = self.root / 'logs' / 'radar_conversion_state.json'
        for folder in (self.itl_dir, self.pcd_dir, self.log_path.parent):
            folder.mkdir(parents=True, exist_ok=True)
        self.log = log or self._log
        self.command = config.get('converter_command', [])
        self.converter = converter
        self.stable_seconds = max(0.0, float(config.get('stable_seconds', 3)))
        self.timeout = max(1.0, float(config.get('conversion_timeout_seconds', 120)))
        self.stop_event = threading.Event()
        self._log_lock = threading.Lock()
        self._observations = {}
        self._retry_after = {}
        self._unavailable = set()
        self._threads = []
        self.process = None
        self.state = {}
        if self.state_path.exists():
            try:
                self.state = json.loads(self.state_path.read_text(encoding='utf-8'))
                if not isinstance(self.state, dict):
                    raise ValueError('expected object')
            except (OSError, ValueError) as exc:
                self.state = {}
                self.log(f'转换状态文件无法读取，将重新检查本地 ITL：{exc}')

    @classmethod
    def from_project(cls, root):
        config_path = Path(root) / 'config' / 'radar_pipeline.json'
        return cls(root, json.loads(config_path.read_text(encoding='utf-8')))

    def _log(self, message):
        line = f"[{datetime.now().isoformat(timespec='seconds')}] RADAR {message}"
        with self._log_lock:
            with self.log_path.open('a', encoding='utf-8') as stream:
                stream.write(line + '\n')
            try:
                print(line, flush=True)
            except (OSError, UnicodeError):
                pass

    @staticmethod
    def _fingerprint(path):
        stat = path.stat()
        return f'{stat.st_size}:{stat.st_mtime_ns}'

    def _save_state(self):
        temp = self.state_path.with_suffix('.tmp')
        temp.write_text(json.dumps(self.state, ensure_ascii=False, indent=2), encoding='utf-8')
        os.replace(temp, self.state_path)

    def scan_once(self, *, now=None):
        now = time.monotonic() if now is None else now
        for source in sorted(self.itl_dir.glob('*')):
            if self.stop_event.is_set():
                break
            if source.suffix.lower() != '.itl' or not source.is_file():
                continue
            try:
                fingerprint = self._fingerprint(source)
                if source.stat().st_size == 0:
                    continue
                key = source.name
                observed = self._observations.get(key)
                if observed is None or observed[0] != fingerprint:
                    self._observations[key] = (fingerprint, now)
                    continue
                if now - observed[1] < self.stable_seconds:
                    continue
                done = self.state.get(key, {})
                if done.get('fingerprint') == fingerprint and Path(done.get('output', '')).is_file():
                    continue
                failure_key = (key, fingerprint)
                if not self.command:
                    if failure_key not in self._unavailable:
                        self.log(f'{key}: {VENDOR_CONVERTER_STATUS}')
                        self._unavailable.add(failure_key)
                    continue
                if now < self._retry_after.get(failure_key, 0):
                    continue
                target = self.pcd_dir / (source.stem + '.pcd')
                if target.exists():
                    suffix = hashlib.sha256(fingerprint.encode()).hexdigest()[:12]
                    target = self.pcd_dir / f'{source.stem}_{suffix}.pcd'
                    counter = 1
                    while target.exists():
                        target = self.pcd_dir / f'{source.stem}_{suffix}_{counter}.pcd'
                        counter += 1
                self.log(f'开始转换 {source.name} → {target.name}')
                self.converter(source, target, self.command, timeout_s=self.timeout, cancel_event=self.stop_event)
                # A source replaced during conversion is queued again, not marked current.
                self.state[key] = {'fingerprint': fingerprint, 'output': str(target.resolve())}
                self._save_state()
                self.log(f'转换成功 {target}')
            except ConversionCancelled:
                break
            except Exception as exc:
                self._retry_after[(source.name, locals().get('fingerprint', ''))] = now + 60
                self.log(f'转换失败 {source.name}：{exc}；60 秒后重试')

    def _convert_loop(self):
        while not self.stop_event.is_set():
            try:
                self.scan_once()
            except Exception as exc:
                self.log(f'扫描本地 ITL 失败：{exc}')
            self.stop_event.wait(1)

    def _read_sync_output(self):
        for line in self.process.stdout:
            self.log(line.strip())
        if not self.stop_event.is_set():
            self.log(f'ITL 监听进程退出，code={self.process.wait()}')

    def start(self):
        if not self.config.get('enabled', True) or self._threads:
            return
        self.log(f'监听启动：{self.config.get("source_share", "")} → {self.itl_dir}；PCD：{self.pcd_dir}')
        if os.name == 'nt':
            script = self.root / 'scripts' / 'radar_sync.ps1'
            command = ['powershell.exe', '-NoProfile', '-WindowStyle', 'Hidden', '-ExecutionPolicy', 'Bypass',
                       '-File', str(script), '-SourcePath', self.config['source_share'],
                       '-DestinationPath', str(self.itl_dir),
                       '-IntervalSeconds', str(max(1, int(self.config.get('poll_seconds', 10)))),
                       '-StableSeconds', str(self.stable_seconds)]
            if self.config.get('sync_existing', False):
                command.append('-SyncExisting')
            try:
                self.process = subprocess.Popen(command, creationflags=subprocess.CREATE_NO_WINDOW,
                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    encoding='utf-8', errors='replace')
                reader = threading.Thread(target=self._read_sync_output, name='radar-sync-log', daemon=True)
                self._threads.append(reader)
                reader.start()
            except OSError as exc:
                self.log(f'ITL 监听启动失败：{exc}')
        worker = threading.Thread(target=self._convert_loop, name='radar-converter', daemon=True)
        self._threads.append(worker)
        worker.start()
        if not self.command:
            self.log(VENDOR_CONVERTER_STATUS)

    def stop(self):
        self.stop_event.set()
        if self.process is not None and self.process.poll() is None:
            try:
                if os.name == 'nt':
                    subprocess.run(['taskkill', '/PID', str(self.process.pid), '/T', '/F'],
                        creationflags=subprocess.CREATE_NO_WINDOW, stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL, timeout=4)
                else:
                    self.process.terminate()
                self.process.wait(timeout=2)
            except (OSError, subprocess.TimeoutExpired):
                self.process.kill()
        for thread in self._threads:
            thread.join(timeout=3)
        if self.process is not None and self.process.stdout is not None:
            self.process.stdout.close()
        if self._threads:
            self.log('监听与转换后台已停止')
        self._threads.clear()
