from __future__ import annotations

import os
import contextlib
import io
import queue
import shutil
import subprocess
import tempfile
import threading
import time
import unittest
from datetime import datetime
from pathlib import Path
from unittest.mock import patch

import main


@unittest.skipUnless(os.name == "nt", "Windows PowerShell sync integration")
class RadarSyncScriptTests(unittest.TestCase):
    def setUp(self):
        self.folder = self.enterContext(tempfile.TemporaryDirectory(prefix="radar sync 中文 "))
        self.source = Path(self.folder) / "工控机 wave"
        self.target = Path(self.folder) / "存itl"
        self.source.mkdir()
        self.target.mkdir()

    def launch(self, scans=1, source=None):
        script = Path(__file__).resolve().parents[1] / "scripts" / "radar_sync.ps1"
        process = subprocess.Popen([
            "powershell.exe", "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass",
            "-File", str(script), "-RemoteShare", str(source or self.source),
            "-LocalPath", str(self.target), "-Interval", "0.2", "-MaxScans", str(scans),
            "-SkipAuthentication",
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding="utf-8", errors="replace")
        def cleanup():
            if process.poll() is None:
                process.kill()
            process.wait(timeout=5)
            process.stdout.close()
        self.addCleanup(cleanup)
        return process

    def test_copies_all_names_and_types_after_cutoff_without_overwriting_existing(self):
        for name in ("点云.PCD", "数据.itl", "任意 [1].txt", "无扩展名"):
            path = self.source / name
            path.write_bytes(name.encode("utf-8"))
            timestamp = datetime(2026, 10, 1).timestamp()
            os.utime(path, (timestamp, timestamp))
        (self.target / "数据.itl").write_text("stale local content", encoding="utf-8")
        process = self.launch()
        output, _ = process.communicate(timeout=15)
        self.assertEqual(process.returncode, 0, output)
        for source in self.source.iterdir():
            expected = b"stale local content" if source.name == "数据.itl" else source.read_bytes()
            self.assertEqual((self.target / source.name).read_bytes(), expected, output)
        self.assertIn("Copied:", output)
        self.assertIn("Scan #1 complete", output)

    def test_capture_date_in_filename_overrides_mtime_and_boundary_is_included(self):
        cases = {
            "LaserData-socket1-2025-08-27-13-37-21.itl": (datetime(2026, 10, 15), False),
            "LaserData-socket1-2026-09-30-23-59-59.itl": (datetime(2026, 10, 15), False),
            "LaserData-socket1-2026-10-01-00-00-00.itl": (datetime(2025, 1, 1), True),
            "date-2026-10-15.bin": (datetime(2025, 1, 1), True),
            "undated-old.txt": (datetime(2026, 9, 30, 23, 59, 59), False),
            "undated-new.txt": (datetime(2026, 10, 1), True),
        }
        for name, (date, _) in cases.items():
            path = self.source / name
            path.write_bytes(name.encode("utf-8"))
            os.utime(path, (date.timestamp(), date.timestamp()))
        process = self.launch()
        output, _ = process.communicate(timeout=15)
        self.assertEqual(process.returncode, 0, output)
        for name, (_, expected) in cases.items():
            self.assertEqual((self.target / name).exists(), expected, f"{name}\n{output}")
        self.assertIn("2026-10-01", output)
        self.assertIn("before_cutoff=3", output)

    def test_latest_capture_is_copied_first(self):
        older = "LaserData-2026-10-02-10-00-00.itl"
        newer = "LaserData-2026-10-15-10-00-00.itl"
        for name in (older, newer):
            (self.source / name).write_bytes(b"data")
        process = self.launch()
        output, _ = process.communicate(timeout=15)
        self.assertEqual(process.returncode, 0, output)
        self.assertLess(output.index(f"Copying: {newer}"), output.index(f"Copying: {older}"))

    def test_file_added_after_start_with_recent_capture_date_and_old_mtime_is_copied(self):
        process = self.launch(scans=5)
        messages = queue.Queue()
        reader = threading.Thread(target=lambda: [messages.put(line) for line in process.stdout], daemon=True)
        reader.start()
        seen = []
        deadline = time.monotonic() + 15
        while not any("Scan #1 complete" in line for line in seen):
            remaining = deadline - time.monotonic()
            self.assertGreater(remaining, 0, "\n".join(seen))
            seen.append(messages.get(timeout=max(0.01, remaining)))
        fixture = Path(self.folder) / "old-file.txt"
        fixture.write_bytes(b"manually copied with old timestamp")
        os.utime(fixture, (1_500_000_000, 1_500_000_000))
        name = "新文件-2026-10-15-09-00-00 [old].anything"
        shutil.copy2(fixture, self.source / name)
        process.wait(timeout=15)
        reader.join(timeout=2)
        self.assertEqual(process.returncode, 0)
        self.assertEqual((self.target / name).read_bytes(), fixture.read_bytes())

    def test_inaccessible_source_is_reported_and_retried(self):
        process = self.launch(scans=2, source=Path(self.folder) / "missing-share")
        output, _ = process.communicate(timeout=15)
        self.assertEqual(process.returncode, 0, output)
        self.assertIn("Remote path not accessible", output)
        self.assertIn("Scan #2", output)

    def test_real_child_output_reaches_python_console_and_startup_log(self):
        (self.source / "桥接日志.txt").write_bytes(b"bridge test")
        process = self.launch()
        console = io.StringIO()
        log = Path(self.folder) / "startup.log"
        with patch.object(main, "STARTUP_LOG", log), contextlib.redirect_stdout(console):
            thread = threading.Thread(target=main._forward_radar_sync_output, args=(process,), daemon=True)
            thread.start()
            thread.join(timeout=15)
            self.assertFalse(thread.is_alive(), "PowerShell output forwarding did not finish")
        for output in (console.getvalue(), log.read_text(encoding="utf-8")):
            self.assertIn("桥接日志.txt", output)
            self.assertIn("Copied:", output)
            self.assertIn("Scan #1 complete", output)
            self.assertIn("exited code=0", output)

    def test_file_still_open_for_writing_is_retried_after_writer_closes(self):
        source = self.source / "still-writing.pcd"
        escaped = str(source).replace("'", "''")
        # Match a producer that permits readers while a preallocated file is open.
        code = (
            "$initial = [byte[]]::new(1024); $initial[0] = 65; "
            f"[IO.File]::WriteAllBytes('{escaped}', $initial); "
            f"$writer = [IO.File]::Open('{escaped}', 'Open', 'ReadWrite', 'ReadWrite'); "
            "[Console]::WriteLine('ready'); [Console]::ReadLine() | Out-Null; "
            "$writer.Position = 1; $writer.WriteByte(66); $writer.Dispose()"
        )
        producer = subprocess.Popen([
            "powershell.exe", "-NoProfile", "-NonInteractive", "-Command", code,
        ], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            encoding="utf-8", errors="replace")
        try:
            self.assertEqual(producer.stdout.readline().strip(), "ready")
            process = self.launch(scans=2)
            output, _ = process.communicate(timeout=15)
            self.assertEqual(process.returncode, 0, output)
            self.assertFalse((self.target / source.name).exists(), output)
            self.assertIn("Copy failed:", output)
        finally:
            producer.communicate(input="\n", timeout=5)
        process = self.launch()
        output, _ = process.communicate(timeout=15)
        self.assertEqual(process.returncode, 0, output)
        self.assertEqual((self.target / source.name).read_bytes(), source.read_bytes())
        self.assertEqual((self.target / source.name).read_bytes()[:2], b"AB")
