from __future__ import annotations

import struct
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

from utils.itl_to_pcd import convert_itl_to_pcd
from services.itl_conversion_service import ItlConversionWatcher
from algorithm_modules.point_cloud_segment_module.inference import load_pcd
import main


def write_itl(path: Path, encoder=7200, distances=None):
    frame = bytearray(2980)
    frame[:4] = b"\x02" * 4
    struct.pack_into(">I", frame, 4, 2971)
    frame[8:24] = b"sSN LMDscandata "
    frame[64:69] = b"DIST1"
    struct.pack_into(">ffiHH", frame, 69, 1.0, 0.0, 0, 12500, 1441)
    ranges = np.zeros(1441, dtype=">u2")
    for index, value in (distances or {}).items():
        ranges[index] = value
    frame[85:2967] = ranges.tobytes()
    struct.pack_into(">h", frame, 2978, encoder)
    path.write_bytes(frame)


class ItlConversionTests(unittest.TestCase):
    def setUp(self):
        folder = self.enterContext(tempfile.TemporaryDirectory(prefix="itl 中文 "))
        self.source = Path(folder) / "存itl"
        self.output = Path(folder) / "PCD"
        self.source.mkdir()
        self.output.mkdir()
        self.itl = self.source / "任意 [1].ITL"
        self.pcd = self.output / "任意 [1].pcd"

    def test_matches_vendor_coordinates_and_keeps_zero_returns(self):
        write_itl(self.itl, distances={0: 80, 720: 18750, 1440: 90})
        convert_itl_to_pcd(self.itl, self.pcd)
        points = load_pcd(str(self.pcd))
        self.assertEqual(points.shape, (1441, 3))
        # Independently taken from the paired vendor-produced ITL/PCD in to_pcd.zip.
        np.testing.assert_array_equal(points[[0, 720, 1440]], [
            [80, 91, 5799], [89, 17710, -614], [-89, 91, 5797],
        ])
        np.testing.assert_array_equal(points[1], [0, 91, 5798])

    def test_truncated_or_wrong_format_never_replaces_existing_pcd(self):
        self.pcd.write_bytes(b"existing good file")
        for data in (b"incomplete", b"\0" * 2980):
            self.itl.write_bytes(data)
            with self.assertRaises(ValueError):
                convert_itl_to_pcd(self.itl, self.pcd)
            self.assertEqual(self.pcd.read_bytes(), b"existing good file")
        self.assertEqual(list(self.output.iterdir()), [self.pcd])

    def test_waits_for_stable_file_and_converts_existing_files_without_time_cutoff(self):
        write_itl(self.itl)
        watcher = ItlConversionWatcher(self.source, self.output, state_path=self.output / "state.json", stable_seconds=2)
        watcher.scan_once(now=0)
        self.assertFalse(self.pcd.exists())
        watcher.scan_once(now=1)
        self.assertFalse(self.pcd.exists())
        watcher.scan_once(now=2)
        self.assertTrue(self.pcd.is_file())

    def test_conversion_failure_retries_and_does_not_block_other_files(self):
        write_itl(self.itl)
        other = self.source / "正常.itl"
        write_itl(other, encoder=0)
        attempts = []

        def convert(source, destination, **kwargs):
            attempts.append(source.name)
            if source == self.itl and attempts.count(source.name) == 1:
                raise PermissionError("source still being written")
            return convert_itl_to_pcd(source, destination, **kwargs)

        log = Mock()
        watcher = ItlConversionWatcher(self.source, self.output, state_path=self.output / "state.json",
                                       stable_seconds=0, retry_seconds=1, converter=convert, log=log)
        watcher.scan_once(now=0)
        self.assertTrue((self.output / "正常.pcd").is_file())
        self.assertFalse(self.pcd.exists())
        watcher.scan_once(now=2)
        self.assertTrue(self.pcd.is_file())
        self.assertTrue(any("source still being written" in call.args[0] for call in log.call_args_list))

    def test_completed_files_are_not_reconverted_after_restart_but_updates_are(self):
        write_itl(self.itl)
        state = self.output / "state.json"
        convert = Mock(wraps=convert_itl_to_pcd)
        watcher = ItlConversionWatcher(self.source, self.output, state_path=state, stable_seconds=0, converter=convert)
        watcher.scan_once(now=0)
        restarted = ItlConversionWatcher(self.source, self.output, state_path=state, stable_seconds=0, converter=convert)
        restarted.scan_once(now=1)
        self.assertEqual(convert.call_count, 1)
        write_itl(self.itl, encoder=0, distances={0: 100})
        restarted.scan_once(now=2)
        self.assertEqual(convert.call_count, 2)

    def test_background_thread_starts_once_and_can_stop(self):
        watcher = ItlConversionWatcher(self.source, self.output, state_path=self.output / "state.json", poll_seconds=.02)
        watcher.start()
        try:
            first = watcher.thread
            watcher.start()
            self.assertIs(watcher.thread, first)
        finally:
            watcher.stop()
        self.assertFalse(first.is_alive())

    def test_itl_still_open_for_writing_is_not_published(self):
        write_itl(self.itl)
        if os.name != "nt":
            self.skipTest("Windows file-sharing lock")
        with self.itl.open("r+b") as writer:
            with self.assertRaises(OSError):
                convert_itl_to_pcd(self.itl, self.pcd)
            self.assertFalse(self.pcd.exists())
        convert_itl_to_pcd(self.itl, self.pcd)
        self.assertTrue(self.pcd.is_file())

    def test_startup_conversion_failure_is_logging_only(self):
        with patch.object(main, "_ITL_CONVERSION_WATCHER", None), \
             patch.object(main, "_log_startup") as log, \
             patch.object(ItlConversionWatcher, "from_config", side_effect=ValueError("bad calibration")):
            main._start_itl_conversion()
            self.assertTrue(any("bad calibration" in call.args[0] for call in log.call_args_list))
