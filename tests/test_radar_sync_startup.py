# -*- coding: utf-8 -*-
from __future__ import annotations

import contextlib
import io
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import main


class RadarSyncStartupTests(unittest.TestCase):
    def setUp(self):
        self.enterContext(patch.object(main, "_RADAR_SYNC_STARTED", False, create=True))
        self.log = self.enterContext(patch.object(main, "_log_startup"))
        # Never contact the industrial PC or start an endless sync loop in tests.
        self.popen = self.enterContext(patch("subprocess.Popen"))

    def test_windows_launch_is_hidden_nonblocking_and_only_happens_once(self):
        with patch("platform.system", return_value="Windows"):
            main._start_radar_sync()
            main._start_radar_sync()

        self.popen.assert_called_once()
        args, kwargs = self.popen.call_args
        self.assertEqual(args[0], [
            "powershell.exe", "-WindowStyle", "Hidden",
            "-ExecutionPolicy", "Bypass", "-File",
            str(Path(__file__).resolve().parents[1] / "scripts" / "radar_sync.ps1"),
        ])
        self.assertFalse(kwargs.get("shell", False))
        self.assertTrue(kwargs["creationflags"] & subprocess.CREATE_NO_WINDOW)
        self.popen.return_value.wait.assert_not_called()
        self.popen.return_value.communicate.assert_not_called()

    def test_other_platforms_do_not_launch(self):
        for system in ("Linux", "Darwin"):
            with self.subTest(system=system), patch("platform.system", return_value=system):
                main._start_radar_sync()

        self.popen.assert_not_called()

    def test_launch_failure_is_logged_without_crashing_and_allows_retry(self):
        self.popen.side_effect = [OSError("PowerShell unavailable"), self.popen.return_value]
        errors = io.StringIO()
        with patch("platform.system", return_value="Windows"), contextlib.redirect_stderr(errors):
            main._start_radar_sync()
            main._start_radar_sync()
            main._start_radar_sync()

        self.assertIn("PowerShell unavailable", errors.getvalue())
        self.assertTrue(any("PowerShell unavailable" in call.args[0] for call in self.log.call_args_list))
        self.assertEqual(self.popen.call_count, 2)

    def test_script_path_does_not_depend_on_working_directory(self):
        with tempfile.TemporaryDirectory(prefix="radar sync 中文 ") as folder:
            with contextlib.chdir(folder), patch("platform.system", return_value="Windows"):
                main._start_radar_sync()

        script = Path(self.popen.call_args.args[0][-1])
        self.assertTrue(script.is_absolute())
        self.assertTrue(script.is_file())
        self.assertEqual(script.parent.parent, Path(main.__file__).resolve().parent)


if __name__ == "__main__":
    unittest.main()
