import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

from services.radar_file_pipeline import RadarFilePipeline
from tests.test_itl_converter import VALID_PCD


class RadarPipelineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.logs = []
        self.calls = []

        def convert(source, destination, command_template, **kwargs):
            self.calls.append(source.name)
            destination.write_text('converted ' + source.read_text())
            return destination

        self.config = {'stable_seconds': 2, 'converter_command': ['fake', '{input}', '{output}']}
        self.worker = RadarFilePipeline(self.root, self.config, log=self.logs.append, converter=convert)

    def test_waits_for_stability_and_skips_unchanged_success(self):
        source = self.root / '存itl' / '车厢.itl'
        source.write_text('first')
        self.worker.scan_once(now=0)
        self.worker.scan_once(now=1)
        self.assertEqual([], self.calls)
        self.worker.scan_once(now=2)
        self.worker.scan_once(now=4)
        self.assertEqual(['车厢.itl'], self.calls)
        self.assertTrue((self.root / 'pcd' / '车厢.pcd').exists())
        restarted = RadarFilePipeline(self.root, self.config, log=self.logs.append, converter=self.worker.converter)
        restarted.scan_once(now=5)
        restarted.scan_once(now=8)
        self.assertEqual(1, len(self.calls))

    def test_changed_file_restarts_wait_and_preserves_previous_pcd(self):
        source = self.root / '存itl' / 'sample.itl'
        source.write_text('one')
        self.worker.scan_once(now=0)
        source.write_text('second version')
        self.worker.scan_once(now=2)
        self.assertEqual([], self.calls)
        self.worker.scan_once(now=4)
        source.write_text('third version more data')
        self.worker.scan_once(now=5)
        self.worker.scan_once(now=7)
        self.assertEqual(2, len(self.calls))
        self.assertEqual('converted second version', (self.root / 'pcd' / 'sample.pcd').read_text())
        self.assertEqual(2, len(list((self.root / 'pcd').glob('*.pcd'))))

    def test_unconfigured_converter_logs_once_without_fabricating_output(self):
        self.worker.command = []
        (self.root / '存itl' / 'sample.itl').write_text('data')
        for now in (0, 2, 4, 6):
            self.worker.scan_once(now=now)
        self.assertEqual([], self.calls)
        self.assertEqual(1, len([line for line in self.logs if '转换未配置' in line]))
        self.assertFalse(list((self.root / 'pcd').glob('*.pcd')))

    def test_retry_failure_backoff_and_state_only_on_success(self):
        def fail(*args, **kwargs):
            self.calls.append('fail')
            raise RuntimeError('dependency missing')
        self.worker.converter = fail
        (self.root / '存itl' / 'sample.itl').write_text('data')
        for now in (0, 2, 4, 8):
            self.worker.scan_once(now=now)
        self.assertEqual(['fail'], self.calls)
        self.worker.scan_once(now=70)
        self.assertEqual(2, len(self.calls))
        self.assertEqual({}, self.worker.state)

    def test_state_parse_failure_is_reported_and_recovers(self):
        state = self.root / 'logs' / 'radar_conversion_state.json'
        state.write_text('{broken')
        worker = RadarFilePipeline(self.root, self.config, log=self.logs.append)
        self.assertEqual({}, worker.state)
        self.assertTrue(any('状态文件' in msg for msg in self.logs))

    @unittest.skipUnless(os.name == 'nt', 'Windows PowerShell watcher')
    def test_actual_watcher_pipeline_conversion_and_shutdown(self):
        remote = self.root / 'remote 中文'
        remote.mkdir()
        (remote / 'new scan.itl').write_text('vendor test fixture')
        (remote / 'ignore.txt').write_text('not radar')
        scripts = self.root / 'scripts'
        scripts.mkdir()
        shutil.copyfile(Path(__file__).resolve().parents[1] / 'scripts' / 'radar_sync.ps1', scripts / 'radar_sync.ps1')
        converter = self.root / 'test converter.py'
        converter.write_text('import pathlib,sys\npathlib.Path(sys.argv[2]).write_bytes(' + repr(VALID_PCD) + ')\n')
        cfg = dict(self.config, source_share=str(remote), sync_existing=True, poll_seconds=1,
                   stable_seconds=0, converter_command=[sys.executable, str(converter), '{input}', '{output}'])
        worker = RadarFilePipeline(self.root, cfg)
        self.addCleanup(worker.stop)
        worker.start()
        threads = list(worker._threads)
        output = self.root / 'pcd' / 'new scan.pcd'
        deadline = time.monotonic() + 10
        while not output.exists() and time.monotonic() < deadline:
            time.sleep(.1)
        worker.stop()
        self.assertTrue(output.exists(), worker.log_path.read_text(encoding='utf-8'))
        self.assertEqual(VALID_PCD, output.read_bytes())
        self.assertIsNotNone(worker.process.poll())
        self.assertFalse(any(thread.is_alive() for thread in threads))
        self.assertFalse((self.root / '存itl' / 'ignore.txt').exists())
        logs = worker.log_path.read_text(encoding='utf-8')
        for expected in ('CONNECTED', 'COPIED', '转换成功', '已停止'):
            self.assertIn(expected, logs)

    @unittest.skipUnless(os.name == 'nt', 'Windows PowerShell watcher')
    def test_locked_source_does_not_starve_other_itl(self):
        import ctypes
        from ctypes import wintypes
        source = self.root / 'remote'
        source.mkdir()
        locked = source / 'a.itl'
        locked.write_text('held exclusive')
        (source / 'b.itl').write_text('readable')
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p,
                                     wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
        kernel.CreateFileW.restype = wintypes.HANDLE
        kernel.CloseHandle.argtypes = [wintypes.HANDLE]
        handle = kernel.CreateFileW(str(locked), 0x80000000, 0, None, 3, 0, None)
        self.assertNotEqual(handle, ctypes.c_void_p(-1).value)
        try:
            result = subprocess.run(['powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                str(Path(__file__).resolve().parents[1] / 'scripts' / 'radar_sync.ps1'),
                '-SourcePath', str(source), '-DestinationPath', str(self.root / 'copied'),
                '-SyncExisting', '-StableSeconds', '0', '-IntervalSeconds', '1', '-MaxScans', '3'],
                capture_output=True, encoding='utf-8', errors='replace', timeout=12)
        finally:
            kernel.CloseHandle(handle)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn('FILE_ERROR', result.stdout)
        self.assertEqual('readable', (self.root / 'copied' / 'b.itl').read_text())
        self.assertEqual([], list((self.root / 'copied').glob('*.part')))


if __name__ == '__main__':
    unittest.main()
