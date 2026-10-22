import contextlib
import io
import tempfile
import unittest
from unittest.mock import patch
import main

class RadarSyncStartupTests(unittest.TestCase):
    def setUp(self):
        self.enterContext(patch("config.feature_switches.RADAR_ONLY_TEST", False))
        self.enterContext(patch.object(main, '_RADAR_PIPELINE', None))
        self.log = self.enterContext(patch.object(main, '_log_startup'))
        self.factory = self.enterContext(patch('services.radar_file_pipeline.RadarFilePipeline.from_project'))
        self.pipeline = self.factory.return_value

    def test_only_one_owned_pipeline_is_started(self):
        main._start_radar_sync()
        main._start_radar_sync()
        self.factory.assert_called_once_with(main.PROJECT_ROOT)
        self.pipeline.start.assert_called_once()

    def test_shutdown_stops_owned_workers_and_allows_new_start(self):
        main._start_radar_sync()
        main._stop_radar_sync()
        main._stop_radar_sync()
        self.pipeline.stop.assert_called_once()
        self.assertIsNone(main._RADAR_PIPELINE)
        main._start_radar_sync()
        self.assertEqual(2, self.pipeline.start.call_count)

    def test_config_failure_is_logged_and_retry_is_possible(self):
        self.factory.side_effect = [OSError('config unavailable'), self.pipeline]
        errors = io.StringIO()
        with contextlib.redirect_stderr(errors):
            main._start_radar_sync()
            main._start_radar_sync()
        self.assertIn('config unavailable', errors.getvalue())
        self.pipeline.start.assert_called_once()

    def test_project_path_does_not_depend_on_working_directory(self):
        with tempfile.TemporaryDirectory(prefix='radar 中文 ') as folder:
            with contextlib.chdir(folder):
                main._start_radar_sync()
        self.factory.assert_called_once_with(main.PROJECT_ROOT)
        self.assertTrue(main.PROJECT_ROOT.is_absolute())

if __name__ == '__main__':
    unittest.main()
