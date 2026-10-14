import os
import tempfile
import time
import unittest
from pathlib import Path
from threading import Event

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication
from ui.plc_coordinate_dialog import PlcCoordinateDialog


class FakePLC:
    def connect(self, *args, **kwargs):
        return self

    def db_read(self, db, start, size):
        return bytes.fromhex('41480000')

    def disconnect(self):
        pass


class BrokenPLC(FakePLC):
    def db_read(self, db, start, size):
        raise RuntimeError('CPU refused')


class CoordinateDialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.path = Path(self.folder.name) / 'plc.json'
        self.dialog = PlcCoordinateDialog(config_path=self.path, client_factory=FakePLC)

    def drain_until(self, condition):
        end = time.monotonic() + 3
        while not condition() and time.monotonic() < end:
            self.app.processEvents()
            time.sleep(.01)
        self.assertTrue(condition())

    def tearDown(self):
        self.dialog.stop_reading()
        self.drain_until(lambda: not self.dialog.is_running())
        self.dialog.close()
        self.dialog.deleteLater()
        self.app.processEvents()
        self.folder.cleanup()

    def configure_x(self):
        for name, fields in self.dialog.axis_fields.items():
            fields['enabled'].setChecked(name == 'X')
        self.dialog.axis_fields['X']['db'].setText('20')
        self.dialog.axis_fields['X']['offset'].setText('100')

    def test_unconfigured_start_does_not_launch_worker(self):
        self.dialog.start_reading()
        self.assertFalse(self.dialog.is_running())
        self.assertIn('待配置', self.dialog.status_label.text())
        self.assertEqual(self.dialog.axis_fields['X']['value'].text(), '—')

    def test_saved_config_restores_editable_addresses(self):
        self.configure_x()
        self.dialog.save_settings()
        other = PlcCoordinateDialog(config_path=self.path, client_factory=FakePLC)
        self.assertEqual(other.axis_fields['X']['offset'].text(), '100')
        self.assertEqual(other.axis_fields['X']['db'].text(), '20')
        other.close()

    def test_background_poll_updates_values_and_stop_invalidates_them(self):
        self.configure_x()
        self.dialog.start_reading()
        self.drain_until(lambda: self.dialog.axis_fields['X']['value'].text() == '12.500000')
        self.assertIn('读取中', self.dialog.status_label.text())
        self.dialog.stop_reading()
        self.drain_until(lambda: not self.dialog.is_running())
        self.assertEqual(self.dialog.axis_fields['X']['value'].text(), '—')

    def test_connection_probe_needs_no_db_addresses(self):
        self.dialog.test_connection()
        self.drain_until(lambda: not self.dialog.is_running())
        self.assertIn('S7 连接成功', self.dialog.status_label.text())
        self.assertEqual(self.dialog.axis_fields['X']['value'].text(), '—')

    def test_read_error_clears_values_and_reenables_configuration(self):
        self.configure_x()
        self.dialog.client_factory = BrokenPLC
        self.dialog.start_reading()
        self.drain_until(lambda: 'CPU refused' in self.dialog.status_label.text())
        self.assertIn('CPU refused', self.dialog.status_label.text())
        self.assertEqual(self.dialog.axis_fields['X']['value'].text(), '—')
        self.assertTrue(self.dialog.is_running())
        self.dialog.stop_reading()
        self.drain_until(lambda: not self.dialog.is_running())
        self.assertTrue(self.dialog.start_btn.isEnabled())

    def test_close_waits_for_inflight_read_without_blocking_ui(self):
        entered, release = Event(), Event()

        class SlowPLC(FakePLC):
            def db_read(self, db, start, size):
                entered.set()
                release.wait(2)
                return super().db_read(db, start, size)

        self.configure_x()
        self.dialog.client_factory = SlowPLC
        self.dialog.show()
        self.dialog.start_reading()
        try:
            self.drain_until(entered.is_set)
            self.dialog.close()
            self.assertTrue(self.dialog.is_running())
            self.assertTrue(self.dialog.isVisible())
        finally:
            release.set()
        self.drain_until(lambda: not self.dialog.is_running())
        self.assertFalse(self.dialog.isVisible())
        self.assertEqual(self.dialog.axis_fields['X']['value'].text(), '—')

    def test_escape_requests_worker_shutdown_instead_of_hiding_live_reader(self):
        self.configure_x()
        self.dialog.show()
        self.dialog.start_reading()
        self.drain_until(lambda: self.dialog.axis_fields['X']['value'].text() == '12.500000')
        self.dialog.reject()  # QDialog Escape path
        self.drain_until(lambda: not self.dialog.is_running())
        self.assertFalse(self.dialog.isVisible())

    def test_dint_option_and_exact_integer_display(self):
        self.assertGreaterEqual(self.dialog.axis_fields['X']['type'].findText('DINT'), 0)
        self.dialog._sample({'X': -2147483648, 'R': 1090}, 'test-time')
        self.assertEqual(self.dialog.axis_fields['X']['value'].text(), '-2147483648')
        self.assertEqual(self.dialog.axis_fields['R']['value'].text(), '1090')

    def test_open_automatically_reads_and_reopen_reads_again(self):
        self.configure_x()
        self.dialog.show()
        self.drain_until(lambda: self.dialog.axis_fields['X']['value'].text() == '12.500000')
        self.dialog.close()
        self.drain_until(lambda: not self.dialog.is_running())
        self.dialog.show()
        self.drain_until(lambda: self.dialog.axis_fields['X']['value'].text() == '12.500000')

    def test_dashboard_updates_vertices_and_clears_them_on_error(self):
        self.dialog._sample({'X': 3200, 'truck': {'length': 6000, 'p8_z': -50}}, 'now')
        self.assertEqual(self.dialog.truck_values['length'].text(), '6000')
        self.assertEqual(self.dialog.vertex_tables[1].item(2, 3).text(), '-50')
        self.dialog._status('连接失败')
        self.assertEqual(self.dialog.truck_values['length'].text(), '—')
        self.assertEqual(self.dialog.vertex_tables[1].item(2, 3).text(), '—')

    def test_failed_connection_automatically_recovers(self):
        self.configure_x()
        attempts = []

        class FailsOnce(FakePLC):
            def connect(self, *args, **kwargs):
                raise ConnectionError('temporarily offline')

        def factory():
            attempts.append(1)
            return FailsOnce() if len(attempts) == 1 else FakePLC()

        self.dialog.client_factory = factory
        self.dialog.show()
        self.drain_until(lambda: 'temporarily offline' in self.dialog.status_label.text())
        self.drain_until(lambda: self.dialog.axis_values['X'].text() == '12.500000')

    def test_baffle_encoding_and_unconfirmed_dunnage_stay_distinct(self):
        self.dialog._sample({'truck': {'dunnage': 1, 'front_baffle': 2}}, 'now')
        self.assertIn('1', self.dialog.truck_values['dunnage'].text())
        self.assertIn('待确认', self.dialog.truck_values['dunnage'].text())
        self.assertEqual(self.dialog.truck_values['front_baffle'].text(), '无（2）')
        self.dialog._sample({'truck': {'front_baffle': 1}}, 'now')
        self.assertEqual(self.dialog.truck_values['front_baffle'].text(), '有（1）')

    def test_compensation_tab_has_five_rows_and_does_not_invent_missing_values(self):
        self.assertEqual(self.dialog.data_tabs.currentIndex(), 0)
        self.assertEqual(self.dialog.data_tabs.count(), 2)
        self.dialog.data_tabs.setCurrentIndex(1)
        self.assertEqual(len(self.dialog.compensation_values), 5)
        self.dialog._sample({'compensation': {'applied': {'d_AxE1': -25}, 'input': {'d_AxE1': 10}}}, 'now')
        self.assertEqual(self.dialog.compensation_values['d_AxE1']['applied'].text(), '-25')
        self.assertEqual(self.dialog.compensation_values['d_AxE1']['input'].text(), '10')
        self.assertEqual(self.dialog.compensation_values['d_Ay']['input'].text(), '待配置')
        self.dialog._status('断开')
        self.assertNotEqual(self.dialog.compensation_values['d_AxE1']['applied'].text(), '-25')


if __name__ == '__main__':
    unittest.main()
