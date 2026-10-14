import tempfile
import unittest
from pathlib import Path

from services.plc_coordinate_reader import (
    default_config, validate_config, save_config, load_config, CoordinateReader,
)


class MemoryPLC:
    """PLC boundary substitute: no physical PLC is available in these tests."""
    def __init__(self, memory):
        self.memory = memory

    def db_read(self, db, start, size):
        return self.memory[db][start:start + size]


class CoordinateReaderTests(unittest.TestCase):
    def configured(self):
        cfg = default_config()
        for axis in cfg['axes'].values():
            axis['enabled'] = False
        cfg['axes']['X'] = {'enabled': True, 'db': 20, 'offset': 100, 'type': 'REAL'}
        cfg['axes']['Y'] = {'enabled': True, 'db': 21, 'offset': 8, 'type': 'LREAL'}
        return cfg

    def test_missing_address_is_rejected_before_any_plc_access(self):
        with self.assertRaisesRegex(ValueError, 'X.*DB'):
            validate_config(default_config(), require_addresses=True)

    def test_partial_addresses_can_be_saved_without_becoming_readable(self):
        cfg = default_config()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'plc.json'
            save_config(path, cfg)
            self.assertEqual(load_config(path), cfg)
            with self.assertRaises(ValueError):
                CoordinateReader(load_config(path), MemoryPLC({}))

    def test_real_lreal_different_dbs_and_nonzero_offsets_decode_correctly(self):
        plc = MemoryPLC({20: bytes(100) + bytes.fromhex('c1480000'),
                         21: bytes(8) + bytes.fromhex('408f420000000000')})
        values = CoordinateReader(self.configured(), plc).read_once()
        self.assertEqual(values, {'X': -12.5, 'Y': 1000.25})

    def test_invalid_addresses_types_and_poll_intervals_are_rejected(self):
        for key, value in [('db', 0), ('offset', -1), ('type', 'INT'), ('db', True)]:
            cfg = self.configured()
            cfg['axes']['X'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                validate_config(cfg, require_addresses=True)
        cfg = self.configured()
        cfg['interval_ms'] = 0
        with self.assertRaises(ValueError):
            validate_config(cfg, require_addresses=True)

    def test_short_read_and_nonfinite_values_do_not_produce_coordinates(self):
        cfg = self.configured()
        for payload in [bytes.fromhex('c148'), bytes.fromhex('7fc00000')]:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                CoordinateReader(cfg, MemoryPLC({20: bytes(100) + payload,
                                                21: bytes(8) + bytes(8)})).read_once()

    def test_no_enabled_axes_is_rejected(self):
        cfg = self.configured()
        for item in cfg['axes'].values():
            item['enabled'] = False
        with self.assertRaises(ValueError):
            validate_config(cfg, require_addresses=True)

    def test_malformed_saved_config_is_reported_instead_of_silently_replaced(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'plc.json'
            path.write_text('{bad', encoding='utf-8')
            with self.assertRaises(ValueError):
                load_config(path)

    def test_missing_address_keys_are_rejected_as_malformed_config(self):
        cfg = default_config()
        del cfg['axes']['X']['db']
        with self.assertRaises(ValueError):
            validate_config(cfg)

    def test_db18_signed_dint_encoder_values(self):
        cfg = default_config()
        for name, offset in zip(('X', 'Y', 'Z', 'R'), (8, 12, 16, 20)):
            cfg['axes'][name] = {'enabled': True, 'db': 18, 'offset': offset, 'type': 'DINT'}
        memory = bytes(8) + bytes.fromhex('00000c80 ffffffce 000003b6 00000442')
        values = CoordinateReader(cfg, MemoryPLC({18: memory})).read_once()
        self.assertEqual(values, {'X': 3200, 'Y': -50, 'Z': 950, 'R': 1090})
        self.assertIsInstance(values['X'], int)

    def test_db112_truck_dimensions_and_all_eight_vertices(self):
        import struct
        cfg = self.configured()
        cfg['truck'] = {'enabled': True, 'db': 112, 'offset': 396}
        memory = bytes(396) + struct.pack('>34i', *range(-10, 24)) + bytes(16) + bytes.fromhex('0001 0002')
        values = CoordinateReader(cfg, MemoryPLC({20: bytes(100) + bytes.fromhex('c1480000'),
                                                21: bytes(8) + bytes(8), 112: memory})).read_once()
        self.assertEqual(values['truck']['length'], -10)
        self.assertEqual(values['truck']['front_height'], -4)
        self.assertEqual(values['truck']['rear_height'], -1)
        self.assertEqual(values['truck']['p1_x'], 0)
        self.assertEqual(values['truck']['p8_z'], 23)
        self.assertEqual(values['truck']['dunnage'], 1)
        self.assertEqual(values['truck']['front_baffle'], 2)

    def test_compensation_reads_only_confirmed_offsets_and_preserves_signed_values(self):
        cfg = self.configured()
        cfg['compensation'] = {'enabled': True, 'db': 102,
                               'applied': {'d_AxE1': 20}, 'input': {'d_AxE1': 24, 'd_Ay': None}}
        plc = MemoryPLC({20: bytes(100) + bytes.fromhex('c1480000'),
                         21: bytes(8) + bytes(8),
                         102: bytes(20) + bytes.fromhex('ffffff38 000000c8')})
        values = CoordinateReader(cfg, plc).read_once()
        self.assertEqual(values['compensation'], {'applied': {'d_AxE1': -200}, 'input': {'d_AxE1': 200}})


if __name__ == '__main__':
    unittest.main()
