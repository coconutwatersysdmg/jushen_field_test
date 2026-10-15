from __future__ import annotations

import struct
import sys
import tempfile
import threading
import unittest
from pathlib import Path

from services import itl_converter


PCD_HEADER = (
    b"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
    b"TYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\n"
)
VALID_PCD = PCD_HEADER + b"DATA ascii\n1 2 3\n4 5 6\n"


class ItlConverterTests(unittest.TestCase):
    def setUp(self):
        self.folder = Path(self.enterContext(tempfile.TemporaryDirectory(prefix="ITL 中文 ")))
        self.source = self.folder / "input sample.itl"
        self.source.write_bytes(b"vendor test input")
        self.output = self.folder / "published" / "result sample.pcd"

    def command(self, body: str):
        script = self.folder / "fake_converter.py"
        script.write_text(
            "import pathlib, sys, time\nsource = pathlib.Path(sys.argv[1])\n"
            "output = pathlib.Path(sys.argv[2])\n" + body,
            encoding="utf-8",
        )
        return [sys.executable, str(script), "{input}", "{output}"]

    def test_publishes_valid_output_from_real_command_with_unicode_and_spaces(self):
        command = self.command(
            "assert source.read_bytes() == b'vendor test input'\n"
            f"output.write_bytes({VALID_PCD!r})\n"
        )
        result = itl_converter.convert_itl(self.source, self.output, command)
        self.assertEqual(result, self.output.resolve())
        self.assertEqual(self.output.read_bytes(), VALID_PCD)
        self.assertEqual(list(self.output.parent.iterdir()), [self.output])

    def test_missing_command_is_unavailable_and_does_not_publish(self):
        with self.assertRaises(itl_converter.ConverterUnavailable):
            itl_converter.convert_itl(self.source, self.output, [])
        self.assertFalse(self.output.exists())

    def test_existing_destination_is_preserved(self):
        self.output.parent.mkdir()
        self.output.write_bytes(b"original")
        with self.assertRaises(itl_converter.ConversionError):
            itl_converter.convert_itl(self.source, self.output, self.command("raise AssertionError\n"))
        self.assertEqual(self.output.read_bytes(), b"original")

    def test_destination_created_during_conversion_is_not_overwritten(self):
        command = self.command(
            "pathlib.Path(sys.argv[3]).write_bytes(b'concurrent result')\n"
            f"output.write_bytes({VALID_PCD!r})\n"
        ) + [str(self.output)]
        with self.assertRaises(itl_converter.ConversionError):
            itl_converter.convert_itl(self.source, self.output, command)
        self.assertEqual(self.output.read_bytes(), b"concurrent result")

    def test_success_exit_with_no_output_is_failure(self):
        with self.assertRaises(itl_converter.ConversionError):
            itl_converter.convert_itl(self.source, self.output, self.command("pass\n"))
        self.assertFalse(self.output.exists())

    def test_nonzero_exit_includes_diagnostic_and_never_publishes(self):
        with self.assertRaisesRegex(itl_converter.ConversionError, "vendor failure"):
            itl_converter.convert_itl(
                self.source, self.output,
                self.command(f"output.write_bytes({VALID_PCD!r})\nprint('vendor failure', file=sys.stderr)\nsys.exit(7)\n"),
            )
        self.assertFalse(self.output.exists())

    def test_truncated_pcd_is_rejected_and_staging_is_cleaned(self):
        truncated = PCD_HEADER + b"DATA ascii\n1 2 3\n"
        with self.assertRaises(itl_converter.ConversionError):
            itl_converter.convert_itl(
                self.source, self.output,
                self.command(f"output.write_bytes({truncated!r})\n"),
            )
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.output.parent.iterdir()), [])

    def test_timeout_stops_process_before_delayed_output(self):
        with self.assertRaisesRegex(itl_converter.ConversionError, "超时"):
            itl_converter.convert_itl(
                self.source, self.output,
                self.command(f"time.sleep(10)\noutput.write_bytes({VALID_PCD!r})\n"), timeout_s=0.1,
            )
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.output.parent.iterdir()), [])

    def test_shutdown_cancels_running_conversion(self):
        event = threading.Event()
        timer = threading.Timer(0.1, event.set)
        timer.start()
        try:
            with self.assertRaises(itl_converter.ConversionCancelled):
                itl_converter.convert_itl(
                    self.source, self.output,
                    self.command(f"time.sleep(10)\noutput.write_bytes({VALID_PCD!r})\n"), cancel_event=event,
                )
        finally:
            timer.join()
        self.assertFalse(self.output.exists())

    def test_templates_must_provide_input_and_output(self):
        for command in ([sys.executable, "{input}"], [sys.executable, "{output}"], [sys.executable, "{input}", "{output}", "{bad}"]):
            with self.subTest(command=command), self.assertRaises(itl_converter.ConversionError):
                itl_converter.convert_itl(self.source, self.output, command)
        self.assertFalse(self.output.exists())

    def test_missing_executable_is_actionable_failure(self):
        with self.assertRaises(itl_converter.ConversionError):
            itl_converter.convert_itl(self.source, self.output, [str(self.folder / "absent.exe"), "{input}", "{output}"])
        self.assertFalse(self.output.exists())

    def test_vendor_cli_reports_runtime_problem_from_executable_working_directory(self):
        helper = Path(__file__).resolve().parents[1] / "services" / "vendor_itl_cli.py"
        runtime = self.folder / "absent-runtime"
        command = [sys.executable, str(helper), "--input", "{input}", "--output", "{output}", "--runtime-dir", str(runtime)]
        with self.assertRaises(itl_converter.ConversionError) as failure:
            itl_converter.convert_itl(self.source, self.output, command)
        self.assertIn(str(runtime), str(failure.exception))
        self.assertFalse(self.output.exists())

    def test_valid_binary_pcd_is_accepted(self):
        self.output.parent.mkdir()
        self.output.write_bytes(PCD_HEADER + b"DATA binary\n" + struct.pack("<6f", 1, 2, 3, 4, 5, 6))
        itl_converter.validate_pcd(self.output)

    def test_valid_lzf_compressed_pcd_is_accepted(self):
        self.output.parent.mkdir()
        # One LZF literal run of 24 bytes; PCD stores field data separately.
        payload = b"\x17" + struct.pack("<6f", 1, 4, 2, 5, 3, 6)
        self.output.write_bytes(PCD_HEADER + b"DATA binary_compressed\n" + struct.pack("<II", 25, 24) + payload)
        itl_converter.validate_pcd(self.output)

    def test_matching_compressed_sizes_with_invalid_lzf_are_rejected(self):
        self.output.parent.mkdir()
        self.output.write_bytes(PCD_HEADER + b"DATA binary_compressed\n" + struct.pack("<II", 2, 24) + b"\xff\xff")
        with self.assertRaises(itl_converter.ConversionError):
            itl_converter.validate_pcd(self.output)

    def test_invalid_headers_and_payloads_are_rejected(self):
        self.output.parent.mkdir()
        invalid = [
            b"not PCD", VALID_PCD.replace(b"FIELDS x y z", b"FIELDS x y q"),
            VALID_PCD.replace(b"POINTS 2", b"POINTS 3"),
            VALID_PCD.replace(b"4 5 6", b"4 5 nan"),
            PCD_HEADER + b"DATA binary\n" + b"\0" * 23,
            PCD_HEADER + b"DATA binary_compressed\n" + struct.pack("<II", 2, 24) + b"x",
        ]
        for payload in invalid:
            with self.subTest(payload=payload):
                self.output.write_bytes(payload)
                with self.assertRaises(itl_converter.ConversionError):
                    itl_converter.validate_pcd(self.output)


class VendorItlCliTests(unittest.TestCase):
    def test_missing_runtime_cannot_launch_or_create_output(self):
        from services.vendor_itl_cli import convert_vendor_itl
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / "input.itl"
            source.write_bytes(b"input")
            output = root / "output.pcd"
            with self.assertRaises(itl_converter.ConverterUnavailable):
                convert_vendor_itl(source, output, root / "absent")
            self.assertFalse(output.exists())

    def test_existing_output_is_preserved_before_gui_launch(self):
        from services.vendor_itl_cli import convert_vendor_itl
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / "input.itl"
            source.write_bytes(b"input")
            output = root / "output.pcd"
            output.write_bytes(b"keep")
            with self.assertRaises(itl_converter.ConversionError):
                convert_vendor_itl(source, output, root / "runtime")
            self.assertEqual(output.read_bytes(), b"keep")


if __name__ == "__main__":
    unittest.main()
