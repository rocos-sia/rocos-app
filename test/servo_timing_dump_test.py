import importlib.util
import json
import struct
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location('timing', Path(__file__).resolve().parents[1] / 'examples/servo_timing_dump.py')
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)


class ExportTest(unittest.TestCase):
    def test_wrapped_rings_are_exported_in_chronological_order(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / 'capture.bin'
            header = bytearray(256)
            header[:8] = b'RCSTIME1'
            struct.pack_into('<6Q', header, 8, 1, 4, 64, 80, 10, 20)
            header[56:59] = b'abc'
            struct.pack_into('<Q', header, 128, 6)
            struct.pack_into('<Q', header, 192, 1)
            udp = bytearray(4 * 64)
            for count in range(6):
                struct.pack_into('<8Q', udp, (count % 4) * 64, count + 1, 1, 2, 3, 4, 1, 0, 0)
            control = struct.pack('<8QqQ', 1, 1, 2, 3, 6, 6, 5, 4, -1008, 0) + bytes(3 * 80)
            source.write_bytes(header + udp + control)
            meta = timing.export(source, root / 'out')
            self.assertEqual(meta['udp_overwritten'], 2)
            self.assertEqual([line.split(',')[0] for line in (root / 'out/udp.csv').read_text().splitlines()[1:]], ['3', '4', '5', '6'])
            self.assertIn('-1008', (root / 'out/control.csv').read_text())
            self.assertEqual(json.loads((root / 'out/metadata.json').read_text())['session_id'], 'abc')
            with self.assertRaises(FileExistsError):
                timing.export(source, root / 'out')

    def test_bad_or_truncated_capture_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / 'capture.bin'
            source.write_bytes(bytes(256))
            with self.assertRaises(ValueError):
                timing.export(source, Path(temp) / 'out')


if __name__ == '__main__':
    unittest.main()
