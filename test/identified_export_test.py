"""Offline checks for exporting Studio basis parameters without rewriting URDF."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "identified_export", ROOT / "examples/dynamics_id/export_impedance_model.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ExportTest(unittest.TestCase):
    def test_export_and_reject_inconsistent_inputs(self):
        with tempfile.TemporaryDirectory(prefix="rocos-export-") as temporary:
            directory = Path(temporary)
            beta = np.array([1.2, -0.4, 0.8])
            basis = np.eye(20)[:, :3]
            metadata = dict(
                converged=True, joint_names=["joint_1", "joint_2"],
                base_parameters=beta.tolist(), base_parameter_count=3, urdf_hash="fixture-hash",
            )
            result = directory / "identification.json"
            archive = directory / "identification.npz"
            output = directory / "rigid.json"
            result.write_text(json.dumps(metadata))
            np.savez_compressed(archive, P=basis, base=beta)
            module.export(directory, output)
            exported = json.loads(output.read_text())
            np.testing.assert_array_equal(exported["rigid_parameters"], basis @ beta)
            self.assertEqual(exported["joint_names"], metadata["joint_names"])
            with self.assertRaises(FileExistsError):
                module.export(directory, output)
            invalid = directory / "invalid.json"
            for matrix, stored in ((basis, beta + 1), (basis * 2, beta), (basis[:10], beta)):
                np.savez_compressed(archive, P=matrix, base=stored)
                with self.assertRaises(ValueError):
                    module.export(directory, invalid)
                self.assertFalse(invalid.exists())
            np.savez_compressed(archive, P=basis, base=beta)
            metadata["converged"] = False
            result.write_text(json.dumps(metadata))
            with self.assertRaises(ValueError):
                module.export(directory, invalid)


if __name__ == "__main__":
    unittest.main()
