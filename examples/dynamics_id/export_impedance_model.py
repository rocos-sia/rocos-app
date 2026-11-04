"""Export Studio's P @ beta once, so the C++ controller needs no NPZ parser."""
import argparse
import json
from pathlib import Path

import numpy as np


def export(directory, output):
    result = json.loads((directory / "identification.json").read_text())
    if not result["converged"]:
        raise ValueError("Identification has not converged")
    names = result["joint_names"]
    beta = np.asarray(result["base_parameters"], dtype=float)
    with np.load(directory / "identification.npz", allow_pickle=False) as archive:
        P, stored_beta = archive["P"], archive["base"]
    if (not names or len(set(names)) != len(names) or beta.ndim != 1 or
            not 0 < beta.size <= 10 * len(names) or result["base_parameter_count"] != beta.size or
            P.shape != (10 * len(names), beta.size) or stored_beta.shape != beta.shape or
            not np.isfinite(P).all() or not np.isfinite(beta).all() or
            not np.allclose(stored_beta, beta, rtol=1e-12, atol=0) or
            not np.allclose(P.T @ P, np.eye(beta.size), rtol=1e-8, atol=1e-8)):
        raise ValueError("Invalid or mismatched Studio P/beta/joint names")
    model = dict(schema="rocos_identified_rigid_v1", urdf_hash=result["urdf_hash"],
                 joint_names=names, rigid_parameters=(P @ beta).tolist())
    # Exclusive creation protects an existing exported model.
    with output.open("x") as stream:
        json.dump(model, stream, indent=2, allow_nan=False)
        stream.write("\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("identification_dir", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    export(args.identification_dir, args.output)
