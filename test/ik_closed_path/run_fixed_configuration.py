"""Sequential, offline IK comparison. Preserve each case and fail on any incomplete run."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    repository = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary-dir", type=Path,
                        default=repository / "build/ik_closed_path_comparison/bin")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)

    cases = []
    for repeat in range(3):
        for backend in ("newton", "anchored", "speed"):
            cases.append(("pin", backend, "ellipse", 100, 1000, "1e-5", repeat))
    cases.append(("trac", "newton", "ellipse", 100, 1000, "1e-5", 0))
    for backend in ("newton", "anchored"):
        cases.append(("pin", backend, "ellipse", 1000, 1000, "1e-5", 0))
        for shape in ("reverse", "line", "hold", "pose"):
            cases.append(("pin", backend, shape, 100, 1000, "1e-5", 0))
        for samples in (500, 2000):
            cases.append(("pin", backend, "ellipse", 100, samples, "1e-5", 0))
        cases.append(("pin", backend, "ellipse", 100, 1000, "1e-6", 0))

    outcomes = []
    for library, backend, shape, cycles, samples, epsilon, repeat in cases:
        name = f"{library}_{backend}_{shape}_c{cycles}_n{samples}_e{epsilon}_r{repeat}"
        prefix = (args.output / name).resolve()
        environment = os.environ.copy()
        environment.update(IK_DRIFT_BACKEND=backend, IK_DRIFT_SHAPE=shape,
                           IK_DRIFT_SEED="previous", IK_DRIFT_CYCLES=str(cycles),
                           IK_DRIFT_SAMPLES=str(samples), IK_DRIFT_EPS=epsilon,
                           IK_DRIFT_TIMEOUT_MS="1", IK_DRIFT_OUTPUT=str(prefix))
        start = time.monotonic()
        with prefix.with_suffix(".log").open("w") as log:
            run = subprocess.run(
                [str(args.binary_dir.resolve() / f"{library}_ik_closed_path_test"),
                 "--test-case=Standalone Talon closed path IK comparison"],
                env=environment, stdout=log, stderr=subprocess.STDOUT, check=False)
        outcome = {"name": name, "returncode": run.returncode,
                   "wall_seconds": time.monotonic() - start}
        if prefix.with_suffix(".json").exists():
            report = json.loads(prefix.with_suffix(".json").read_text())
            outcome.update({key: report[key] for key in
                            ("completed", "calls", "failed_step", "failed_reason",
                             "max_cycle_drift_rad", "solve_us", "over_budget")})
        outcomes.append(outcome)
        (args.output / "runs.json").write_text(json.dumps(outcomes, indent=2))
        print(json.dumps(outcome), flush=True)
    return int(any(outcome["returncode"] != 0 for outcome in outcomes))


if __name__ == "__main__":
    raise SystemExit(main())
