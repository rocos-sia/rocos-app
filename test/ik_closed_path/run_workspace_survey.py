"""Run sequential joint-range sampling and multi-posture closed-path comparisons."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--binary-dir", type=Path, default=root / "build/ik_closed_path_comparison/bin")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    methods = [("pin", "speed"), ("trac", "speed"), ("pin", "anchored"),
               ("pin", "distance"), ("trac", "distance")]
    runs = []
    for model in ("arm6", "talon"):
        for kind in ("global", "constrained", "warm", "loops"):
            selected = methods[:3] if kind == "loops" else methods
            for library, backend in selected:
                name = f"{model}_{kind}_{library}_{backend}"
                prefix = (args.output / name).resolve()
                environment = os.environ.copy()
                environment.update(IK_SURVEY_MODEL=model, IK_SURVEY_KIND=kind,
                                   IK_SURVEY_COUNT="64" if kind == "loops" else "2000",
                                   IK_DRIFT_BACKEND=backend, IK_DRIFT_OUTPUT=str(prefix))
                start = time.monotonic()
                with prefix.with_suffix(".log").open("w") as log:
                    result = subprocess.run([str(args.binary_dir.resolve() /
                                                 f"{library}_ik_workspace_survey")],
                                            env=environment, stdout=log,
                                            stderr=subprocess.STDOUT, check=False)
                row = {"name": name, "exit_code": result.returncode,
                       "wall_seconds": time.monotonic() - start}
                if prefix.with_suffix(".json").exists():
                    data = json.loads(prefix.with_suffix(".json").read_text())
                    row.update({key:data[key] for key in ("completed", "samples", "calls",
                        "pose_successes", "on_time_pose_successes", "initialization_failures", "solve_us")})
                runs.append(row)
                (args.output / "runs.json").write_text(json.dumps(runs, indent=2))
                print(json.dumps(row), flush=True)
    return int(any(row["exit_code"] != 0 for row in runs))


if __name__ == "__main__":
    raise SystemExit(main())
