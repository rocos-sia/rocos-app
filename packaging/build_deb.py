#!/usr/bin/env python3
"""Package a clean Ubuntu 22.04 build, including only tracked default configuration."""
import argparse
from pathlib import Path
import re
import tempfile
from deb_common import copy_tracked, finish, licenses, run, stage_runtime

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--build-dir', type=Path, default=root / 'build/deb-release')
parser.add_argument('--output', type=Path, default=root / 'build/packages')
args = parser.parse_args()
build, output = args.build_dir.resolve(), args.output.resolve()
cmake = (root / 'CMakeLists.txt').read_text()
version = re.search(r'project\(rocos_app VERSION (\d+\.\d+\.\d+)', cmake)[1]
suffix = re.search(r'set\(ROCOS_APP_VERSION_PRERELEASE "([^"]*)"\)', cmake)[1]
if suffix:
    version += '-' + suffix
if f'"{version}"' not in (build / 'ver.h').read_text():
    raise ValueError('Build version differs from source; rebuild before packaging')
arch = run('dpkg', '--print-architecture')
if arch not in ('amd64', 'arm64', 'armhf'):
    raise ValueError('Unsupported Debian architecture: ' + arch)
with tempfile.TemporaryDirectory(prefix='rocos-app-deb-') as tmp:
    stage = Path(tmp)
    elves = stage_runtime(build / 'bin/rocosAppMain', build, stage, 'opt/rocos/app')
    copy_tracked(root, stage, 'config', 'opt/rocos/config')
    (stage / 'opt/rocos/web').mkdir()  # Viz owns the contents; App validates this directory.
    licenses(root, stage, 'app')
    finish(stage, output, 'app', version, arch,
           {'api_version': '3.0.0', 'shm_abi_version': 2},
           'ROCOS robot controller (Ubuntu 22.04)', elves, stage / 'opt/rocos/app/lib')
