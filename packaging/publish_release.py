#!/usr/bin/env python3
"""Publish verified build output to the current repository's tagged release."""
import argparse
import json
import os
from pathlib import Path
from deb_common import checksum, run

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('directory', type=Path)
args = parser.parse_args()
directory = args.directory.resolve()
data = json.loads((directory / 'release.json').read_text())
tag = os.environ['GITHUB_REF_NAME']
repo = os.environ['GITHUB_REPOSITORY']
if os.environ.get('GITHUB_REF_TYPE') != 'tag' or tag != 'v' + data['version']:
    raise ValueError('Release tag must match the packaged source version')
for asset in data['assets']:
    path = directory / asset['name']
    if path.parent != directory or checksum(path) != asset['sha256']:
        raise ValueError('Package checksum mismatch')
files = [str(directory / a['name']) for a in data['assets']]
files += [str(directory / name) for name in ('release.json', 'SHA256SUMS')]
existing = json.loads(run('gh', 'api', '--paginate', '--slurp', f'repos/{repo}/releases?per_page=100'))
release = next((r for page in existing for r in page if r['tag_name'] == tag), None)
if release and not release['draft']:
    raise ValueError('Release already published; never overwrite it. Use a new version/tag.')
notes = directory / 'release-notes.md'
notes.write_text(f'ROCOS {data["component"]} {data["version"]}\n\n'
                 'DEB packaging test release. Native binaries target Ubuntu 22.04; '
                 'Viz contains architecture-independent static files.\n\n'
                 'This release has not been validated on robot hardware. Installation does not start services.\n\n'
                 'Verify downloads with `sha256sum -c SHA256SUMS`. '
                 'See the repository README for installation paths and dependencies.\n')
if release is None:
    command = ['gh', 'release', 'create', tag, *files, '--repo', repo, '--verify-tag',
               '--draft', '--title', tag, '--notes-file', str(notes)]
    if '-' in data['version']:
        command.append('--prerelease')
    run(*command)
else:
    if release['prerelease'] != ('-' in data['version']):
        raise ValueError('Existing draft prerelease flag differs from tag')
    run('gh', 'release', 'upload', tag, *files, '--repo', repo, '--clobber')
listed = json.loads(run('gh', 'api', '--paginate', '--slurp', f'repos/{repo}/releases?per_page=100'))
release_id = next(r['id'] for page in listed for r in page if r['tag_name'] == tag)
remote = json.loads(run('gh', 'api', f'repos/{repo}/releases/{release_id}'))
assets = {a['name']: a for a in remote['assets']}
for file in files:
    path = Path(file)
    asset = assets[path.name]
    if asset['state'] != 'uploaded' or asset['size'] != path.stat().st_size:
        raise ValueError('Incomplete release upload')
    if asset.get('digest') != 'sha256:' + checksum(path):
        raise ValueError('GitHub upload digest mismatch or missing digest')
run('gh', 'release', 'edit', tag, '--repo', repo, '--draft=false',
    '--latest=false' if '-' in data['version'] else '--latest=true')
