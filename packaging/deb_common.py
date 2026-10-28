"""Small Debian packaging helpers. No installation or service activation."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


def run(*args, **kwargs):
    return subprocess.check_output(args, text=True, **kwargs).strip()


def copy_file(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def copy_tracked(root, stage, prefix, destination):
    for name in run('git', '-C', str(root), 'ls-files', '-z', prefix).split('\0'):
        if name and (root / name).is_file():
            relative = Path(name).relative_to(prefix)
            copy_file(root / name, stage / destination / relative)


def stage_runtime(binary, build, stage, prefix):
    environment = dict(os.environ, LD_LIBRARY_PATH=str(build / 'lib'))
    listing = run('ldd', str(binary), env=environment)
    if 'not found' in listing:
        raise ValueError('Unresolved library: ' + listing)
    executable = stage / prefix / 'bin' / binary.name
    copy_file(binary, executable)
    run('patchelf', '--set-rpath', '$ORIGIN/../lib', str(executable))
    elf_files = [executable]
    for line in listing.splitlines():
        if ' => /' not in line:
            continue
        name, rest = line.strip().split(' => ', 1)
        library = Path(rest.split(' (', 1)[0]).resolve()
        if library.is_relative_to(build):
            target = stage / prefix / 'lib' / name
            copy_file(library, target)
            run('patchelf', '--set-rpath', '$ORIGIN', str(target))
            elf_files.append(target)
        elif not (library.is_relative_to('/usr/lib') or library.is_relative_to('/lib')):
            raise ValueError(f'Non-system dependency {library}; use a clean distro build')
    return elf_files


def checksum(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def finish(stage, output, component, version, arch, compatibility, description,
           elf_files=(), private_lib=None, config_dirs=('opt/rocos/config',)):
    stage.chmod(0o755)
    package = 'rocos-' + component
    deb_version = version.replace('-', '~', 1)
    control = (f'Package: {package}\nVersion: {deb_version}\nArchitecture: {arch}\n'
               'Maintainer: ROCOS maintainers <noreply@github.com>\nSection: misc\nPriority: optional\n'
               f'Homepage: https://github.com/rocos-sia/rocos-releases\nDescription: {description}\n')
    metadata = stage / 'DEBIAN'
    metadata.mkdir()
    if elf_files:
        # dpkg-shlibdeps needs a source control file, even for an unpacked binary.
        debian = stage / 'debian'
        debian.mkdir()
        (debian / 'control').write_text(f'Source: {package}\n\n' + control)
        environment = dict(os.environ)
        environment.pop('LD_LIBRARY_PATH', None)
        dependencies = run('dpkg-shlibdeps', '-O', '--ignore-missing-info',
                           '-l' + str(private_lib),
                           *['-e' + str(p) for p in elf_files], cwd=stage, env=environment)
        shutil.rmtree(debian)
        depends = dependencies.removeprefix('shlibs:Depends=')
        if not depends or depends == dependencies:
            raise ValueError('Unable to determine runtime dependencies')
        control += 'Depends: ' + depends + '\n'
    (metadata / 'control').write_text(control)
    configs = sorted('/' + str(p.relative_to(stage)) for directory in config_dirs
                     for p in (stage / directory).rglob('*')
                     if p.is_file() and p.suffix in ('.yaml', '.yml', '.xml', '.urdf'))
    if configs:
        (metadata / 'conffiles').write_text('\n'.join(configs) + '\n')
    output.mkdir(parents=True, exist_ok=True)
    # GitHub normalizes '~' in asset names; keep Debian ordering inside control only.
    deb = output / f'{package}_{version}_{arch}.deb'
    run('dpkg-deb', '--root-owner-group', '--build', str(stage), str(deb))
    release = {'schema_version': 1, 'component': component, 'version': version,
               'compatibility': compatibility, 'assets': [
                   {'name': deb.name, 'os': 'linux', 'arch': arch, 'format': 'deb',
                    'size_bytes': deb.stat().st_size, 'sha256': checksum(deb)}]}
    manifest = output / 'release.json'
    manifest.write_text(json.dumps(release, indent=2) + '\n')
    (output / 'SHA256SUMS').write_text(''.join(f'{checksum(p)}  {p.name}\n' for p in (deb, manifest)))
    print(deb)


def licenses(root, stage, component):
    dest = stage / 'usr/share/doc' / ('rocos-' + component)
    copy_file(root / 'README.md', dest / 'README.md')
    for name in run('git', '-C', str(root), 'ls-files', '-z').split('\0'):
        if name and Path(name).name.lower().startswith(('license', 'copying', 'notice')):
            copy_file(root / name, dest / 'licenses' / name)
