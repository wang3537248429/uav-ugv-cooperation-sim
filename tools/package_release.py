#!/usr/bin/env python3
"""Export audited staged source files, without Git metadata or local output."""

import argparse
from pathlib import Path
import subprocess
import zipfile

try:
    from . import check_open_source_hygiene as hygiene
except ImportError:
    import check_open_source_hygiene as hygiene


def write_archive(root, output):
    root, output = Path(root).resolve(), Path(output).resolve()
    if output.is_relative_to(root):
        raise ValueError('Release archives must be written outside the public source repository')
    entries = subprocess.check_output(['git', 'ls-files', '--stage', '-z'], cwd=root)
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for item in sorted(entries.split(b'\0')):
            if not item:
                continue
            metadata, filename = item.decode().split('\t', 1)
            mode, blob, stage = metadata.split()
            path = Path(filename)
            if stage != '0' or mode not in ('100644', '100755') or path.is_absolute() or '..' in path.parts:
                raise ValueError('Only regular staged source files can enter the release')
            data = subprocess.check_output(['git', 'cat-file', 'blob', blob], cwd=root)
            info = zipfile.ZipInfo('aeromind-sim/' + filename, date_time=(2000, 1, 1, 0, 0, 0))
            info.create_system = 3
            info.external_attr = (0o100755 if mode == '100755' else 0o100644) << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(info, data)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if hygiene.main(['--staged']):
        return 1
    output = write_archive(hygiene.ROOT, args.output)
    print(f'Audited simulation source archive: {output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
