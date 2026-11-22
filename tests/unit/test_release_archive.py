"""Export exactly the audited source index without local data or host metadata."""

import subprocess
import zipfile

import pytest

from tools.package_release import write_archive


def test_archive_uses_staged_bytes_and_excludes_git_and_untracked_private_files(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    subprocess.run(['git', 'init', '--quiet', str(source)], check=True)
    (source / 'README.md').write_text('AeroMind Sim\n')
    subprocess.run(['git', 'add', 'README.md'], cwd=source, check=True)
    (source / 'README.md').write_text('Unstaged draft\n')
    (source / 'private.local.yaml').write_text('not for publishing\n')
    first, second = tmp_path / 'first.zip', tmp_path / 'second.zip'
    write_archive(source, first)
    write_archive(source, second)
    assert first.read_bytes() == second.read_bytes()
    with zipfile.ZipFile(first) as archive:
        assert archive.namelist() == ['aeromind-sim/README.md']
        assert archive.read('aeromind-sim/README.md') == b'AeroMind Sim\n'
        info = archive.infolist()[0]
        assert info.date_time == (2000, 1, 1, 0, 0, 0)
        assert info.extra == b'' and info.comment == b''


def test_archive_cannot_be_generated_inside_public_repository(tmp_path):
    with pytest.raises(ValueError, match='outside'):
        write_archive(tmp_path, tmp_path / 'release.zip')
