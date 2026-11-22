"""Check the Git index as well as the visible working tree before publishing."""

import subprocess
import struct
import zlib

import pytest

from tools import check_open_source_hygiene as hygiene


@pytest.fixture
def repository(tmp_path, monkeypatch):
    subprocess.run(["git", "init", "--quiet", str(tmp_path)], check=True)
    monkeypatch.setattr(hygiene, "ROOT", tmp_path)
    monkeypatch.setattr(hygiene, "ALLOWED_PACKAGES", set())
    return tmp_path


def stage(repository, relative):
    subprocess.run(["git", "add", "--", relative], cwd=repository, check=True)


def test_staged_scan_detects_a_secret_removed_only_from_working_tree(repository, capsys):
    path = repository / "settings.py"
    path.write_text('TOKEN = "' + "gh" + "p_" + "a" * 36 + '"\n')
    stage(repository, "settings.py")
    path.write_text('TOKEN = ""\n')

    assert hygiene.main([]) == 0
    assert hygiene.main(["--staged"]) == 1
    assert "possible credential/private key" in capsys.readouterr().out


def test_staged_scan_checks_files_deleted_only_from_working_tree(repository, capsys):
    path = repository / ".env"
    path.write_text("EXAMPLE=value\n")
    stage(repository, ".env")
    path.unlink()

    assert hygiene.main(["--staged"]) == 1
    assert "forbidden runtime/credential file" in capsys.readouterr().out


def test_staged_scan_detects_symlink_replaced_in_working_tree(repository, capsys):
    path = repository / "config.yaml"
    path.symlink_to("local.yaml")
    stage(repository, "config.yaml")
    path.unlink()
    path.write_text("enabled: false\n")

    assert hygiene.main([]) == 0
    assert hygiene.main(["--staged"]) == 1
    assert "symbolic links are not publishable" in capsys.readouterr().out


def test_staged_scan_rejects_forced_generated_files(repository, capsys):
    generated = repository / "build" / "generated.txt"
    generated.parent.mkdir()
    generated.write_text("generated\n")
    stage(repository, "build/generated.txt")

    assert hygiene.main(["--staged"]) == 1
    assert "forbidden runtime/credential file" in capsys.readouterr().out


def test_scan_detects_private_paths_and_ipv6_without_echoing_sensitive_values(repository):
    samples = ['/' + 'home/' + 'private-user/calibration.yaml',
               chr(92).join(['C:', 'Users', 'private-user', 'camera.yaml']),
               ':'.join(['2001', 'db8', '1234', '0', '0', '0', '0', '1'])]
    for value in samples:
        failures = []
        hygiene.scan_text(repository / 'settings.txt', 'settings.txt', failures, value.encode())
        assert len(failures) == 1
        assert value not in failures[0]


def test_scan_finds_credentials_in_compressed_png_metadata(repository):
    secret = ('gh' + 'p_' + 'x' * 36).encode()
    chunk = b'Comment\0\0' + zlib.compress(secret)
    data = b'\x89PNG\r\n\x1a\n' + struct.pack('>I4s', len(chunk), b'zTXt') + chunk + b'\0' * 4
    failures = []
    hygiene.scan_text(repository / 'texture.png', 'texture.png', failures, data)
    assert any('credential' in failure for failure in failures)


def test_scan_rejects_image_exif_metadata(repository):
    data = b'\x89PNG\r\n\x1a\n' + struct.pack('>I4s', 4, b'eXIf') + b'EXIF' + b'\0' * 4
    failures = []
    hygiene.scan_text(repository / 'texture.png', 'texture.png', failures, data)
    assert any('EXIF' in failure for failure in failures)


def test_unpacked_source_checks_without_git_but_cannot_claim_staged_audit(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(hygiene, 'ROOT', tmp_path)
    monkeypatch.setattr(hygiene, 'ALLOWED_PACKAGES', set())
    (tmp_path / 'README.md').write_text('AeroMind Sim\n')
    assert hygiene.main([]) == 0
    assert hygiene.main(['--staged']) == 1
    assert 'requires a Git repository' in capsys.readouterr().out


def test_four_part_pinned_version_is_allowed_but_network_urls_still_fail(repository):
    version = '.'.join(['4', '11', '0', '86'])
    failures = []
    hygiene.scan_text(repository / 'requirements.txt', 'requirements.txt', failures,
                      f'opencv-contrib-python-headless=={version}\n'.encode())
    assert failures == []
    hygiene.scan_text(repository / 'requirements.txt', 'requirements.txt', failures,
                      f'package @ http://{version}/wheel\n'.encode())
    assert any('IPv4' in failure for failure in failures)
