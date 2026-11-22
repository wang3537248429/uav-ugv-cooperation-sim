#!/usr/bin/env python3
"""Fail when publishable files contain local identity, network or secret data."""

from __future__ import annotations

import argparse
import ipaddress
from pathlib import Path
import re
import struct
import subprocess
import zlib


ROOT = Path(__file__).resolve().parents[1]
ALLOWED_PACKAGES = {
    "ap_gazebo_bridge",
    "ap_mavlink_bridge",
    "ap_mission",
    "ap_navigation",
    "ap_perception",
    "ap_simulation",
}
ALLOWED_EMAILS = {"contributors@aeromind-sim.invalid"}
VENDORED_PREFIXES = (
    "src/ap_simulation/models/x500/",
    "src/ap_simulation/models/x500_base/",
)
FORBIDDEN_NAMES = {
    ".env",
    "id_dsa",
    "id_ecdsa",
    "id_ed25519",
    "id_rsa",
    "authorized_keys",
}
FORBIDDEN_DIRECTORIES = {
    ".artifacts", ".venv", "__pycache__", "build", "install", "log", "venv",
    "private", "hardware", "backups", "evidence", "releases",
}
FORBIDDEN_SUFFIXES = {
    ".bag",
    ".bin",
    ".db3",
    ".jks",
    ".key",
    ".log",
    ".mcap",
    ".p12",
    ".pem",
    ".pfx",
    ".tlog",
    ".ulg",
    ".npz", ".pcd", ".ply", ".jpg", ".jpeg", ".mp4", ".avi", ".zip", ".tar", ".gz",
}

IPV4 = re.compile(r"(?<![\d.])(?:\d{1,3}\.){3}\d{1,3}(?![\d.])")
HOME_PATH = re.compile(
    r"(?:/home/[A-Za-z0-9._-]+|/Users/[A-Za-z0-9._-]+|/root)(?:/|$)"
)
DEVICE_PATH = re.compile(
    r"/dev/(?:tty(?:USB|ACM|AMA|S)[0-9]+|serial/by-id/[^\s\"']+)"
)
WINDOWS_HOME = re.compile(r"[A-Za-z]:[\\/]Users[\\/][^\s\\/\"']+")
IPV6 = re.compile(r"(?<![\w:])(?:[0-9a-fA-F]{0,4}:){2,}[0-9a-fA-F]{0,4}(?![\w:])")
EMAIL = re.compile(
    r"(?<![A-Za-z0-9._%+-])"
    r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}"
)
SECRET_PATTERNS = (
    re.compile(r"-----BEGIN (?:RSA |OPENSSH |EC |DSA )?PRIVATE KEY-----"),
    re.compile(r"\bAKIA[0-9A-Z]{16}\b"),
    re.compile(r"\bgithub_pat_[A-Za-z0-9_]{20,}\b"),
    re.compile(r"\bgh[pousr]_[A-Za-z0-9_]{20,}\b"),
    re.compile(r"\bxox[baprs]-[A-Za-z0-9-]{20,}\b"),
    re.compile(r"\bsk-[A-Za-z0-9]{20,}\b"),
    re.compile(
        r"(?i)\b(?:password|passwd|client[_-]?secret|api[_-]?key|"
        r"access[_-]?token)\s*[:=]\s*[\"']?"
        r"(?!<|your\b|replace\b|changeme\b|none\b|null\b|false\b)"
        r"[A-Za-z0-9/+_=-]{8,}"
    ),
)


def publishable_paths(staged: bool = False) -> list[Path]:
    if not (ROOT / '.git').exists():
        if staged:
            raise ValueError('Staged-content audit requires a Git repository')
        # GitHub/source ZIP downloads contain no .git directory. Audit their
        # actual source files without depending on a parent repository.
        return sorted(path for path in ROOT.rglob('*')
                      if (path.is_file() or path.is_symlink())
                      and '.git' not in path.relative_to(ROOT).parts)
    arguments = ["git", "ls-files", "--cached", "-z"]
    if not staged:
        arguments.extend(["--others", "--exclude-standard"])
    result = subprocess.run(
        arguments,
        cwd=ROOT,
        check=True,
        capture_output=True,
    )
    return [
        ROOT / item.decode("utf-8")
        for item in result.stdout.split(b"\0")
        if item
    ]


def scan_text(
    path: Path, relative: str, failures: list[str], data: bytes | None = None,
) -> None:
    if data is None:
        data = path.read_bytes()
    text = data.decode("utf-8", errors="replace")

    # Four-part wheel versions (notably OpenCV) can look like IP addresses.
    # Exclude only complete pinned requirements, while scanning URLs/comments.
    network_text = text
    if re.fullmatch(r'requirements(?:-[A-Za-z0-9_-]+)?\.txt', relative):
        network_text = re.sub(r'(?m)^[A-Za-z0-9][A-Za-z0-9_.-]*==[0-9][A-Za-z0-9_.+!-]*[ \t]*$', '', text)
    for match in IPV4.finditer(network_text):
        candidate = match.group(0)
        try:
            address = ipaddress.ip_address(candidate)
        except ValueError:
            continue
        if str(address) != "127.0.0.1":
            failures.append(f"{relative}: non-loopback IPv4 address")

    for match in HOME_PATH.finditer(text):
        failures.append(f"{relative}: absolute user home path")

    for match in WINDOWS_HOME.finditer(text):
        failures.append(f"{relative}: absolute Windows user home path")

    # Two colon bytes in compressed pixels/meshes are not an IPv6 endpoint.
    # PNG text chunks are decompressed and scanned independently below.
    for match in IPV6.finditer(text) if b'\0' not in data else ():
        try:
            address = ipaddress.IPv6Address(match.group())
        except ValueError:
            continue
        if address != ipaddress.IPv6Address('::1') and not address.is_unspecified:
            failures.append(f"{relative}: non-loopback IPv6 address")

    for match in DEVICE_PATH.finditer(text):
        failures.append(f"{relative}: local hardware device path {match.group(0)!r}")

    vendored = relative.startswith(VENDORED_PREFIXES)
    if not vendored:
        for match in EMAIL.finditer(text):
            email = match.group(0)
            if email not in ALLOWED_EMAILS:
                failures.append(f"{relative}: non-anonymous email")

    for pattern in SECRET_PATTERNS:
        if pattern.search(text):
            failures.append(f"{relative}: possible credential/private key")

    if data.startswith(b'\x89PNG\r\n\x1a\n'):
        scan_png_metadata(path, relative, failures, data)


def scan_png_metadata(path, relative, failures, data):
    """Also scan compressed PNG text; image pixel data alone is not an audit."""
    offset = 8
    try:
        while offset + 12 <= len(data):
            size, kind = struct.unpack('>I4s', data[offset:offset + 8])
            if offset + 12 + size > len(data):
                raise ValueError('truncated chunk')
            payload = data[offset + 8:offset + 8 + size]
            offset += size + 12
            if kind == b'eXIf':
                failures.append(f'{relative}: image EXIF metadata must be removed before publishing')
            elif kind == b'tEXt':
                scan_text(path, relative, failures, payload.replace(b'\0', b' '))
            elif kind == b'zTXt':
                keyword, encoded = payload.split(b'\0', 1)
                text = zlib.decompressobj().decompress(encoded[1:], 1024 * 1024)
                scan_text(path, relative, failures, keyword + b' ' + text)
            elif kind == b'iTXt':
                keyword, fields = payload.split(b'\0', 1)
                compressed, method = fields[:2]
                language, translated, text = fields[2:].split(b'\0', 2)
                if compressed == 1:
                    text = zlib.decompressobj().decompress(text, 1024 * 1024)
                scan_text(path, relative, failures, keyword + b' ' + translated + b' ' + text)
    except (ValueError, zlib.error, struct.error):
        failures.append(f'{relative}: malformed PNG metadata')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--staged", action="store_true",
        help="scan the exact Git index contents that the next commit will publish",
    )
    args = parser.parse_args(argv)
    failures: list[str] = []
    try:
        paths = publishable_paths(staged=args.staged)
    except ValueError as error:
        print(f'Open-source hygiene check failed: {error}')
        return 1
    index_modes = {}
    conflicted_paths = set()
    if args.staged:
        result = subprocess.run(
            ["git", "ls-files", "--stage", "-z"], cwd=ROOT,
            check=True, capture_output=True,
        )
        for entry in result.stdout.split(b"\0"):
            if not entry:
                continue
            metadata, relative = entry.decode("utf-8").split("\t", 1)
            mode, _object_id, stage = metadata.split()
            if stage != "0":
                conflicted_paths.add(relative)
            index_modes[relative] = mode
        failures.extend(
            f"{relative}: unresolved merge conflict"
            for relative in sorted(conflicted_paths)
        )

    if args.staged:
        package_dirs = {
            parts[1]
            for path in paths
            if len(parts := path.relative_to(ROOT).parts) == 3
            and parts[0] == "src" and parts[2] == "package.xml"
        }
    else:
        package_dirs = {
            path.parent.name
            for path in (ROOT / "src").glob("*/package.xml")
        }
    if package_dirs != ALLOWED_PACKAGES:
        failures.append(
            "src/: ROS package set is "
            f"{sorted(package_dirs)}, expected {sorted(ALLOWED_PACKAGES)}"
        )

    for path in paths:
        relative = path.relative_to(ROOT).as_posix()
        if relative in conflicted_paths:
            continue
        mode = index_modes.get(relative)
        if mode == "160000":
            failures.append(f'{relative}: dependency checkouts must remain outside this source-only release')
            continue
        if mode == "120000" or (not args.staged and path.is_symlink()):
            failures.append(f"{relative}: symbolic links are not publishable")
            continue
        if not args.staged and not path.is_file():
            continue
        private_name = (
            path.name in FORBIDDEN_NAMES
            or (path.name.startswith(".env.") and path.name != ".env.example")
            or path.name.endswith((".local.yaml", ".local.yml"))
            or path.suffix.lower() in FORBIDDEN_SUFFIXES
            or any(part in FORBIDDEN_DIRECTORIES for part in Path(relative).parts[:-1])
        )
        if private_name:
            failures.append(f"{relative}: forbidden runtime/credential file")
            continue
        data = None
        if args.staged:
            result = subprocess.run(
                ["git", "show", f":{relative}"], cwd=ROOT,
                check=True, capture_output=True,
            )
            data = result.stdout
        scan_text(path, relative, failures, data)

    if failures:
        print("Open-source hygiene check failed:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print(
        "Open-source hygiene check passed: "
        f"{len(paths)} {'staged' if args.staged else 'publishable'} files, "
        f"{len(package_dirs)} simulation packages, "
        "no local data/network/user-home/credential matches"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
