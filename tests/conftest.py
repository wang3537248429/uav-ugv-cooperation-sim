"""Use workspace sources consistently, including before a colcon build."""

from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
for package in sorted((ROOT / "src").iterdir()):
    if (package / "package.xml").is_file():
        sys.path.insert(0, str(package))


def pytest_collection_modifyitems(items):
    for item in items:
        if "ros" in Path(str(item.fspath)).relative_to(ROOT / "tests").parts:
            item.add_marker(pytest.mark.ros)
