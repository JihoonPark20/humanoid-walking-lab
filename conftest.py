"""Lab마다 같은 이름의 모듈을 쓰는 경우 pytest import 충돌을 방지한다.

각 테스트 파일 수집 전에 해당 Lab의 src를 sys.path 앞에 두고,
다른 Lab에서 가져온 동명 모듈을 sys.modules에서 제거한다.
예를 들어 Lab 1·2의 a4_inverse_kinematics는 이름은 같지만 서로 다른 구현이다."""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent


def _owning_lab_src(path: Path) -> Path | None:
    """Return the ``src/`` of the lab directory containing ``path``."""
    for parent in path.resolve().parents:
        if parent.parent == _ROOT and parent.name.startswith("lab-"):
            src = parent / "src"
            return src if src.is_dir() else None
    return None


def _activate(src: Path) -> None:
    """Make ``src`` win imports, dropping same-named modules from other labs."""
    src_str = str(src)
    if src_str in sys.path:
        sys.path.remove(src_str)
    sys.path.insert(0, src_str)

    local_names = {p.stem for p in src.glob("*.py")}
    for name in list(sys.modules):
        if name not in local_names:
            continue
        module_file = getattr(sys.modules[name], "__file__", None)
        if module_file and not Path(module_file).resolve().is_relative_to(src):
            del sys.modules[name]


def pytest_collectstart(collector) -> None:  # noqa: D103 - pytest hook
    path = getattr(collector, "path", None)
    if path is None:
        return
    src = _owning_lab_src(Path(path))
    if src is not None:
        _activate(src)
