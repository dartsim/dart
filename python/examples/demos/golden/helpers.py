"""Helpers for the cross-language golden-set parity smoke.

A "golden" scene exists in both C++ and Python with identical content and
deterministic state. Both languages capture state after ``DEFAULT_STEPS`` steps
and assert it matches the shared fixture in this directory. The Python side
captures via :func:`capture_state`; the C++ side reads the same JSON and
compares.
"""

from __future__ import annotations

import json
import math
import pathlib
from typing import Any

# Golden scenes mirror their C++ counterparts and ship a fixture in this dir.
# Add a scene here once both languages produce identical state for it.
GOLDEN_SCENE_IDS: tuple[str, ...] = ("hello_world", "boxes")

# Deterministic step count used by all golden fixtures. The DART World default
# time step (1e-3 s) is the same in C++ and Python, so identical scene
# definitions produce identical state across languages without explicit setup.
DEFAULT_STEPS: int = 60

_GOLDEN_DIR = pathlib.Path(__file__).resolve().parent


def fixture_path(scene_id: str) -> pathlib.Path:
    return _GOLDEN_DIR / f"{scene_id}.json"


def capture_state(setup: Any) -> dict[str, list[list[float]]]:
    """Capture the positions of every skeleton listed in
    ``setup.info["golden_skeletons"]``. Returns a dict mapping skeleton name
    to positions (a list of floats, one per DoF).

    Skeletons without DoFs (e.g. a welded ground) are still captured as an
    empty list so additions/removals show up in the fixture diff.
    """

    skeleton_names: list[str] = list(setup.info.get("golden_skeletons", []))
    state: dict[str, list[list[float]]] = {}
    for name in skeleton_names:
        skel = setup.world.get_skeleton(name)
        if skel is None:
            state[name] = []
            continue
        positions = skel.get_positions()
        try:
            state[name] = [float(v) for v in positions]
        except TypeError:
            state[name] = list(positions)
    return state


def load_fixture(scene_id: str) -> dict[str, Any]:
    path = fixture_path(scene_id)
    return json.loads(path.read_text())


def _all_close(actual: list[float], expected: list[float], abs_tol: float) -> bool:
    if len(actual) != len(expected):
        return False
    return all(math.isclose(a, e, abs_tol=abs_tol, rel_tol=1e-12)
               for a, e in zip(actual, expected))


def compare_states(
    actual: dict[str, list[float]],
    expected: dict[str, list[float]],
    abs_tol: float = 1e-9,
) -> list[str]:
    """Return a list of human-readable mismatch descriptions; empty means
    actual ≡ expected within ``abs_tol`` per element."""

    errors: list[str] = []
    if set(actual.keys()) != set(expected.keys()):
        errors.append(
            f"skeleton set mismatch: actual={sorted(actual)} "
            f"expected={sorted(expected)}"
        )
    for name in sorted(set(actual) | set(expected)):
        a = actual.get(name)
        e = expected.get(name)
        if a is None or e is None:
            continue
        if not _all_close(a, e, abs_tol):
            errors.append(
                f"skeleton '{name}': actual={a} expected={e} (tol={abs_tol})"
            )
    return errors
