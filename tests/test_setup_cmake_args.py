"""Check how setup.py reads the CMake generator from its arguments."""

import ast
from pathlib import Path
from typing import List, Tuple

ROOT = Path(__file__).resolve().parents[1]


def _load(name):
    tree = ast.parse((ROOT / "setup.py").read_text(encoding="utf-8-sig"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    namespace = {"List": List, "Tuple": Tuple}
    exec(compile(ast.Module([function], []), "setup.py", "exec"), namespace)
    return namespace[name]


def test_g_arguments_override_the_environment_generator():
    cmake_generator = _load("_cmake_generator")
    assert cmake_generator([], "Visual Studio 17 2022") == "Visual Studio 17 2022"
    assert cmake_generator(["-DX=ON", "-G", "Ninja"], "") == "Ninja"
    assert cmake_generator(["-GNMake Makefiles"], "Ninja") == "NMake Makefiles"
    assert cmake_generator(["-G"], "Ninja") == "Ninja"


def test_windows_generator_traits():
    traits = _load("_windows_generator_traits")
    # (single_config, takes_platform)
    assert traits("") == (False, True)
    assert traits("Visual Studio 17 2022") == (False, True)
    assert traits("Ninja") == (True, False)
    assert traits("NMake Makefiles") == (True, False)
    assert traits("Ninja Multi-Config") == (False, False)
