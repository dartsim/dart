"""Check how setup.py reads the CMake generator from its arguments."""

import ast
from pathlib import Path
from typing import List

ROOT = Path(__file__).resolve().parents[1]


def _load_cmake_generator():
    tree = ast.parse((ROOT / "setup.py").read_text(encoding="utf-8-sig"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_cmake_generator"
    )
    namespace = {"List": List}
    exec(compile(ast.Module([function], []), "setup.py", "exec"), namespace)
    return namespace["_cmake_generator"]


def test_g_arguments_override_the_environment_generator():
    cmake_generator = _load_cmake_generator()
    assert cmake_generator([], "Visual Studio 17 2022") == "Visual Studio 17 2022"
    assert cmake_generator(["-DX=ON", "-G", "Ninja"], "") == "Ninja"
    assert cmake_generator(["-GNMake Makefiles"], "Ninja") == "NMake Makefiles"
    assert cmake_generator(["-G"], "Ninja") == "Ninja"
