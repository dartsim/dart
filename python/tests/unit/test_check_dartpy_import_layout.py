import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_dartpy_import_layout.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_dartpy_import_layout", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _write_minimal_layout(root: Path) -> None:
    _write(
        root / "python" / "dartpy" / "dartpy.cpp",
        """
auto simulation = m.def_submodule(
    "simulation", "ECS-backed simulation API");
dart::python_nb::defSimulationModule(simulation);
""",
    )
    _write(
        root / "python" / "dartpy" / "_layout.py",
        """
_LEGACY_MODULES: tuple[str, ...] = ("common", "dynamics")
_PROMOTE_MODULES: tuple[str, ...] = ("simulation",) + tuple(
    name for name in _LEGACY_MODULES
)
""",
    )
    _write(
        root / "python" / "dartpy" / "__init__.py",
        '''
module_name = f"{__name__}.simulation.diff"
setattr(sys.modules[__name__], "diff", diff_module)
''',
    )
    _write(
        root / "python" / "dartpy" / "_world_render_bridge.py",
        '''
class WorldRenderBridge:
    def renderable_provider(self): ...
''',
    )
    _write(
        root / "python" / "stubs" / "dartpy" / "__init__.pyi",
        '''
from . import diff
from . import io
from . import simulation
from .simulation import (
    World,
)
__all__: list[str] = [
    "World",
    "diff",
    "io",
    "simulation",
]
''',
    )
    _write(
        root / "python" / "stubs" / "dartpy" / "simulation.pyi",
        '''
from dartpy import diff as diff
__all__: list[str] = [
    "World",
]
class World:
    pass
''',
    )
    _write(
        root / "python" / "stubs" / "dartpy" / "io" / "SdfParser.pyi",
        '''
__all__: list[str] = ["Options", "RootJointType", "readSkeleton", "read_skeleton"]
def readSkeleton(*args, **kwargs): ...
read_skeleton = readSkeleton
''',
    )
    _write(
        root / "python" / "stubs" / "dartpy" / "io" / "MjcfParser.pyi",
        '''
__all__: list[str] = ["Options"]
class Options:
    pass
''',
    )
    _write(
        root / "python" / "stubs" / "dartpy" / "io" / "__init__.pyi",
        '''
class UrdfParser:
    def parseSkeleton(*args, **kwargs): ...
    parse_skeleton = parseSkeleton
''',
    )


def test_current_repo_static_import_layout_passes():
    module = _load_module()

    assert module.find_static_violations(ROOT) == []


def test_minimal_static_layout_passes(tmp_path):
    module = _load_module()
    _write_minimal_layout(tmp_path)

    assert module.find_static_violations(tmp_path) == []


def test_static_check_rejects_second_simulation_experimental_module(tmp_path):
    module = _load_module()
    _write_minimal_layout(tmp_path)
    dartpy_cpp = tmp_path / "python" / "dartpy" / "dartpy.cpp"
    dartpy_cpp.write_text(
        dartpy_cpp.read_text(encoding="utf-8")
        + 'auto sx = m.def_submodule("simulation_experimental");\n',
        encoding="utf-8",
    )

    messages = [violation.message for violation in module.find_static_violations(tmp_path)]

    assert any("simulation_experimental must not be a second" in m for m in messages)


def test_static_check_rejects_python_world_loader_stubs(tmp_path):
    module = _load_module()
    _write_minimal_layout(tmp_path)
    sdf_stub = tmp_path / "python" / "stubs" / "dartpy" / "io" / "SdfParser.pyi"
    sdf_stub.write_text(
        sdf_stub.read_text(encoding="utf-8")
        + "\ndef readWorld(*args, **kwargs): ...\nread_world = readWorld\n",
        encoding="utf-8",
    )

    messages = [violation.message for violation in module.find_static_violations(tmp_path)]

    assert any("whole-world loader readWorld" in m for m in messages)
    assert any("whole-world loader read_world" in m for m in messages)


def test_runtime_check_can_be_static_only_when_dartpy_is_missing(monkeypatch):
    module = _load_module()

    def missing_import(name):
        if name == "dartpy":
            raise ModuleNotFoundError("No module named 'dartpy'", name="dartpy")
        raise AssertionError(name)

    monkeypatch.setattr(module.importlib, "import_module", missing_import)

    assert module.find_runtime_violations(require_runtime=False) == []
    assert module.find_runtime_violations(require_runtime=True)
