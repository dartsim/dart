"""Check the reusable migration tools without modifying the bindings."""

import enum
import json
import subprocess
import sys
import types
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[4] / "scripts/nanobind"


@pytest.mark.parametrize(
    ("tool", "argument"),
    [
        ("codemod.py", "--self-check"),
        ("api_surface.py", "--self-check"),
        ("check_guards.py", "--self-test"),
    ],
)
def test_migration_tool_self_checks(tool, argument):
    result = subprocess.run(
        [sys.executable, str(TOOLS / tool), argument],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_codemod_preserves_an_existing_destination(tmp_path):
    sentinel = tmp_path / "keep.cpp"
    sentinel.write_text("existing binding\n")
    result = subprocess.run(
        [sys.executable, str(TOOLS / "codemod.py"), "--output", str(tmp_path)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 2
    assert "destination must not exist" in result.stderr
    assert sentinel.read_text() == "existing binding\n"


def test_codemod_requires_historical_sources(tmp_path):
    destination = tmp_path / "port"
    result = subprocess.run(
        [sys.executable, str(TOOLS / "codemod.py"), "--output", str(destination)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 2
    assert "--source must name an existing pybind11 source directory" in result.stderr
    assert not destination.exists()


@pytest.fixture
def api_surface(monkeypatch):
    monkeypatch.syspath_prepend(str(TOOLS))
    import api_surface

    return api_surface


@pytest.mark.parametrize("native", [False, True])
def test_api_property_preserves_raw_overloads(api_surface, native):
    getters = [
        "(self: X) -> float | None",
        '(self: X) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]", "flags.writeable"]',
    ]
    setters = [
        "(self: X, value: float | None) -> None",
        '(self: X, value: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None',
    ]

    def getter(self):
        pass

    def setter(self, value):
        pass

    for function, signatures in ((getter, getters), (setter, setters)):
        if native:
            function.__nb_signature__ = tuple(
                ("def " + signature, None, None) for signature in signatures
            )
        else:
            function.__doc__ = "\n".join(
                f"{index}. {signature}"
                for index, signature in enumerate(signatures, start=1)
            )
    row = api_surface.property_types(property(getter, setter))
    assert row["getter_signatures"] == getters
    assert row["setter_signatures"] == setters
    assert row["getter_types"] == row["setter_types"] == ["float", "array<float64,3>"]
    assert api_surface.property_types(property(getter))["setter_signatures"] == []


@pytest.mark.parametrize("native", [False, True])
def test_api_snapshot_masks_default_addresses(api_surface, monkeypatch, native):
    module = types.ModuleType("dartpy")
    module.__version__ = "test"

    class Example:
        def operation(self, option=None):
            pass

        @property
        def value(self, option=None):
            pass

        @value.setter
        def value(self, value=None):
            pass

    Example.__module__ = "dartpy"
    module.Example = Example
    monkeypatch.setitem(sys.modules, "dartpy", module)
    functions = (
        (Example.operation, "(self: X, option: object = {default}) -> bool"),
        (Example.value.fget, "(self: X, option: object = {default}) -> object"),
        (Example.value.fset, "(self: X, value: object = {default}) -> None"),
    )
    snapshots = []
    for default in (object(), object()):
        for function, signature in functions:
            if native:
                function.__nb_signature__ = (
                    ("def " + signature.format(default=r"\0"), None, (default,)),
                )
            else:
                function.__doc__ = signature.format(default=repr(default))
        rows = api_surface.snapshot()
        for path, key, signature in (
            ("operation", "signatures", functions[0][1]),
            ("value", "getter_signatures", functions[1][1]),
            ("value", "setter_signatures", functions[2][1]),
        ):
            assert rows[f"dartpy.Example.{path}"][key] == [
                signature.format(default="<object object at ADDRESS>")
            ]
        serialized = json.dumps(rows, indent=2)
        assert "0x" not in serialized
        snapshots.append(serialized)
    assert snapshots[0] == snapshots[1]
    assert not any(
        api_surface.compare(*(json.loads(row) for row in snapshots)).values()
    )


def test_api_snapshot_attributes_runtime_aliases(api_surface, monkeypatch):
    module = types.ModuleType("dartpy")
    module.__version__ = "test"
    module.domain = types.ModuleType("dartpy.domain")

    class Example:
        @property
        def value(self):
            """(self: X) -> float | None"""

        @staticmethod
        def operation():
            """() -> None"""

    Example.__module__ = "dartpy.domain"
    Example.__qualname__ = "Example"
    module.Alias = module.domain.Example = Example
    module.domain.Mode = enum.Enum("Mode", {"FIRST": 1, "ALIAS": 1})
    module.domain.Mode.__module__ = "dartpy.domain"
    module.AliasMode = module.domain.Mode
    module.operation = Example.operation
    module.z_domain = module.domain
    module.domain.root = module
    monkeypatch.setitem(sys.modules, "dartpy", module)

    rows = api_surface.snapshot()
    assert rows["dartpy.domain.Example"]["runtime"] == {
        "identity_path": "dartpy.Alias",
        "module": "dartpy.domain",
        "qualname": "Example",
    }
    assert rows["dartpy.Alias"]["runtime"] == rows["dartpy.domain.Example"]["runtime"]
    assert rows["dartpy.z_domain"]["runtime"]["identity_path"] == "dartpy.domain"
    assert rows["dartpy.z_domain"]["runtime"]["name"] == "dartpy.domain"
    assert rows["dartpy.domain.root"]["runtime"]["identity_path"] == "dartpy"
    assert rows["dartpy.domain.Mode"]["runtime"]["identity_path"] == "dartpy.AliasMode"
    assert rows["dartpy.domain.Mode"]["members"] == {"FIRST": 1, "ALIAS": 1}
    assert (
        rows["dartpy.operation"]["runtime"]["identity_path"] == "dartpy.Alias.operation"
    )
    assert rows["dartpy.Alias.value"]["getter_signatures"] == [
        "(self: X) -> float | None"
    ]
    assert "dartpy.domain.Example.value" not in rows
    assert "0x" not in json.dumps(rows)
    assert rows == api_surface.snapshot()


def test_api_snapshot_attributes_inherited_members_to_defining_class(
    api_surface, monkeypatch
):
    module = types.ModuleType("dartpy")
    module.__version__ = "test"

    class Base:
        def method(self):
            """(self: Base) -> None"""

        @property
        def value(self):
            """(self: Base) -> int"""

    class Derived(Base):
        pass

    Base.__module__ = Derived.__module__ = "dartpy"
    # Visit the subclass first, before its defining owner is walked.
    module.ADerived = Derived
    module.ZBase = Base
    module.method = Base.method
    monkeypatch.setitem(sys.modules, "dartpy", module)

    rows = api_surface.snapshot()
    for name in ("method", "value"):
        identity = f"dartpy.ZBase.{name}"
        assert rows[identity]["runtime"]["identity_path"] == identity
        assert (
            rows[f"dartpy.ADerived.{name}"]["runtime"]["identity_path"] == identity
        )
    assert rows["dartpy.method"]["runtime"]["identity_path"] == "dartpy.ZBase.method"
    assert rows == api_surface.snapshot()


def test_api_snapshot_leaves_external_members_untracked(api_surface, monkeypatch):
    module = types.ModuleType("dartpy")
    module.__version__ = "test"

    class External:
        def method(self):
            pass

    class Example(External):
        pass

    Example.__module__ = "dartpy"
    module.Example = Example
    monkeypatch.setitem(sys.modules, "dartpy", module)

    rows = api_surface.snapshot()
    for name in ("__init__", "__eq__", "__repr__", "__str__", "method"):
        assert rows[f"dartpy.Example.{name}"]["runtime"]["identity_path"] is None


def test_api_snapshot_links_bound_classmethod_reexports(api_surface, monkeypatch):
    module = types.ModuleType("dartpy")
    module.__version__ = "test"

    class Base:
        @classmethod
        def operation(cls):
            """() -> None"""

    class Derived(Base):
        pass

    Base.__module__ = Derived.__module__ = "dartpy"
    module.ADerived = Derived
    module.ZBase = Base
    # A re-export visited before the class must still point at the owner.
    module.AOperation = Base.operation
    module.operation = Base.operation
    module.derived_operation = Derived.operation
    monkeypatch.setitem(sys.modules, "dartpy", module)

    rows = api_surface.snapshot()
    for path in (
        "dartpy.ADerived.operation",
        "dartpy.ZBase.operation",
        "dartpy.AOperation",
        "dartpy.operation",
    ):
        assert rows[path]["runtime"]["identity_path"] == "dartpy.ZBase.operation"
    assert (
        rows["dartpy.derived_operation"]["runtime"]["identity_path"]
        == "dartpy.derived_operation"
    )
    assert rows == api_surface.snapshot()


def test_api_compare_ignores_additive_inventory_metadata(api_surface):
    legacy = {
        "module": {"kind": "module"},
        "class": {"kind": "class"},
        "enum": {"kind": "enum", "members": {"ONE": 1}},
        "callable": {
            "kind": "callable",
            "signatures": ["() -> int"],
            "normalized": [{"arguments": [], "return": "int"}],
        },
        "property": {
            "kind": "property",
            "writable": False,
            "getter_types": ["float"],
            "setter_types": [],
        },
    }
    enriched = {
        path: {**row, "runtime": {"identity_path": path}}
        for path, row in legacy.items()
    }
    enriched["property"].update(
        getter_signatures=["(self: X) -> float | None"], setter_signatures=[]
    )
    assert not any(api_surface.compare(legacy, enriched).values())
    assert not any(api_surface.compare(enriched, legacy).values())
    enriched["property"]["writable"] = True
    diff = api_surface.compare(legacy, enriched)
    assert len(diff["property_types"]) == 1
    assert not any(value for key, value in diff.items() if key != "property_types")
    assert "runtime" not in diff["property_types"][0]["nb"]
    assert "getter_signatures" not in diff["property_types"][0]["nb"]
