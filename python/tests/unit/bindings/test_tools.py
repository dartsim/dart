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


def test_api_surface_budget(api_surface):
    """Pin World's current surface; the DART 7 target is about 20 public names."""
    budget = json.loads(api_surface.BUDGET_PATH.read_text())
    failures = api_surface.budget_growth(
        api_surface.budget_surface(api_surface.snapshot()), budget, check_stale=True
    )
    assert not failures, (
        "\n".join(failures)
        + "\nThe functions/classes counters cannot distinguish old names from "
        + "additions; container failures name the additions directly. "
        + "Revert growth, or deliberately raise budgets with "
        + f"{api_surface.BUDGET_UPDATE} --allow-growth. "
        + f"Lowering is automatic with {api_surface.BUDGET_UPDATE}."
    )


def test_api_budget_uses_canonical_snapshot_identities(api_surface, monkeypatch):
    module = types.ModuleType("dartpy")
    module.__version__ = "test"
    module.domain = types.ModuleType("dartpy.domain")
    module.a_alias = module.domain

    class External:
        def externalMethod(self):
            pass

    class Base(External):
        @property
        def m_value(self):
            return 1

        def getFoo(self):
            pass

        @property
        def value(self):
            pass

    class Derived(Base):
        def set_value(self):
            pass

    class Inner:
        def SetValue(self):
            pass

    class Outer:
        pass

    for cls in (Base, Derived, Outer, Inner):
        cls.__module__ = "dartpy.domain"
        cls.__qualname__ = cls.__name__
    Inner.__qualname__ = "Outer.Inner"
    Outer.Inner = Inner
    module.AInner = Inner
    module.domain.ADerived = Derived
    module.domain.Base = Base
    module.domain.Outer = Outer
    module.BaseAlias = Base
    module.getAlias = Base.getFoo
    module.domain.Mode = enum.Enum("Mode", {"FIRST": 1, "SECOND": 2})
    module.domain.Mode.__module__ = "dartpy.domain"

    def get_value():
        pass

    module.domain.get_value = module.domain.z_alias = get_value
    module.domain._private = lambda: None
    monkeypatch.setitem(sys.modules, "dartpy", module)
    surface = api_surface.budget_surface(api_surface.snapshot())
    # A module-level alias of a method counts where it is exported.
    assert surface["dartpy"]["functions"] == {
        "public": ["getAlias"],
        "camel_case": ["getAlias"],
        "get_set": ["getAlias"],
    }
    domain = surface["dartpy.domain"]
    assert domain["functions"] == {
        "public": ["get_value", "z_alias"],
        "camel_case": [],
        "get_set": ["get_value"],
    }
    assert domain["classes"] == {
        "Base": {
            "public": ["getFoo", "m_value", "value"],
            "camel_case": ["getFoo", "m_value"],
            "get_set": ["getFoo"],
        },
        "Derived": {
            "public": ["set_value"],
            "camel_case": [],
            "get_set": ["set_value"],
        },
        "Mode": {"public": ["FIRST", "SECOND"], "camel_case": [], "get_set": []},
        "Outer": {"public": ["Inner"], "camel_case": [], "get_set": []},
        "Outer.Inner": {
            "public": ["SetValue"],
            "camel_case": ["SetValue"],
            "get_set": ["SetValue"],
        },
    }
    assert len(surface) == 2
    assert surface == api_surface.budget_surface(api_surface.snapshot())


def test_api_budget_counts_same_name_reexport_in_its_module(api_surface):
    """A function re-exported under the same name counts where it is exported."""
    rows = {
        "dartpy.domain": {"kind": "module", "runtime": {"name": "dartpy.domain"}},
        "dartpy.domain.get_value": {
            "kind": "callable",
            "runtime": {"identity_path": "dartpy.domain.get_value"},
        },
    }
    reexported = dict(rows)
    reexported["dartpy.get_value"] = {
        "kind": "callable",
        "runtime": {"identity_path": "dartpy.domain.get_value"},
    }
    surface = api_surface.budget_surface(rows)
    assert surface["dartpy"]["functions"]["public"] == []
    assert surface["dartpy.domain"]["functions"]["public"] == ["get_value"]
    budget = {
        module: {
            "functions": {
                key: len(names) for key, names in scope["functions"].items()
            },
            "containers": scope["containers"]["public"],
            "classes": {},
        }
        for module, scope in surface.items()
    }
    grown = api_surface.budget_surface(reexported)
    assert grown["dartpy"]["functions"]["public"] == ["get_value"]
    assert any(
        "dartpy (functions): public budget 0 -> 1" in failure
        for failure in api_surface.budget_growth(grown, budget)
    )


def test_api_budget_counts_function_and_member_aliases(api_surface):
    """Aliases share an identity but export distinct public names."""
    single = {
        "dartpy.getFoo": {
            "kind": "callable",
            "runtime": {"identity_path": "dartpy.getFoo"},
        },
    }
    aliased = dict(single)
    aliased["dartpy.getFooAlias"] = {
        "kind": "callable",
        "runtime": {"identity_path": "dartpy.getFoo"},
    }
    budget = {
        "dartpy": {
            "functions": {"public": 1, "camel_case": 1, "get_set": 1},
            "containers": [],
            "classes": {},
        }
    }
    assert api_surface.budget_surface(single)["dartpy"]["functions"]["public"] == [
        "getFoo"
    ]
    surface = api_surface.budget_surface(aliased)
    assert surface["dartpy"]["functions"]["public"] == ["getFoo", "getFooAlias"]
    assert any(
        "dartpy (functions)" in failure and "public budget 1 -> 2" in failure
        for failure in api_surface.budget_growth(surface, budget)
    )

    member_single = {
        "dartpy.Foo": {
            "kind": "class",
            "runtime": {"module": "dartpy", "qualname": "Foo"},
        },
        "dartpy.Foo.method": {
            "kind": "callable",
            "runtime": {"identity_path": "dartpy.Foo.method"},
        },
    }
    member_aliased = dict(member_single)
    member_aliased["dartpy.Foo.alias"] = {
        "kind": "callable",
        "runtime": {"identity_path": "dartpy.Foo.method"},
    }
    member_budget = {
        "dartpy": {
            "functions": {"public": 0, "camel_case": 0, "get_set": 0},
            "containers": ["Foo"],
            "classes": {"Foo": {"public": 1, "camel_case": 0, "get_set": 0}},
        }
    }
    member_surface = api_surface.budget_surface(member_aliased)
    assert member_surface["dartpy"]["classes"]["Foo"]["public"] == [
        "alias",
        "method",
    ]
    assert any(
        "dartpy.Foo" in failure and "public budget 1 -> 2" in failure
        for failure in api_surface.budget_growth(member_surface, member_budget)
    )

    # An inherited member keeps its defining name, so the subclass adds nothing.
    inherited_rows = {
        "dartpy.Base": {
            "kind": "class",
            "runtime": {"module": "dartpy", "qualname": "Base"},
        },
        "dartpy.Derived": {
            "kind": "class",
            "runtime": {"module": "dartpy", "qualname": "Derived"},
        },
        "dartpy.Base.method": {
            "kind": "callable",
            "runtime": {"identity_path": "dartpy.Base.method"},
        },
        "dartpy.Derived.method": {
            "kind": "callable",
            "runtime": {"identity_path": "dartpy.Base.method"},
        },
    }
    inherited = api_surface.budget_surface(inherited_rows)
    assert inherited["dartpy"]["classes"]["Base"]["public"] == ["method"]
    assert inherited["dartpy"]["classes"]["Derived"]["public"] == []
    inherited_budget = {
        "dartpy": {
            "functions": {"public": 0, "camel_case": 0, "get_set": 0},
            "containers": ["Base", "Derived"],
            "classes": {
                "Base": {"public": 1, "camel_case": 0, "get_set": 0},
                "Derived": {"public": 0, "camel_case": 0, "get_set": 0},
            },
        }
    }
    assert not api_surface.budget_growth(
        inherited, inherited_budget, check_stale=True
    )


def test_api_budget_reports_unconditional_deletions_but_not_optional(api_surface):
    """Only feature-conditional owners are exempt when absent."""
    full_rows = {
        "dartpy.collision": {"kind": "module", "runtime": {"name": "dartpy.collision"}},
        "dartpy.collision.Core": {
            "kind": "class",
            "runtime": {"module": "dartpy.collision", "qualname": "Core"},
        },
        "dartpy.collision.BulletCollisionDetector": {
            "kind": "class",
            "runtime": {
                "module": "dartpy.collision",
                "qualname": "BulletCollisionDetector",
            },
        },
    }
    full = api_surface.budget_surface(full_rows)
    budget = {
        module: {
            "functions": {key: len(names) for key, names in scope["functions"].items()},
            "containers": sorted(scope["containers"]["public"]),
            "classes": {
                cls: {key: len(names) for key, names in counters.items()}
                for cls, counters in scope["classes"].items()
            },
        }
        for module, scope in full.items()
    }
    assert not api_surface.budget_growth(full, budget, check_stale=True)

    # Omitting the HAVE_BULLET class passes on a reduced build...
    reduced_rows = dict(full_rows)
    del reduced_rows["dartpy.collision.BulletCollisionDetector"]
    reduced = api_surface.budget_surface(reduced_rows)
    assert not api_surface.budget_growth(reduced, budget, check_stale=True)

    # ...but deleting the unconditional class is a staleness failure.
    deleted_rows = dict(full_rows)
    del deleted_rows["dartpy.collision.Core"]
    deleted = api_surface.budget_surface(deleted_rows)
    assert api_surface.budget_growth(deleted, budget, check_stale=True)

    submodule_rows = {
        "dartpy.collision": {"kind": "module", "runtime": {"name": "dartpy.collision"}},
        "dartpy.viewer": {"kind": "module", "runtime": {"name": "dartpy.viewer"}},
    }
    submodule_surface = api_surface.budget_surface(submodule_rows)
    submodule_budget = {
        module: {
            "functions": {key: len(names) for key, names in scope["functions"].items()},
            "containers": sorted(scope["containers"]["public"]),
            "classes": {
                cls: {key: len(names) for key, names in counters.items()}
                for cls, counters in scope["classes"].items()
            },
        }
        for module, scope in submodule_surface.items()
    }
    viewer_absent = api_surface.budget_surface(
        {"dartpy.collision": submodule_rows["dartpy.collision"]}
    )
    assert not api_surface.budget_growth(
        viewer_absent, submodule_budget, check_stale=True
    )
    collision_absent = api_surface.budget_surface(
        {"dartpy.viewer": submodule_rows["dartpy.viewer"]}
    )
    assert api_surface.budget_growth(
        collision_absent, submodule_budget, check_stale=True
    )


def test_api_budget_update_ratchets_and_keeps_absent_owners(
    api_surface, tmp_path, capsys
):
    path = tmp_path / "budget.json"
    empty = {"public": [], "camel_case": [], "get_set": []}
    members = {key: ["getFoo"] for key in empty}
    surface = {
        "dartpy": {"functions": empty, "classes": {}},
        "dartpy.collision": {
            "functions": empty,
            "classes": {"Example": members, "BulletCollisionDetector": members},
        },
        "dartpy.viewer": {"functions": members, "classes": {}},
    }
    with pytest.raises(ValueError, match="--allow-growth"):
        api_surface.update_budget(path, surface)
    assert not path.exists()
    api_surface.update_budget(path, surface, allow_growth=True)
    output = capsys.readouterr().out
    for owner in (
        "dartpy.collision.Example",
        "dartpy.collision.BulletCollisionDetector",
        "dartpy.viewer (functions)",
    ):
        for counter in empty:
            assert f"{owner}: {counter} budget 0 -> 1" in output
    original = json.loads(path.read_text())
    assert original["dartpy.collision"]["classes"]["Example"] == {
        "public": 1,
        "camel_case": 1,
        "get_set": 1,
    }

    smaller = {
        "dartpy": {"functions": empty, "classes": {}},
        "dartpy.collision": {"functions": empty, "classes": {"Example": empty}},
    }
    api_surface.update_budget(path, smaller)
    lowered = json.loads(path.read_text())
    assert lowered["dartpy.collision"]["classes"]["Example"] == {
        "public": 0,
        "camel_case": 0,
        "get_set": 0,
    }
    assert lowered["dartpy.viewer"] == original["dartpy.viewer"]
    assert lowered["dartpy.collision"]["classes"]["BulletCollisionDetector"] == (
        original["dartpy.collision"]["classes"]["BulletCollisionDetector"]
    )
    assert not api_surface.budget_growth(smaller, lowered)
    assert not api_surface.budget_growth(smaller, lowered, check_stale=True)
    with pytest.raises(
        ValueError, match="dartpy.collision.Example: public budget 0 -> 1.*getFoo"
    ):
        api_surface.update_budget(path, surface)
    assert json.loads(path.read_text()) == lowered

    api_surface.update_budget(path, surface, allow_growth=True)
    absent = {"dartpy": {"functions": empty, "classes": {}}}
    api_surface.update_budget(path, absent)
    kept = json.loads(path.read_text())
    # dartpy.collision is unconditional, so its absence is a removal; the
    # optional viewer is kept for builds that have it.
    assert "dartpy.collision" not in kept
    assert kept["dartpy.viewer"] == original["dartpy.viewer"]
    assert not api_surface.budget_growth(absent, kept, check_stale=True)
    api_surface.update_budget(path, absent, prune=True)
    assert json.loads(path.read_text()) == {
        "dartpy": {"functions": dict.fromkeys(empty, 0), "classes": {}}
    }
    assert (
        "dartpy.collision.Example: public budget 0 -> 1"
        in api_surface.budget_growth(surface, json.loads(path.read_text()))[0]
    )


def test_api_budget_counts_nested_container_aliases(api_surface):
    rows = {
        "dartpy.collision": {
            "kind": "module",
            "runtime": {"name": "dartpy.collision"},
        },
        "dartpy.collision.FCL": {
            "kind": "class",
            "runtime": {"module": "dartpy.collision", "qualname": "FCL"},
        },
        "dartpy.collision.FCL.PrimitiveShape": {
            "kind": "enum",
            "runtime": {"module": "dartpy.collision", "qualname": "FCL.PrimitiveShape"},
            "members": {"BOX": 0},
        },
    }
    surface = api_surface.budget_surface(rows)
    fcl = surface["dartpy.collision"]["classes"]["FCL"]
    assert fcl["public"] == ["PrimitiveShape"]
    budget = {
        module: {
            "functions": {key: len(v) for key, v in scope["functions"].items()},
            "containers": scope["containers"]["public"],
            "classes": {
                cls: {key: len(v) for key, v in counters.items()}
                for cls, counters in scope["classes"].items()
            },
        }
        for module, scope in surface.items()
    }
    aliased_rows = dict(rows)
    aliased_rows["dartpy.collision.FCL.PrimitiveShapeAlias"] = {
        "kind": "enum",
        "runtime": {
            "module": "dartpy.collision",
            "qualname": "FCL.PrimitiveShape",
            "identity_path": "dartpy.collision.FCL.PrimitiveShape",
        },
        "members": {"BOX": 0},
    }
    aliased = api_surface.budget_surface(aliased_rows)
    assert aliased["dartpy.collision"]["classes"]["FCL"]["public"] == [
        "PrimitiveShape",
        "PrimitiveShapeAlias",
    ]
    assert any(
        "dartpy.collision.FCL: public budget 1 -> 2" in failure
        for failure in api_surface.budget_growth(aliased, budget)
    )


def test_api_budget_update_drops_removed_unconditional_class(api_surface, tmp_path):
    path = tmp_path / "budget.json"
    empty = {"public": [], "camel_case": [], "get_set": []}
    full = {
        "dartpy": {"functions": empty, "containers": empty, "classes": {}},
        "dartpy.collision": {
            "functions": empty,
            "containers": {"public": ["Core", "Gone"], "camel_case": [], "get_set": []},
            "classes": {"Core": empty, "Gone": empty},
        },
    }
    api_surface.update_budget(path, full, allow_growth=True)
    removed = {
        "dartpy": full["dartpy"],
        "dartpy.collision": {
            "functions": empty,
            "containers": {"public": ["Core"], "camel_case": [], "get_set": []},
            "classes": {"Core": empty},
        },
    }
    failures = api_surface.budget_growth(
        removed, json.loads(path.read_text()), check_stale=True
    )
    assert any("dartpy.collision.Gone: budgeted class missing" in f for f in failures)
    api_surface.update_budget(path, removed)
    lowered = json.loads(path.read_text())
    assert "Gone" not in lowered["dartpy.collision"]["classes"]
    assert not api_surface.budget_growth(removed, lowered, check_stale=True)


def test_api_budget_update_handles_removed_and_optional_absences_together(
    api_surface, tmp_path
):
    path = tmp_path / "budget.json"
    empty = {"public": [], "camel_case": [], "get_set": []}

    def containers(*names):
        return {"public": list(names), "camel_case": [], "get_set": []}

    full = {
        "dartpy": {
            "functions": empty,
            "containers": containers("collision", "viewer"),
            "classes": {},
        },
        "dartpy.collision": {"functions": empty, "containers": empty, "classes": {}},
        "dartpy.viewer": {"functions": empty, "containers": empty, "classes": {}},
    }
    api_surface.update_budget(path, full, allow_growth=True)
    # dartpy.collision was removed; the optional viewer is just not built.
    neither = {
        "dartpy": {"functions": empty, "containers": containers(), "classes": {}}
    }
    api_surface.update_budget(path, neither)
    budget = json.loads(path.read_text())
    assert "dartpy.collision" not in budget
    assert budget["dartpy"]["containers"] == ["viewer"]
    # A later build with the viewer passes without staleness.
    with_viewer = {
        "dartpy": {
            "functions": empty,
            "containers": containers("viewer"),
            "classes": {},
        },
        "dartpy.viewer": full["dartpy.viewer"],
    }
    assert not api_surface.budget_growth(with_viewer, budget, check_stale=True)


def test_api_budget_rejects_stale_counters(api_surface, tmp_path):
    path = tmp_path / "budget.json"
    counters = {key: ["getFoo"] for key in ("public", "camel_case", "get_set")}
    surface = {"dartpy": {"functions": counters, "classes": {"Example": counters}}}
    stale = {
        "dartpy": {
            "functions": dict.fromkeys(counters, 2),
            "classes": {"Example": dict.fromkeys(counters, 2)},
        }
    }
    path.write_text(json.dumps(stale))
    failures = api_surface.budget_growth(surface, stale, check_stale=True)
    assert len(failures) == 6
    for failure in failures:
        assert "budget 2 exceeds current count 1" in failure
        assert f"run {api_surface.BUDGET_UPDATE} to lower it" in failure
        assert "no --allow-growth needed" in failure
    api_surface.update_budget(path, surface)
    assert not api_surface.budget_growth(
        surface, json.loads(path.read_text()), check_stale=True
    )


def test_api_budget_counts_module_data_but_not_containers(api_surface):
    rows = {
        f"dartpy.{name}": {"kind": kind}
        for name, kind in (
            ("constant", "constant"),
            ("FIRST", "enum_value"),
            ("value", "property"),
            ("descriptor", "descriptor"),
            ("operation", "callable"),
        )
    }
    rows["dartpy.Example"] = {
        "kind": "class",
        "runtime": {"module": "dartpy", "qualname": "Example"},
    }
    rows["dartpy.Mode"] = {
        "kind": "enum",
        "members": {"FIRST": 1},
        "runtime": {"module": "dartpy", "qualname": "Mode"},
    }
    rows["dartpy.domain"] = {
        "kind": "module",
        "runtime": {"name": "dartpy.domain"},
    }
    assert api_surface.budget_surface(rows)["dartpy"]["functions"]["public"] == [
        "FIRST",
        "constant",
        "descriptor",
        "operation",
        "value",
    ]


def test_api_budget_container_ceilings_tolerate_absent_optional_classes(
    api_surface,
):
    full_rows = {
        "dartpy.collision": {"kind": "module", "runtime": {"name": "dartpy.collision"}},
        "dartpy.collision.Core": {
            "kind": "class",
            "runtime": {"module": "dartpy.collision", "qualname": "Core"},
        },
        "dartpy.collision.BulletCollisionDetector": {
            "kind": "class",
            "runtime": {
                "module": "dartpy.collision",
                "qualname": "BulletCollisionDetector",
            },
        },
    }
    full = api_surface.budget_surface(full_rows)
    budget = {
        module: {
            "functions": {key: len(names) for key, names in scope["functions"].items()},
            "containers": sorted(scope["containers"]["public"]),
            "classes": {
                cls: {key: len(names) for key, names in counters.items()}
                for cls, counters in scope["classes"].items()
            },
        }
        for module, scope in full.items()
    }
    assert not api_surface.budget_growth(full, budget, check_stale=True)

    # A build without the optional binding still passes...
    reduced_rows = dict(full_rows)
    del reduced_rows["dartpy.collision.BulletCollisionDetector"]
    reduced = api_surface.budget_surface(reduced_rows)
    assert not api_surface.budget_growth(reduced, budget, check_stale=True)
    # ...but an alias cannot take the absent class's place unnoticed.
    aliased_rows = dict(reduced_rows)
    aliased_rows["dartpy.collision.Extra"] = {
        "kind": "class",
        "runtime": {
            "module": "dartpy.collision",
            "qualname": "Core",
            "identity_path": "dartpy.collision.Core",
        },
    }
    aliased = api_surface.budget_surface(aliased_rows)
    assert aliased["dartpy.collision"]["containers"]["public"] == ["Core", "Extra"]
    assert any(
        "dartpy.collision (containers)" in failure and "Extra" in failure
        for failure in api_surface.budget_growth(aliased, budget, check_stale=True)
    )
    # ...and still reports growth.
    reduced_rows["dartpy.collision.Added"] = {
        "kind": "class",
        "runtime": {"module": "dartpy.collision", "qualname": "Added"},
    }
    grown = api_surface.budget_surface(reduced_rows)
    assert any(
        "dartpy.collision.Added: new class" in failure
        for failure in api_surface.budget_growth(grown, budget, check_stale=True)
    )

    # A stale container name is still reported, with or without absent
    # owners elsewhere.
    budget["dartpy"]["containers"].append("StaleDummy")
    for surface in (reduced, full):
        assert any(
            "dartpy (containers)" in failure and "StaleDummy" in failure
            for failure in api_surface.budget_growth(surface, budget, check_stale=True)
        )
    budget["dartpy"]["containers"].remove("StaleDummy")

    # An absent optional submodule skips its parent's container staleness, where
    # growth is still reported; an absent unconditional submodule fails.
    viewer_rows = {
        "dartpy.collision": {"kind": "module", "runtime": {"name": "dartpy.collision"}},
        "dartpy.viewer": {"kind": "module", "runtime": {"name": "dartpy.viewer"}},
    }
    viewer_surface = api_surface.budget_surface(viewer_rows)
    viewer_budget = {
        module: {
            "functions": {key: len(names) for key, names in scope["functions"].items()},
            "containers": sorted(scope["containers"]["public"]),
            "classes": {
                cls: {key: len(names) for key, names in counters.items()}
                for cls, counters in scope["classes"].items()
            },
        }
        for module, scope in viewer_surface.items()
    }
    viewer_absent = api_surface.budget_surface(
        {"dartpy.collision": viewer_rows["dartpy.collision"]}
    )
    assert not api_surface.budget_growth(
        viewer_absent, viewer_budget, check_stale=True
    )
    collision_absent = api_surface.budget_surface(
        {"dartpy.viewer": viewer_rows["dartpy.viewer"]}
    )
    assert api_surface.budget_growth(
        collision_absent, viewer_budget, check_stale=True
    )
    added = api_surface.budget_surface(
        {
            "dartpy.collision": viewer_rows["dartpy.collision"],
            "dartpy.Added": {
                "kind": "class",
                "runtime": {"module": "dartpy", "qualname": "Added"},
            },
        }
    )
    assert any(
        "dartpy.Added: new class" in failure
        for failure in api_surface.budget_growth(added, viewer_budget, check_stale=True)
    )


def test_api_budget_container_names_tolerate_absent_alias(api_surface, tmp_path):
    full_rows = {
        "dartpy.collision": {"kind": "module", "runtime": {"name": "dartpy.collision"}},
        "dartpy.collision.Core": {
            "kind": "class",
            "runtime": {"module": "dartpy.collision", "qualname": "Core"},
        },
        "dartpy.collision.BulletCollisionDetector": {
            "kind": "class",
            "runtime": {
                "module": "dartpy.collision",
                "qualname": "BulletCollisionDetector",
            },
        },
        "dartpy.collision.Alternate": {
            "kind": "class",
            "runtime": {
                "module": "dartpy.collision",
                "qualname": "BulletCollisionDetector",
            },
        },
    }
    full = api_surface.budget_surface(full_rows)
    assert full["dartpy.collision"]["containers"]["public"] == [
        "Alternate",
        "BulletCollisionDetector",
        "Core",
    ]
    budget = {
        module: {
            "functions": {key: len(names) for key, names in scope["functions"].items()},
            "containers": sorted(scope["containers"]["public"]),
            "classes": {
                cls: {key: len(names) for key, names in counters.items()}
                for cls, counters in scope["classes"].items()
            },
        }
        for module, scope in full.items()
    }
    assert not api_surface.budget_growth(full, budget, check_stale=True)

    # Both the optional class and its alias vanish, but the reduced build
    # still passes unchanged.
    reduced_rows = dict(full_rows)
    del reduced_rows["dartpy.collision.BulletCollisionDetector"]
    del reduced_rows["dartpy.collision.Alternate"]
    reduced = api_surface.budget_surface(reduced_rows)
    assert reduced["dartpy.collision"]["containers"]["public"] == ["Core"]
    assert not api_surface.budget_growth(reduced, budget, check_stale=True)

    # A new alias cannot hide behind the absent names.
    extra_rows = dict(reduced_rows)
    extra_rows["dartpy.collision.Extra"] = {
        "kind": "class",
        "runtime": {"module": "dartpy.collision", "qualname": "Core"},
    }
    extra = api_surface.budget_surface(extra_rows)
    assert any(
        "dartpy.collision (containers)" in failure and "Extra" in failure
        for failure in api_surface.budget_growth(extra, budget, check_stale=True)
    )

    # Deleting an unconditional nested class and its module-level export fails.
    nested_full_rows = {
        "dartpy.collision": {"kind": "module", "runtime": {"name": "dartpy.collision"}},
        "dartpy.collision.Core": {
            "kind": "class",
            "runtime": {"module": "dartpy.collision", "qualname": "Core"},
        },
        "dartpy.collision.Outer": {
            "kind": "class",
            "runtime": {"module": "dartpy.collision", "qualname": "Outer"},
        },
        "dartpy.collision.AInner": {
            "kind": "class",
            "runtime": {"module": "dartpy.collision", "qualname": "Outer.Inner"},
        },
    }
    nested_full = api_surface.budget_surface(nested_full_rows)
    nested_budget = {
        module: {
            "functions": {key: len(names) for key, names in scope["functions"].items()},
            "containers": sorted(scope["containers"]["public"]),
            "classes": {
                cls: {key: len(names) for key, names in counters.items()}
                for cls, counters in scope["classes"].items()
            },
        }
        for module, scope in nested_full.items()
    }
    nested_reduced_rows = dict(nested_full_rows)
    del nested_reduced_rows["dartpy.collision.Outer"]
    del nested_reduced_rows["dartpy.collision.AInner"]
    nested_reduced = api_surface.budget_surface(nested_reduced_rows)
    assert api_surface.budget_growth(
        nested_reduced, nested_budget, check_stale=True
    )
    nested_extra_rows = dict(nested_reduced_rows)
    nested_extra_rows["dartpy.collision.Extra"] = {
        "kind": "class",
        "runtime": {"module": "dartpy.collision", "qualname": "Core"},
    }
    assert any(
        "dartpy.collision (containers)" in failure and "Extra" in failure
        for failure in api_surface.budget_growth(
            api_surface.budget_surface(nested_extra_rows),
            nested_budget,
            check_stale=True,
        )
    )

    # The updater writes name lists, retaining absent names without --prune.
    path = tmp_path / "budget.json"
    path.write_text(json.dumps(budget))
    api_surface.update_budget(path, reduced)
    retained = json.loads(path.read_text())
    assert retained["dartpy.collision"]["containers"] == [
        "Alternate",
        "BulletCollisionDetector",
        "Core",
    ]
    with pytest.raises(ValueError, match="Extra"):
        api_surface.update_budget(path, extra)
    api_surface.update_budget(path, extra, allow_growth=True)
    grown = json.loads(path.read_text())
    assert grown["dartpy.collision"]["containers"] == [
        "Alternate",
        "BulletCollisionDetector",
        "Core",
        "Extra",
    ]
    api_surface.update_budget(path, reduced, prune=True)
    assert json.loads(path.read_text())["dartpy.collision"]["containers"] == ["Core"]


def test_api_budget_reports_empty_new_class(api_surface):
    rows = {
        "dartpy.Example": {
            "kind": "class",
            "runtime": {"module": "dartpy", "qualname": "Example"},
        }
    }
    surface = api_surface.budget_surface(rows)
    failures = api_surface.budget_growth(surface, {})
    assert any(
        "dartpy (containers)" in failure and "Example" in failure
        for failure in failures
    )
    assert any("dartpy.Example: new class" in failure for failure in failures)
    assert surface["dartpy"]["containers"]["public"] == ["Example"]
    assert surface["dartpy"]["classes"]["Example"]["public"] == []


def test_api_budget_reports_container_alias_in_second_module(api_surface):
    base_rows = {
        "dartpy.domain": {
            "kind": "module",
            "runtime": {"name": "dartpy.domain"},
        },
        "dartpy.domain.Foo": {
            "kind": "class",
            "runtime": {"module": "dartpy.domain", "qualname": "Foo"},
        },
        "dartpy.gui": {"kind": "module", "runtime": {"name": "dartpy.gui"}},
    }
    rows = dict(base_rows)
    rows["dartpy.gui.FooAlias"] = {
        "kind": "class",
        "runtime": {"module": "dartpy.domain", "qualname": "Foo"},
    }
    empty = {"public": 0, "camel_case": 0, "get_set": 0}
    budget = {
        "dartpy": {
            "functions": dict(empty),
            "containers": ["domain", "gui"],
            "classes": {},
        },
        "dartpy.domain": {
            "functions": dict(empty),
            "containers": ["Foo"],
            "classes": {"Foo": dict(empty)},
        },
        "dartpy.gui": {
            "functions": dict(empty),
            "containers": [],
            "classes": {},
        },
    }
    assert not api_surface.budget_growth(
        api_surface.budget_surface(base_rows), budget
    )
    surface = api_surface.budget_surface(rows)
    failures = api_surface.budget_growth(surface, budget)
    assert any(
        "dartpy.gui (containers)" in failure and "FooAlias" in failure
        for failure in failures
    )
    assert surface["dartpy.gui"]["containers"]["public"] == ["FooAlias"]
    assert surface["dartpy.domain"]["containers"]["public"] == ["Foo"]


def test_api_budget_reports_empty_new_enum(api_surface):
    rows = {
        "dartpy.Mode": {
            "kind": "enum",
            "members": {},
            "runtime": {"module": "dartpy", "qualname": "Mode"},
        }
    }
    surface = api_surface.budget_surface(rows)
    failures = api_surface.budget_growth(surface, {})
    assert any(
        "dartpy (containers)" in failure and "Mode" in failure
        for failure in failures
    )
    assert any("dartpy.Mode: new class" in failure for failure in failures)
    assert surface["dartpy"]["containers"]["public"] == ["Mode"]
    assert surface["dartpy"]["classes"]["Mode"]["public"] == []


def test_api_budget_reports_empty_new_submodule(api_surface):
    rows = {
        "dartpy.newsub": {
            "kind": "module",
            "runtime": {"name": "dartpy.newsub"},
        }
    }
    surface = api_surface.budget_surface(rows)
    failures = api_surface.budget_growth(surface, {})
    assert any(
        "dartpy (containers)" in failure and "newsub" in failure
        for failure in failures
    )
    assert any("dartpy.newsub: new module" in failure for failure in failures)
    assert surface["dartpy"]["containers"]["public"] == ["newsub"]
    assert surface["dartpy.newsub"]["containers"]["public"] == []


def test_api_budget_accessor_boundaries_and_single_word_pascal_case(api_surface):
    names = ["settings", "setup", "setUpView", "getFoo", "GET_FOO", "SET", "Step"]
    counters = api_surface.budget_surface(
        {f"dartpy.{name}": {"kind": "callable"} for name in names}
    )["dartpy"]["functions"]
    assert counters == {
        "public": sorted(names),
        "camel_case": ["getFoo", "setUpView"],
        "get_set": ["GET_FOO", "SET", "getFoo", "setUpView"],
    }


def test_api_budget_diagnostics_sort_and_truncate_names(api_surface):
    names = [f"name{i:02}" for i in reversed(range(43))]
    surface = {
        "dartpy": {"functions": {"public": names}, "classes": {}}
    }
    assert api_surface.budget_growth(surface, {}) == [
        "dartpy (functions): public budget 0 -> 43; "
        "current names (including additions): "
        + ", ".join(sorted(names)[:40])
        + ", ... (3 more)"
    ]


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
    # A property is described by its getter's module and qualified name.
    assert rows["dartpy.Alias.value"]["runtime"]["module"] == (
        Example.value.fget.__module__
    )
    assert rows["dartpy.Alias.value"]["runtime"]["qualname"] == (
        Example.value.fget.__qualname__
    )
    assert "dartpy.domain.Example.value" not in rows
    assert "0x" not in json.dumps(rows)
    assert rows == api_surface.snapshot()


def test_api_snapshot_attributes_inherited_members_to_defining_class(
    api_surface, monkeypatch
):
    module = types.ModuleType("dartpy")
    module.__version__ = "test"

    class Descriptor:
        def __get__(self, instance, owner):
            return self

    class Base:
        CONSTANT = 1
        descriptor = Descriptor()

        def method(self):
            """(self: Base) -> None"""

        @property
        def value(self):
            """(self: Base) -> int"""

    class Derived(Base):
        pass

    mode = enum.Enum("Mode", {"FIRST": 1})
    mode.__module__ = "dartpy"
    Base.Mode = mode
    Base.FIRST = mode.FIRST
    Base.__module__ = Derived.__module__ = "dartpy"
    Base.__qualname__ = "Base"
    Derived.__qualname__ = "Derived"
    # Visit the subclass first, before its defining owner is walked.
    module.ADerived = Derived
    module.ZBase = Base
    module.method = Base.method
    monkeypatch.setitem(sys.modules, "dartpy", module)

    rows = api_surface.snapshot()
    for name in ("method", "value", "CONSTANT", "descriptor", "FIRST"):
        identity = f"dartpy.ZBase.{name}"
        assert rows[identity]["runtime"]["identity_path"] == identity
        assert (
            rows[f"dartpy.ADerived.{name}"]["runtime"]["identity_path"] == identity
        )
    assert rows["dartpy.method"]["runtime"]["identity_path"] == "dartpy.ZBase.method"
    classes = api_surface.budget_surface(rows)["dartpy"]["classes"]
    assert classes["Derived"]["public"] == []
    assert classes["Base"]["public"] == [
        "CONSTANT", "FIRST", "Mode", "descriptor", "method", "value"
    ]
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
        "dartpy.derived_operation",
    ):
        assert rows[path]["runtime"]["identity_path"] == "dartpy.ZBase.operation"
    assert rows == api_surface.snapshot()


def test_api_snapshot_collapses_members_shared_by_unrelated_classes(
    api_surface, monkeypatch
):
    module = types.ModuleType("dartpy")
    module.__version__ = "test"

    class Inner:
        pass

    def helper():
        """() -> None"""

    class First:
        pass

    class Second:
        pass

    for cls in (Inner, First, Second):
        cls.__module__ = "dartpy"
    # Objects stored in two unrelated class dicts are one identity.
    First.Inner = Second.Inner = Inner
    First.helper = staticmethod(helper)
    Second.other = staticmethod(helper)
    module.First = First
    module.Second = Second
    module.Inner = Inner
    module.helper = helper
    monkeypatch.setitem(sys.modules, "dartpy", module)

    rows = api_surface.snapshot()
    for path in ("dartpy.First.Inner", "dartpy.Second.Inner", "dartpy.Inner"):
        assert rows[path]["runtime"]["identity_path"] == "dartpy.First.Inner"
    for path in ("dartpy.First.helper", "dartpy.Second.other", "dartpy.helper"):
        assert rows[path]["runtime"]["identity_path"] == "dartpy.First.helper"
    assert rows == api_surface.snapshot()


def test_api_snapshot_prefers_walked_defining_owner_of_shared_objects(
    api_surface, monkeypatch
):
    module = types.ModuleType("dartpy")
    module.__version__ = "test"

    def method():
        """() -> None"""

    class BFirst:
        pass

    class ZBase:
        pass

    class ADerived(ZBase):
        pass

    for cls in (BFirst, ZBase, ADerived):
        cls.__module__ = "dartpy"
    # An inherited row visited first must not claim a shared object before the
    # first walked class that defines it.
    BFirst.method = staticmethod(method)
    ZBase.method = staticmethod(method)
    module.ADerived = ADerived
    module.BFirst = BFirst
    module.ZBase = ZBase
    monkeypatch.setitem(sys.modules, "dartpy", module)

    rows = api_surface.snapshot()
    for path in ("dartpy.ADerived.method", "dartpy.BFirst.method", "dartpy.ZBase.method"):
        assert rows[path]["runtime"]["identity_path"] == "dartpy.BFirst.method"


def test_api_snapshot_keeps_hexadecimal_text_in_signatures(api_surface, monkeypatch):
    module = types.ModuleType("dartpy")
    module.__version__ = "test"

    def operation():
        """(tag: str = '0xdead', note: str = 'look at 0xbeef', text: str = '<object object at 0xbeef>', option: object = <object object at 0x7f00beef>, callback: object = <function callback at 0x7f00>, local: object = <f.<locals>.Default object at 0x7f01>) -> None"""

    module.operation = operation
    monkeypatch.setitem(sys.modules, "dartpy", module)

    assert api_surface.snapshot()["dartpy.operation"]["signatures"] == [
        "(tag: str = '0xdead', note: str = 'look at 0xbeef', "
        "text: str = '<object object at 0xbeef>', "
        "option: object = <object object at ADDRESS>, "
        "callback: object = <function callback at ADDRESS>, "
        "local: object = <f.<locals>.Default object at ADDRESS>) -> None"
    ]


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
