"""Snapshot each backend in a fresh process, then compare public API surfaces."""

import argparse
import enum
import inspect
import json
import os
import re
import subprocess
import sys
import types
from pathlib import Path

from codemod import split

# Quoted defaults stay intact; any other "<... at 0x...>" address is masked.
ADDRESS_PATTERN = re.compile(
    r"""(?P<quoted>'(?:[^'\\]|\\.)*'|"(?:[^"\\]|\\.)*")|(?<= at )0x[0-9a-fA-F]+(?=>)"""
)
BUDGET_PATH = (
    Path(__file__).resolve().parents[2] / "python/tests/unit/bindings/api_budget.json"
)
BUDGET_UPDATE = "pixi run dartpy scripts/nanobind/api_surface.py --update-budget"


def signatures(value):
    native = getattr(value, "__nb_signature__", None)
    if native:
        result = []
        for sig, _, defaults in native:
            for index, default in enumerate(defaults or ()):
                sig = sig.replace("\\=" + str(index), str(default))
                sig = sig.replace("\\" + str(index), repr(default))
            result.append(sig.removeprefix("def "))
    else:
        doc = getattr(value, "__doc__", "") or ""
        result = [
            re.sub(r"^\d+\. ", "", line.strip())
            for line in doc.splitlines()
            if re.match(r"^(?:\d+\. )?(?:\w+)?\(.*\) ->", line.strip())
        ]
    return [
        ADDRESS_PATTERN.sub(lambda match: match["quoted"] or "ADDRESS", sig)
        for sig in result
    ]


def canonical_type(value):
    value = value.replace(" | None", "").strip().strip('"')
    value = value.replace("typing.SupportsInt | typing.SupportsIndex", "int")
    value = value.replace("typing.SupportsFloat | typing.SupportsIndex", "float")
    value = value.replace("typing.SupportsFloat", "float")
    for old, new in (
        ("typing.List", "list"),
        ("typing.Tuple", "tuple"),
        ("typing.Dict", "dict"),
        ("typing.Set", "set"),
        ("collections.abc.Sequence", "list"),
        ("typing.Sequence", "list"),
        ("collections.abc.Mapping", "dict"),
    ):
        value = value.replace(old, new)
    value = value.replace("Eigen::Quaternion<double, 0>", "dartpy.math.Quaternion")
    value = value.replace("typing_extensions.CapsuleType", "types.CapsuleType")
    value = re.sub(
        r"\bdart::(common|math|optimizer|dynamics|collision|constraint|simulation|utils)::([\w:]+)",
        lambda m: "dartpy." + m[1] + "." + m[2].replace("::", "."),
        value,
    )

    def array(match):
        dtype, dims = match.groups()
        dims = [
            d.strip().replace("m", "*").replace("n", "*").replace("-1", "*")
            for d in dims.split(",")
            if d.strip()
        ]
        if len(dims) == 2 and dims[-1] == "1":
            dims.pop()
        return f"array<{dtype},{','.join(dims)}>"

    value = re.sub(
        r'typing\.Annotated\[numpy\.typing\.ArrayLike, numpy\.(\w+),\s*"\[([^\]]+)\]"\]',
        array,
        value,
    )
    value = re.sub(
        r'typing\.Annotated\[numpy\.typing\.NDArray\[numpy\.(\w+)\],\s*"\[([^\]]+)\]"(?:, "flags.writeable")?\]',
        array,
        value,
    )
    value = re.sub(
        r"numpy\.ndarray\[dtype=(\w+), shape=\(([^)]*)\)[^\]]*\]", array, value
    )
    value = value.replace(
        "dartpy.dynamics.ArrowShape.Properties", "dartpy.dynamics.ArrowShapeProperties"
    )
    value = re.sub(
        r"dartpy\.dynamics\.detail\.(\w+Joint(?:2D)?Properties)",
        r"dartpy.dynamics.\1",
        value,
    )
    for old, new in (
        ("TranslationalJoint.Properties", "TranslationalJointProperties"),
        ("Chain.Criteria", "ChainCriteria"),
        ("Linkage.Criteria", "LinkageCriteria"),
    ):
        value = value.replace("dartpy.dynamics." + old, "dartpy.dynamics." + new)
    return value


def shape(signature):
    signature = re.sub(r"0x[0-9a-fA-F]+", "ADDRESS", signature)
    match = re.match(r"(?:\w+)?\((.*)\)\s*->\s*(.*)", signature)
    if not match:
        return {"unparsed": signature}
    args = split(match[1]) if match[1] else []
    result = []
    for arg in args:
        if re.match(r"self(?:\s*:|$)", arg):
            continue
        name = re.split(r"\s*[:=]\s*", arg, maxsplit=1)[0]
        if name == "/":
            continue
        if re.fullmatch(r"arg\d*", name):
            name = "@positional"
        default = arg.split(" = ", 1)[1] if " = " in arg else None
        if default:
            default = re.sub(r"<([\w.]+\.\w+):\s*-?\d+>", r"\1", default)
        declaration = arg.split(" = ", 1)[0]
        typename = declaration.partition(":")[2] if ":" in declaration else ""
        result.append(
            {"name": name, "default": default, "type": canonical_type(typename)}
        )
    return {"arguments": result, "return": canonical_type(match[2])}


def property_types(value):
    getters, setters = signatures(value.fget), signatures(value.fset)
    return {
        "getter_types": [shape(s)["return"] for s in getters],
        "setter_types": [shape(s)["arguments"][-1]["type"] for s in setters],
        "getter_signatures": getters,
        "setter_signatures": setters,
    }


def snapshot():
    import dartpy

    rows = {
        "dartpy.__version__": {"kind": "constant", "value": dartpy.__version__},
        "dartpy.__doc__": {"kind": "constant", "value": dartpy.__doc__},
    }
    seen = {}
    identities = {id(dartpy): ("dartpy", dartpy)}
    pending_runtime = []
    pending_data = []

    def identity(value):
        if inspect.ismethod(value):
            return (id(value.__func__), id(value.__self__))
        return id(value)

    def runtime(value, path, parent, name):
        owner = None
        if inspect.isclass(parent):
            owner = next(
                (cls for cls in parent.__mro__ if name in cls.__dict__), type(parent)
            )
        # A property has no name of its own; describe it by its accessor.
        source = value
        if isinstance(value, property):
            source = value.fget or value.fset or value
        result = {
            "identity_path": None,
            "module": getattr(source, "__module__", None),
            "qualname": getattr(source, "__qualname__", None),
        }
        if isinstance(value, types.ModuleType):
            result["name"] = value.__name__
        # Keep objects alive, and resolve owners after all classes have been walked.
        inherited = owner is not None and owner is not parent
        pending_runtime.append((result, value, path, owner, name, inherited))
        return result

    def walk(value, path):
        if id(value) in seen:
            return
        seen[id(value)] = path
        for name in dir(value):
            if name.startswith("_") and name not in {
                "__init__",
                "__GUIEventHandler__",
                "__str__",
                "__repr__",
                "__mul__",
                "__eq__",
            }:
                continue
            child_path = path + "." + name
            child = getattr(value, name)
            if isinstance(child, types.ModuleType) and child.__name__.startswith(
                "dartpy"
            ):
                rows[child_path] = {
                    "kind": "module",
                    "runtime": runtime(child, child_path, value, name),
                }
                walk(child, child_path)
            elif inspect.isclass(child) and getattr(child, "__module__", "").startswith(
                "dartpy"
            ):
                members = getattr(child, "__members__", None)
                if members is not None:
                    enum = {}
                    for key, member in members.items():
                        enum[key] = enum_number(member)
                    rows[child_path] = {
                        "kind": "enum",
                        "members": enum,
                        "runtime": runtime(child, child_path, value, name),
                    }
                else:
                    rows[child_path] = {
                        "kind": "class",
                        "runtime": runtime(child, child_path, value, name),
                    }
                    walk(child, child_path)
            elif callable(child):
                sig = signatures(child)
                rows[child_path] = {
                    "kind": "callable",
                    "signatures": sig,
                    "normalized": [shape(s) for s in sig],
                    "runtime": runtime(child, child_path, value, name),
                }
            elif isinstance(child, property):
                rows[child_path] = {
                    "kind": "property",
                    "writable": child.fset is not None,
                    **property_types(child),
                    "runtime": runtime(child, child_path, value, name),
                }
            elif hasattr(type(child), "__members__"):
                rows[child_path] = {"kind": "enum_value", "value": enum_number(child)}
            elif isinstance(child, (str, int, float, bool, type(None))):
                rows[child_path] = {"kind": "constant", "value": child}
            else:
                try:
                    rows[child_path] = {
                        "kind": "enum_value",
                        "value": enum_number(child),
                    }
                except (TypeError, ValueError):
                    rows[child_path] = {"kind": type(child).__name__}
            if "runtime" not in rows[child_path] and inspect.isclass(value):
                owner = next(
                    (cls for cls in value.__mro__ if name in cls.__dict__), type(value)
                )
                pending_data.append((rows[child_path], owner, name))

    walk(dartpy, "dartpy")
    # Data identities follow defining names, not interned values or shared enums.
    for row, owner, name in pending_data:
        row["runtime"] = {
            "identity_path": seen[id(owner)] + "." + name if id(owner) in seen else None
        }
    # Defining owners take precedence over inherited rows and module re-exports.
    # Members stay under the first walked class path to preserve comparison keys;
    # a nested class reached through an alias may lack rows at <identity_path>.member.
    # Rows a class defines claim identities in walk order before inherited rows.
    for claim_inherited in (False, True):
        for _, value, _, owner, name, inherited in pending_runtime:
            if inherited != claim_inherited or owner is None or id(owner) not in seen:
                continue
            member = getattr(owner, name)
            owner_path = seen[id(owner)] + "." + name
            identities.setdefault(identity(member), (owner_path, member))
            identities.setdefault(identity(value), (owner_path, value))
    for metadata, value, path, owner, name, _ in pending_runtime:
        if owner is not None:
            if id(owner) not in seen:
                continue
            # The first walked owner wins when unrelated classes share an object.
            metadata["identity_path"] = identities[identity(value)][0]
            continue
        metadata["identity_path"], _ = identities.setdefault(
            identity(value), (path, value)
        )
    enum_values = {}
    for path, row in rows.items():
        if row["kind"] == "enum":
            for name, number in row["members"].items():
                enum_values[path.rsplit(".", 1)[1] + "." + name] = str(number)
    for row in rows.values():
        for signature in row.get("normalized", []):
            for argument in signature.get("arguments", []):
                if argument["default"] in enum_values:
                    argument["default"] = enum_values[argument["default"]]
    return rows


def budget_surface(rows):
    """Group snapshot identities by canonical module and class, retaining names."""
    owners = {"dartpy": ("dartpy", None)}
    surface = {"dartpy": {"functions": set(), "containers": set(), "classes": {}}}
    for path, row in rows.items():
        runtime = row.get("runtime", {})
        if row["kind"] == "module":
            module, cls = runtime["name"], None
        elif row["kind"] in {"class", "enum"}:
            module, cls = runtime["module"], runtime["qualname"]
            if any(part.startswith("_") for part in cls.split(".")):
                continue
        else:
            continue
        owners[path] = (module, cls)
        scope = surface.setdefault(
            module, {"functions": set(), "containers": set(), "classes": {}}
        )
        if cls is not None:
            names = scope["classes"].setdefault(cls, set())
            # Enum machinery varies with Python; only snapshot enum names count.
            names.update(row.get("members", {}))

    # Exported container names count per exported path, not by identity, so an
    # alias in a second module counts in that module. Nested classes stay
    # counted as members of their defining class.
    for path, row in rows.items():
        if row["kind"] not in {"module", "class", "enum"}:
            continue
        parent, _, name = path.rpartition(".")
        if not parent or not name or name.startswith("_") or parent not in owners:
            continue
        parent_module, parent_cls = owners[parent]
        if parent_cls is not None:
            continue
        scope = surface.get(parent_module)
        if scope is not None:
            scope["containers"].add(name)

    seen = set()
    for path, row in rows.items():
        identity = row.get("runtime", {}).get("identity_path", path)
        if identity is None or identity in seen:
            continue
        seen.add(identity)
        parent, _, name = identity.rpartition(".")
        if name.startswith("_") or parent not in owners:
            continue
        module, cls = owners[parent]
        scope = surface[module]
        if cls is not None:
            scope["classes"][cls].add(name)
        elif row["kind"] not in {"module", "class", "enum"}:
            scope["functions"].add(name)

    def counters(names):
        names = sorted(name for name in names if not name.startswith("_"))
        return {
            "public": names,
            "camel_case": [
                name for name in names if re.search(r"[a-z0-9][A-Z]|^m_", name)
            ],
            "get_set": [
                name for name in names if re.match(r"^(?i:get|set)([A-Z_]|$)", name)
            ],
        }

    return {
        module: {
            "functions": counters(scope["functions"]),
            "containers": counters(scope["containers"]),
            "classes": {
                cls: counters(names) for cls, names in sorted(scope["classes"].items())
            },
        }
        for module, scope in sorted(surface.items())
    }


def budget_growth(surface, budget, *, check_stale=False):
    """Describe growth and optionally stale ceilings for present owners."""
    failures = []
    # A build without optional bindings lacks budgeted modules or classes. Their
    # names cannot be told apart from removals in a module's container counts,
    # so those ceilings are checked for staleness only on full-feature builds.
    reduced = any(
        module not in surface
        or any(
            cls not in surface[module].get("classes", {})
            for cls in limits.get("classes", {})
        )
        for module, limits in budget.items()
    )
    for module, scope in surface.items():
        previous = budget.get(module, {})
        owners = [
            (
                module + " (functions)",
                scope.get("functions", {}),
                previous.get("functions", {}),
            ),
            (
                module + " (containers)",
                scope.get("containers", {}),
                previous.get("containers", {}),
            ),
        ]
        previous_classes = previous.get("classes", {})
        owners.extend(
            (module + "." + cls, counters, previous_classes.get(cls, {}))
            for cls, counters in scope.get("classes", {}).items()
        )
        for owner, counters, limits in owners:
            for counter, names in counters.items():
                limit = limits.get(counter, 0)
                if len(names) > limit:
                    shown = ", ".join(sorted(names)[:40])
                    if len(names) > 40:
                        shown += f", ... ({len(names) - 40} more)"
                    failures.append(
                        f"{owner}: {counter} budget {limit} -> {len(names)}; "
                        f"current names (including additions): {shown}"
                    )
                elif (
                    check_stale
                    and len(names) < limit
                    and not (reduced and owner.endswith(" (containers)"))
                ):
                    failures.append(
                        f"{owner}: {counter} budget {limit} exceeds current count "
                        f"{len(names)}; run {BUDGET_UPDATE} to lower it "
                        "(no --allow-growth needed)"
                    )
        if module not in budget:
            module_names = [
                name
                for key in ("functions", "containers")
                for names in scope.get(key, {}).values()
                for name in names
            ]
            if not module_names:
                failures.append(
                    f"{module}: new module; run {BUDGET_UPDATE} --allow-growth "
                    "to add it"
                )
        for cls, counters in scope.get("classes", {}).items():
            if cls not in previous_classes and all(
                len(names) == 0 for names in counters.values()
            ):
                failures.append(
                    f"{module}.{cls}: new class; run {BUDGET_UPDATE} --allow-growth "
                    "to add it"
                )
    return failures


def update_budget(path, surface, allow_growth=False, *, prune=False):
    previous = json.loads(path.read_text()) if path.exists() else {}
    failures = budget_growth(surface, previous)
    if failures and not allow_growth:
        raise ValueError(
            "\n".join(failures)
            + f"\nTo deliberately raise budgets: {BUDGET_UPDATE} --allow-growth"
        )
    if failures:
        print("\n".join(failures))
    budget = {} if prune else previous
    for module, scope in surface.items():
        target = budget.setdefault(module, {"classes": {}})
        if "functions" in scope:
            target["functions"] = {
                key: len(names) for key, names in scope["functions"].items()
            }
        if "containers" in scope:
            target["containers"] = {
                key: len(names) for key, names in scope["containers"].items()
            }
        target.setdefault("classes", {}).update(
            {
                cls: {key: len(names) for key, names in counters.items()}
                for cls, counters in scope.get("classes", {}).items()
            }
        )
    path.write_text(json.dumps(budget, indent=2, sort_keys=True) + "\n")


def compare(pb, nb):
    # Inventory attribution and raw property spelling do not change binder parity.
    pb, nb = (
        {
            path: {
                key: value
                for key, value in row.items()
                if key not in {"runtime", "getter_signatures", "setter_signatures"}
            }
            for path, row in rows.items()
        }
        for rows in (pb, nb)
    )
    diff = {
        key: []
        for key in (
            "missing",
            "extra",
            "kind",
            "enum",
            "constant",
            "overload_order_keywords_defaults",
            "argument_return_types",
            "property_types",
            "type_spelling",
        )
    }
    for path in sorted(pb.keys() | nb.keys()):
        if path not in nb:
            diff["missing"].append(path)
        elif path not in pb:
            diff["extra"].append(path)
        elif pb[path]["kind"] != nb[path]["kind"]:
            diff["kind"].append({"path": path, "pb": pb[path], "nb": nb[path]})
        elif pb[path]["kind"] == "callable":
            left, right = pb[path]["normalized"], nb[path]["normalized"]

            def names_defaults(signatures):
                return [
                    [
                        {k: a[k] for k in ("name", "default")}
                        for a in s.get("arguments", [])
                    ]
                    for s in signatures
                ]

            if names_defaults(left) != names_defaults(right):
                diff["overload_order_keywords_defaults"].append(
                    {"path": path, "pb": left, "nb": right}
                )
            elif left != right:
                diff["argument_return_types"].append(
                    {"path": path, "pb": left, "nb": right}
                )
            elif pb[path]["signatures"] != nb[path]["signatures"]:
                diff["type_spelling"].append(
                    {
                        "path": path,
                        "pb": pb[path]["signatures"],
                        "nb": nb[path]["signatures"],
                    }
                )
        elif pb[path]["kind"] == "property":
            if pb[path] != nb[path]:
                diff["property_types"].append(
                    {"path": path, "pb": pb[path], "nb": nb[path]}
                )
        elif pb[path] != nb[path]:
            category = (
                "enum" if pb[path]["kind"] in {"enum", "enum_value"} else "constant"
            )
            diff[category].append({"path": path, "pb": pb[path], "nb": nb[path]})
    return diff


def enum_number(member):
    """Return an enum member's number; only arithmetic enums support int()."""
    return int(getattr(member, "value", member))


def self_check():
    assert shape("f(self: X, x: int = 4) -> int")["arguments"] == [
        {"name": "x", "default": "4", "type": "int"}
    ]
    assert shape("f(self, x: int = 4) -> int")["arguments"] == [
        {"name": "x", "default": "4", "type": "int"}
    ]
    assert canonical_type(
        'typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]'
    ) == canonical_type("numpy.ndarray[dtype=float64, shape=(3), order='C']")
    assert (
        canonical_type(
            'tuple[typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]]'
        )
        == "tuple[array<float64,3>]"
    )
    assert compare({"X": {"kind": "class"}}, {})["missing"] == ["X"]
    # Non-arithmetic nanobind enums, such as dartpy.gui.CameraMode, reject int().
    Mode = enum.Enum("Mode", {"RGBA": 0, "DEPTH": 1})
    assert enum_number(Mode.DEPTH) == 1
    assert enum_number(3) == 3
    assert shape("(self: X, value: float) -> None")["arguments"] == [
        {"name": "value", "default": None, "type": "float"}
    ]

    def getter(self):
        """(self: X) -> float"""

    def setter(self, value):
        """(self: X, value: float) -> None"""

    assert property_types(property(getter, setter)) == {
        "getter_types": ["float"],
        "setter_types": ["float"],
        "getter_signatures": ["(self: X) -> float"],
        "setter_signatures": ["(self: X, value: float) -> None"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", action="store_true")
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--update-budget", action="store_true")
    parser.add_argument("--allow-growth", action="store_true")
    parser.add_argument("--prune", action="store_true")
    parser.add_argument("--pb", type=Path)
    parser.add_argument("--nb", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.allow_growth and not args.update_budget:
        parser.error("--allow-growth requires --update-budget")
    if args.prune and not args.update_budget:
        parser.error("--prune requires --update-budget")
    self_check()
    if args.self_check:
        return
    if args.snapshot:
        print(json.dumps(snapshot(), indent=2))
        return
    if args.update_budget:
        try:
            update_budget(
                BUDGET_PATH,
                budget_surface(snapshot()),
                args.allow_growth,
                prune=args.prune,
            )
        except ValueError as error:
            parser.exit(1, str(error) + "\n")
        print(f"Updated {BUDGET_PATH.name} ({BUDGET_PATH.stat().st_size} bytes)")
        return
    if not (args.pb and args.nb and args.output):
        parser.error("--pb, --nb and --output are required")
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for label, path in [("pb", args.pb), ("nb", args.nb)]:
        env = dict(os.environ, PYTHONPATH=str(path.resolve()))
        run = subprocess.run(
            [sys.executable, __file__, "--snapshot"],
            env=env,
            capture_output=True,
            text=True,
            check=True,
        )
        rows = json.loads(run.stdout)
        (args.output / f"api-{label}.json").write_text(run.stdout)
        (args.output / f"api-{label}.stderr").write_text(run.stderr)
        results.append(rows)
    diff = compare(*results)
    (args.output / "api-diff.json").write_text(json.dumps(diff, indent=2) + "\n")
    print(json.dumps({key: len(value) for key, value in diff.items()}))


if __name__ == "__main__":
    main()
