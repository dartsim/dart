"""Snapshot each backend in a fresh process, then compare public API surfaces."""

import argparse
import copy
import enum
import importlib
import inspect
import json
import os
import pkgutil
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
NAMING_EXCEPTIONS_PATH = BUDGET_PATH.with_name("api_naming_exceptions.json")
TRACKED_SPECIAL_NAMES = frozenset(
    {"__init__", "__GUIEventHandler__", "__str__", "__repr__", "__mul__", "__eq__"}
)

# Owners registered only when an optional dependency or component is present.
# Reduced builds omit whole groups without failing the ratchet; a partial
# group or any other absent budgeted owner is a staleness failure. Guards:
# - HAVE_BULLET / HAVE_ODE: python/dartpy/collision/module.cpp (from
#   dart/config.hpp via cmake/DARTFindDependencies.cmake)
# - HAVE_SDFORMAT: python/dartpy/utils/SdfParser.cpp (Options and RootJointType)
# - HAVE_OCTOMAP: python/dartpy/dynamics/Shape.cpp (VoxelGridShape)
# - DARTPY_HAS_UTILS_ASSETS: python/dartpy/utils/ResourceRetriever.cpp
#   (defined in python/dartpy/CMakeLists.txt when dart-utils-assets exists)
# - DARTPY_HAS_VIEWER: python/dartpy/module.cpp (defined in
#   python/dartpy/CMakeLists.txt when dart-gui-app exists, which needs
#   IMGUI_HAS_DOCK per dart/gui/CMakeLists.txt)
OPTIONAL_OWNER_GROUPS = (
    {
        "dartpy.collision.BulletCollisionDetector",
        "dartpy.collision.BulletCollisionGroup",
    },
    {
        "dartpy.collision.OdeCollisionDetector",
        "dartpy.collision.OdeCollisionGroup",
    },
    {"dartpy.dynamics.VoxelGridShape"},
    {"dartpy.utils.ModelResourceRetriever"},
    {
        "dartpy.utils.SdfParser.Options",
        "dartpy.utils.SdfParser.RootJointType",
    },
    {"dartpy.viewer"},
)


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


def import_public_subpackages(package):
    """Import every public pure-Python subpackage of ``package``.

    A subpackage is an attribute of its parent only once something imports it,
    so without this the snapshot would depend on what ran before it. Plain
    modules are skipped, which also skips stale extension files in a build tree.
    """
    # iter_modules imports nothing, unlike walk_packages, so private packages
    # stay unimported.
    for info in pkgutil.iter_modules(getattr(package, "__path__", ())):
        if info.ispkg and not info.name.startswith("_"):
            module = importlib.import_module(f"{package.__name__}.{info.name}")
            import_public_subpackages(module)


def snapshot():
    import dartpy

    import_public_subpackages(dartpy)
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
        if (
            inspect.isclass(value)
            and hasattr(value, "__len__")
            and hasattr(value, "__getitem__")
        ):
            result["protocol_methods"] = ["index", "count"]
        # Keep objects alive, and resolve owners after all classes have been walked.
        inherited = owner is not None and owner is not parent
        pending_runtime.append((result, value, path, owner, name, inherited))
        return result

    def walk(value, path):
        if id(value) in seen:
            return
        seen[id(value)] = path
        for name in dir(value):
            if name.startswith("_") and name not in TRACKED_SPECIAL_NAMES:
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
            # Identity stays shared, but unrelated class exports count separately.
            metadata["owner_path"] = seen[id(owner)]
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


def name_counters(names):
    """Sort public names into the budget's counters."""
    names = sorted(
        name
        for name in names
        if not name.startswith("_") or name in TRACKED_SPECIAL_NAMES
    )
    return {
        "public": names,
        "camel_case": [name for name in names if re.search(r"[a-z0-9][A-Z]|^m_", name)],
        "get_set": [
            name for name in names if re.match(r"^(?i:get|set)([A-Z_]|$)", name)
        ],
    }


def visible_class_members(rows):
    """R57 names by canonical class, including inherited rows, without dunders."""
    classes = {}
    owners = {}
    for path, row in rows.items():
        if row["kind"] not in {"class", "enum"} or any(
            part.startswith("_") for part in path.split(".")
        ):
            continue
        runtime = row["runtime"]
        owner = runtime["module"] + "." + runtime["qualname"]
        owners[path] = owner
        members = classes.setdefault(owner, {})
        members.update(
            (name, {"kind": "enum_value"})
            for name in row.get("members", {})
            if not name.startswith("_")
        )
    for path, row in rows.items():
        parent, _, name = path.rpartition(".")
        if parent in owners and not name.startswith("_"):
            classes[owners[parent]][name] = row
    return classes


def cap_failures(classes, caps):
    failures = []
    for owner, cap in sorted(caps.items()):
        if owner not in classes:
            failures.append(f"{owner}: capped class missing from build (cap {cap})")
        elif len(classes[owner]) > cap:
            failures.append(
                f"{owner}: visible public names {len(classes[owner])} exceed cap {cap}"
            )
    return failures


def r2_deferred(owner, name, row):
    """R2 defers GUI and aspect/composite state, including inherited members."""
    if owner.startswith("dartpy.gui."):
        return True
    if owner.startswith("dartpy.viewer."):
        return "dartpy.gui." in json.dumps(row)
    return bool(re.search(r"Aspect|Composite|composite_state", owner + "." + name))


def naming_failures(rows, *, exceptions=(), freeze=False):
    """R57's four checks; every exemption cites a recorded rule exception."""
    allowed = set()
    for entry in exceptions:
        if not all(
            entry.get(key) for key in ("check", "path", "rule", "reason", "exception")
        ):
            raise ValueError(
                "Naming exceptions require check, path, rule, reason and exception"
            )
        allowed.add((entry["check"], entry["path"]))
    failures = []
    nouns = {}

    def report(check, path, reason):
        if (check, path) not in allowed:
            failures.append(f"{path}: naming check {check}: {reason}")

    classes = visible_class_members(rows)
    constructors = {
        row["runtime"]["module"]
        + "."
        + row["runtime"]["qualname"]: rows[path + ".__init__"]
        for path, row in rows.items()
        if row["kind"] == "class" and path + ".__init__" in rows
    }
    protocols = {
        row["runtime"]["module"]
        + "."
        + row["runtime"]["qualname"]: row["runtime"].get("protocol_methods", [])
        for row in rows.values()
        if row["kind"] == "class"
    }
    for owner, members in sorted(classes.items()):
        active = {
            name: row
            for name, row in members.items()
            if not r2_deferred(owner, name, row)
        }
        if freeze:
            camel = name_counters(active)["camel_case"]
            if camel:
                failures.append(
                    f"{owner}: API freeze forbids camelCase names: {', '.join(camel)}"
                )
        if owner in constructors and not r2_deferred(
            owner, "__init__", constructors[owner]
        ):
            active["__init__"] = constructors[owner]
        for name, row in sorted(active.items()):
            path = owner + "." + name
            callable_member = row["kind"] in {"callable", "class", "enum"}
            if callable_member and name.startswith("get_"):
                noun = name[4:]
                if not any(
                    members.get(candidate, {}).get("kind") == "property"
                    for candidate in (noun, "world_" + noun, "relative_" + noun)
                ):
                    report(1, path, "get_ requires a matching property")
            signatures = row.get("normalized", [])
            if callable_member and name.startswith(("compute_", "calc_", "eval_")):
                if any(sig.get("return") not in {None, "None"} for sig in signatures):
                    report(2, path, "compute_/calc_/eval_ cannot return a value")
            if name.startswith("compute_") and (
                not callable_member
                or not signatures
                or any(sig.get("return") != "None" for sig in signatures)
            ):
                report(3, path, "compute_ must be a method returning None")
            for sig in signatures:
                for argument in sig.get("arguments", []):
                    if argument["name"].startswith("compute_"):
                        report(
                            3, path, f"keyword {argument['name']} starts with compute_"
                        )
            if (
                name.startswith("_")
                or name in protocols.get(owner, [])
                or (4, path) in allowed
            ):
                continue
            kinds = nouns.setdefault(name, {True: [], False: []})
            kinds[callable_member].append(path)
    for name, kinds in sorted(nouns.items()):
        if kinds[True] and kinds[False]:
            failures.append(
                f"{name}: naming check 4: attribute on {', '.join(kinds[False])}; "
                f"callable on {', '.join(kinds[True])}"
            )
    return failures


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
            if any(part.startswith("_") for part in path.split(".")):
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
            module, cls = owners[path]
            scope.setdefault("container_owners", {})[name] = (
                module + "." + cls if cls is not None else module
            )

    seen = set()
    for path, row in rows.items():
        identity = row.get("runtime", {}).get("identity_path", path)
        if identity is None:
            continue
        # Attribute inherited members to their defining owner, but count each
        # distinct exported name: aliases share an identity yet export new names.
        # Module-level containers are counted per exported path above. A nested
        # container counts on its defining class once under its own name and
        # once under each name a class exports it as, so a nested alias counts
        # while inherited access adds nothing.
        if row["kind"] in {"module", "class", "enum"}:
            parent, _, name = identity.rpartition(".")
            if parent not in owners:
                continue
            module, cls = owners[parent]
            if cls is None:
                continue
            names = surface[module]["classes"][cls]
            if identity not in seen:
                seen.add(identity)
                if not name.startswith("_"):
                    names.add(name)
            exported_parent, _, exported_name = path.rpartition(".")
            if (
                exported_parent in owners
                and owners[exported_parent][1] is not None
                and not exported_name.startswith("_")
            ):
                names.add(exported_name)
            continue
        exported_parent, _, exported_name = path.rpartition(".")
        if exported_name.startswith("_") and exported_name not in TRACKED_SPECIAL_NAMES:
            continue
        # Module-level names count where they are exported, so a same-name
        # re-export into another module counts there too; class members count
        # on their defining class, so inherited members are not counted twice.
        if exported_parent in owners and owners[exported_parent][1] is None:
            owner_path = exported_parent
        else:
            owner_path = row.get("runtime", {}).get(
                "owner_path", identity.rpartition(".")[0]
            )
        key = (owner_path, exported_name)
        if key in seen:
            continue
        seen.add(key)
        if owner_path not in owners:
            continue
        module, cls = owners[owner_path]
        scope = surface[module]
        if cls is not None:
            scope["classes"][cls].add(exported_name)
        else:
            scope["functions"].add(exported_name)

    return {
        module: {
            "functions": name_counters(scope["functions"]),
            "containers": name_counters(scope["containers"]),
            "container_owners": dict(sorted(scope.get("container_owners", {}).items())),
            "classes": {
                cls: name_counters(names)
                for cls, names in sorted(scope["classes"].items())
            },
        }
        for module, scope in sorted(surface.items())
    }


def _surface_containers(scope):
    raw = scope.get("containers", {})
    if isinstance(raw, dict):
        return sorted(raw.get("public", []))
    return sorted(raw)


def _budgeted_containers(previous):
    raw = previous.get("containers", [])
    if isinstance(raw, dict):
        return []
    return sorted(raw)


def _absent_optional_owners(surface):
    """Only a wholly absent feature group can explain missing exports."""
    present = set(surface) | {
        name + "." + cls
        for name, current in surface.items()
        for cls in current.get("classes", {})
    }
    return {
        owner
        for group in OPTIONAL_OWNER_GROUPS
        if group.isdisjoint(present)
        for owner in group
    }


def _absent_owners(module, scope, surface, budget):
    """Partition budgeted-but-absent owners into optional and unconditional."""
    optional_owners = _absent_optional_owners(surface)
    previous = budget.get(module, {})
    current_classes = scope.get("classes", {})
    optional, unconditional = [], []
    for cls in previous.get("classes", {}):
        if cls not in current_classes:
            owner = module + "." + cls
            (optional if owner in optional_owners else unconditional).append(owner)
    for child in budget:
        if child == module or child in surface:
            continue
        if child.rpartition(".")[0] != module:
            continue
        (optional if child in optional_owners else unconditional).append(child)
    return sorted(optional), sorted(unconditional)


def _container_name_for_owner(owner, module):
    """Map an absent owner to its container name in module, if any."""
    prefix = module + "."
    if not owner.startswith(prefix):
        return None
    rest = owner[len(prefix) :]
    if not rest or "." in rest:
        return None
    return rest


def budget_growth(surface, budget, *, check_stale=False):
    """Describe growth and optionally stale ceilings for present owners."""
    failures = []
    absent_optional = _absent_optional_owners(surface)
    for module, scope in surface.items():
        previous = budget.get(module, {})
        current_containers = _surface_containers(scope)
        budgeted_containers = _budgeted_containers(previous)
        new_names = sorted(set(current_containers) - set(budgeted_containers))
        if new_names:
            failures.append(
                f"{module} (containers): new container names: "
                f"{', '.join(new_names)}; run {BUDGET_UPDATE} "
                "--allow-growth to add them"
            )
        if check_stale:
            optional, unconditional = _absent_owners(module, scope, surface, budget)
            for owner in unconditional:
                kind = "submodule" if owner in budget else "class"
                failures.append(
                    f"{owner}: budgeted {kind} missing from build; "
                    f"run {BUDGET_UPDATE} to lower it (no --allow-growth needed)"
                )
            if "container_owners" in previous:
                absent_names = {
                    name
                    for name, owner in previous["container_owners"].items()
                    if owner in absent_optional
                }
            elif optional and not unconditional:
                absent_names = set(budgeted_containers)
            else:
                absent_names = {
                    _container_name_for_owner(owner, module)
                    for owner in (*optional, *unconditional)
                } - {None}
            missing = sorted(
                set(budgeted_containers) - set(current_containers) - absent_names
            )
            if missing:
                failures.append(
                    f"{module} (containers): budgeted containers missing "
                    f"from build: {', '.join(missing)}; run {BUDGET_UPDATE} "
                    "to lower it (no --allow-growth needed)"
                )
        owners = [
            (
                module + " (functions)",
                scope.get("functions", {}),
                previous.get("functions", {}),
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
                elif check_stale and len(names) < limit:
                    failures.append(
                        f"{owner}: {counter} budget {limit} exceeds current count "
                        f"{len(names)}; run {BUDGET_UPDATE} to lower it "
                        "(no --allow-growth needed)"
                    )
        if module not in budget:
            module_names = []
            for key in ("functions", "containers"):
                raw = scope.get(key, {})
                if isinstance(raw, dict):
                    for names in raw.values():
                        module_names.extend(names)
                else:
                    module_names.extend(raw)
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
    absent_optional = _absent_optional_owners(surface)
    failures = budget_growth(surface, previous)
    if failures and not allow_growth:
        raise ValueError(
            "\n".join(failures)
            + f"\nTo deliberately raise budgets: {BUDGET_UPDATE} --allow-growth"
        )
    if failures:
        print("\n".join(failures))
    # A copy, so removing owners below leaves `previous` intact for the
    # container pass, which needs the absent owners.
    budget = {} if prune else copy.deepcopy(previous)
    if "caps" in previous:
        budget["caps"] = copy.deepcopy(previous["caps"])
    if not prune:
        # Unconditional owners missing from the build were removed: drop them
        # (and any submodules below a removed module) so the next check passes.
        for module, scope in surface.items():
            _, unconditional = _absent_owners(module, scope, surface, previous)
            for owner in unconditional:
                if owner in budget:
                    for key in [
                        key
                        for key in budget
                        if key == owner or key.startswith(owner + ".")
                    ]:
                        del budget[key]
                else:
                    budget.get(module, {}).get("classes", {}).pop(
                        owner[len(module) + 1 :], None
                    )
    for module, scope in surface.items():
        target = budget.setdefault(module, {"classes": {}})
        if "functions" in scope:
            target["functions"] = {
                key: len(names) for key, names in scope["functions"].items()
            }
        if "containers" in scope:
            current = _surface_containers(scope)
            optional, unconditional = _absent_owners(module, scope, surface, previous)
            previous_owners = previous.get(module, {}).get("container_owners")
            retained = {
                name: owner
                for name, owner in (previous_owners or {}).items()
                if owner in absent_optional
            }
            if prune:
                target["containers"] = current
            elif previous_owners is not None:
                target["containers"] = sorted(set(current) | retained.keys())
            else:
                budgeted = _budgeted_containers(previous.get(module, {}))
                if not optional:
                    target["containers"] = current
                elif not unconditional:
                    target["containers"] = sorted(set(current) | set(budgeted))
                else:
                    drop = {
                        _container_name_for_owner(owner, module)
                        for owner in unconditional
                    } - {None}
                    target["containers"] = sorted(set(current) | (set(budgeted) - drop))
            # shortcut: legacy budgets lack alias provenance, refresh on a full build.
            if "container_owners" in scope and (
                prune or previous_owners is not None or not optional
            ):
                target["container_owners"] = {
                    **({} if prune else retained),
                    **scope["container_owners"],
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
    parser.add_argument("--check-budget", action="store_true")
    parser.add_argument("--freeze", action="store_true")
    parser.add_argument("--pb", type=Path)
    parser.add_argument("--nb", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.allow_growth and not args.update_budget:
        parser.error("--allow-growth requires --update-budget")
    if args.prune and not args.update_budget:
        parser.error("--prune requires --update-budget")
    if args.freeze and not args.check_budget:
        parser.error("--freeze requires --check-budget")
    self_check()
    if args.self_check:
        return
    if args.snapshot:
        print(json.dumps(snapshot(), indent=2))
        return
    if args.check_budget:
        rows = snapshot()
        budget = json.loads(BUDGET_PATH.read_text())
        failures = budget_growth(budget_surface(rows), budget, check_stale=True)
        failures += cap_failures(visible_class_members(rows), budget.get("caps", {}))
        failures += naming_failures(
            rows,
            exceptions=json.loads(NAMING_EXCEPTIONS_PATH.read_text()),
            freeze=args.freeze,
        )
        if failures:
            parser.exit(1, "\n".join(failures) + "\n")
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
