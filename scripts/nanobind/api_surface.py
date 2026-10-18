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
    return [re.sub(r"0x[0-9a-fA-F]+", "ADDRESS", sig) for sig in result]


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
        result = {
            "identity_path": None,
            "module": getattr(value, "__module__", None),
            "qualname": getattr(value, "__qualname__", None),
        }
        if isinstance(value, types.ModuleType):
            result["name"] = value.__name__
        # Keep objects alive, and resolve owners after all classes have been walked.
        pending_runtime.append((result, value, path, owner, name))
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

    walk(dartpy, "dartpy")
    # Defining owners take precedence over inherited rows and module re-exports.
    # Members stay under the first walked class path to preserve comparison keys;
    # a nested class reached through an alias may lack rows at <identity_path>.member.
    for _, value, _, owner, name in pending_runtime:
        if owner is not None and id(owner) in seen:
            member = getattr(owner, name)
            owner_path = seen[id(owner)] + "." + name
            identities.setdefault(identity(member), (owner_path, member))
            identities.setdefault(identity(value), (owner_path, value))
    for metadata, value, path, owner, name in pending_runtime:
        if owner is not None:
            if id(owner) not in seen:
                continue
            metadata["identity_path"] = seen[id(owner)] + "." + name
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
    parser.add_argument("--pb", type=Path)
    parser.add_argument("--nb", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    self_check()
    if args.self_check:
        return
    if args.snapshot:
        print(json.dumps(snapshot(), indent=2))
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
