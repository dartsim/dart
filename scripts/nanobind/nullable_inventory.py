"""Count runtime nullable annotations, including macro expansions and typed setters."""

import json
import types

import dartpy


def inventory():
    objects = {}
    modules = [dartpy]
    seen_modules = set()
    while modules:
        module = modules.pop()
        if id(module) in seen_modules:
            continue
        seen_modules.add(id(module))
        for name, value in vars(module).items():
            if name.startswith("_"):
                continue
            if isinstance(value, types.ModuleType) and value.__name__.startswith(
                "dartpy"
            ):
                modules.append(value)
            elif isinstance(value, type):
                for key, member in vars(value).items():
                    objects.setdefault(
                        id(member),
                        (f"{value.__module__}.{value.__name__}.{key}", member),
                    )
            else:
                objects.setdefault(id(value), (f"{module.__name__}.{name}", value))
    setters = []
    arguments = []
    for path, value in objects.values():
        if isinstance(value, property):
            signatures = getattr(value.fset, "__nb_signature__", ())
            if any("value:" in s[0] and " | None" in s[0] for s in signatures):
                setters.append({"path": path, "signatures": [s[0] for s in signatures]})
            continue
        if isinstance(value, (staticmethod, classmethod)):
            value = value.__func__
        if path.rsplit(".", 1)[-1].startswith("_") and not path.endswith(".__init__"):
            continue
        for signature in getattr(value, "__nb_signature__", ()):
            definition = signature[0]
            args = definition.partition(" -> ")[0]
            if " | None" in args:
                arguments.append(
                    {
                        "path": path,
                        "signature": definition,
                        "nullable_parameters": args.count(" | None"),
                    }
                )
    assert setters, "No typed pointer setters detected"
    return {
        "nullable_setter_count": len(setters),
        "nullable_parameter_count": sum(a["nullable_parameters"] for a in arguments),
        "count_method": "Own class dictionaries and module functions, deduplicated by Python object identity; includes overload and macro expansions, excludes private factories.",
        "setters": sorted(setters, key=lambda row: row["path"]),
        "arguments": sorted(arguments, key=lambda row: (row["path"], row["signature"])),
    }


if __name__ == "__main__":
    print(json.dumps(inventory(), indent=2))
