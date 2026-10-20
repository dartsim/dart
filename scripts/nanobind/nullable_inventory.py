"""Count runtime nullable annotations, including macro expansions and typed setters."""

import json
import re
import types

import dartpy


def inventory():
    objects = {}
    modules = [dartpy]
    seen_modules = set()
    seen_classes = set()
    frame_types = set()

    def visit_class(cls):
        if id(cls) in seen_classes:
            return
        seen_classes.add(id(cls))
        if issubclass(cls, dartpy.dynamics.Frame):
            frame_types.add(f"{cls.__module__}.{cls.__name__}")
        for key, member in vars(cls).items():
            objects.setdefault(
                id(member), (f"{cls.__module__}.{cls.__qualname__}.{key}", member)
            )
            if isinstance(member, type) and member.__module__.startswith("dartpy"):
                visit_class(member)

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
                visit_class(value)
            else:
                objects.setdefault(id(value), (f"{module.__name__}.{name}", value))
    setters = []
    arguments = []
    frame_parameters = []
    for path, value in objects.values():
        if isinstance(value, property):
            signatures = getattr(value.fset, "__nb_signature__", ())
            if any("value:" in s[0] and " | None" in s[0] for s in signatures):
                setters.append({"path": path, "signatures": [s[0] for s in signatures]})
        else:
            if isinstance(value, (staticmethod, classmethod)):
                value = value.__func__
            if path.rsplit(".", 1)[-1].startswith("_") and not path.endswith(
                ".__init__"
            ):
                continue
            signatures = getattr(value, "__nb_signature__", ())
        for signature in signatures:
            definition = signature[0]
            args = definition.partition(" -> ")[0]
            for parameter, cpp_type in re.findall(r"(\w+): ([\w.]+) \| None", args):
                if cpp_type in frame_types:
                    frame_parameters.append(
                        {
                            "path": path,
                            "signature": definition,
                            "parameter": parameter,
                            "type": cpp_type,
                        }
                    )
            if " | None" in args and not isinstance(value, property):
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
        "nullable_frame_parameter_count": len(frame_parameters),
        "count_method": "Own class dictionaries and module functions, deduplicated by Python object identity; includes overload and macro expansions, excludes private factories.",
        "setters": sorted(setters, key=lambda row: row["path"]),
        "arguments": sorted(arguments, key=lambda row: (row["path"], row["signature"])),
        "frame_parameters": sorted(
            frame_parameters,
            key=lambda row: (row["path"], row["signature"], row["parameter"]),
        ),
    }


if __name__ == "__main__":
    print(json.dumps(inventory(), indent=2))
