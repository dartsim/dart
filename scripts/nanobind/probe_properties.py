"""Exercise every default-constructible properties/options/state getter headlessly.

Compare success and value kinds, not uninitialized native numerical contents.
Exceptions are recorded by outcome, preserving the accepted backend difference.
"""

import inspect
import json
import types

import dartpy
import numpy as np


def value_kind(value):
    if isinstance(value, np.ndarray):
        return {
            "kind": "ndarray",
            "shape": list(value.shape),
            "dtype": str(value.dtype),
        }
    if isinstance(value, (list, tuple)):
        return {"kind": type(value).__name__, "length": len(value)}
    if value is None:
        return {"kind": "None"}
    return {"kind": type(value).__module__ + "." + type(value).__name__}


assert value_kind(np.eye(2))["shape"] == [2, 2]
rows = {}
seen = set()
modules = [dartpy]
while modules:
    module = modules.pop()
    for name, value in vars(module).items():
        if name.startswith("_") or id(value) in seen:
            continue
        seen.add(id(value))
        if isinstance(value, types.ModuleType) and value.__name__.startswith("dartpy"):
            modules.append(value)
        elif inspect.isclass(value) and any(
            word in name for word in ("Properties", "Options", "State")
        ):
            key = value.__module__ + "." + value.__name__
            try:
                instance = value()
            except Exception:
                rows[key] = {"construction": "unsupported"}
                continue
            fields = {}
            for field in dir(value):
                if field.startswith("_") or not isinstance(
                    getattr(value, field), property
                ):
                    continue
                try:
                    fields[field] = {
                        "outcome": "success",
                        **value_kind(getattr(instance, field)),
                    }
                except Exception:
                    fields[field] = {"outcome": "exception"}
            rows[key] = {"construction": "success", "fields": fields}
print(json.dumps(rows, indent=2, sort_keys=True))
