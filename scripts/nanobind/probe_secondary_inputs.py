"""Reproduce input conversions through non-polymorphic secondary bases.

Run in a fresh process with either module selected by PYTHONPATH. Attribute
rebindings do not make a derived value usable as a secondary C++ base input.
"""

import json

import dartpy as d


def probe():
    cases = {
        "GradientDescentSolverProperties as UniqueProperties": lambda: d.optimizer.GradientDescentSolverProperties(
            d.optimizer.SolverProperties(),
            d.optimizer.GradientDescentSolverProperties(),
        ),
        "TaskSpaceRegionProperties as UniqueProperties": lambda: d.dynamics.InverseKinematicsTaskSpaceRegionProperties(
            d.dynamics.InverseKinematicsErrorMethodProperties(),
            d.dynamics.InverseKinematicsTaskSpaceRegionProperties(),
        ),
    }
    rows = []
    for name, call in cases.items():
        try:
            value = call()
            rows.append({"case": name, "passes": True, "type": type(value).__name__})
        except TypeError as exc:
            rows.append(
                {
                    "case": name,
                    "passes": False,
                    "exception": type(exc).__name__,
                    "message": str(exc),
                }
            )
    assert len(rows) == 2
    return {"module": d.__file__, "cases": rows}


if __name__ == "__main__":
    result = probe()
    print(json.dumps(result, indent=2))
    assert all(row["passes"] for row in result["cases"])
