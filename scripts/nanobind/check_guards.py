#!/usr/bin/env python3
"""Check that binding translation units select compatible nanobind casters."""

import argparse
import re
from pathlib import Path

FIRST_HEADER = "detail/dart_nb.hpp"
PROPERTY_HEADERS = {
    "optimizer/Solver.cpp": ("detail/optimizer_properties.hpp",),
    "optimizer/GradientDescentSolver.cpp": ("detail/optimizer_properties.hpp",),
    "dynamics/InverseKinematics.cpp": (
        "detail/ik_properties.hpp",
        "detail/optimizer_properties.hpp",
    ),
    "detail/gc.cpp": (
        "detail/optimizer_properties.hpp",
        "detail/ik_properties.hpp",
    ),
}


def violations(source: str, name: str) -> list[str]:
    includes = re.findall(r'^\s*#\s*include\s*[<"]([^>"\n]+)[>"]', source, re.M)
    errors = []
    if not includes or includes[0] != FIRST_HEADER:
        errors.append(f"first include must be {FIRST_HEADER}")
    for header in includes:
        if header.startswith("nanobind/eigen/") or header in {
            "nanobind/stl/shared_ptr.h",
            "nanobind/stl/array.h",
        }:
            errors.append(f"stock caster conflicts with DART caster: {header}")
    property_headers = set(PROPERTY_HEADERS.get(name, ()))
    if re.search(
        r"(?:Solver|GradientDescentSolver)::\s*(?:Properties|UniqueProperties)", source
    ):
        property_headers.add("detail/optimizer_properties.hpp")
    if re.search(
        r"(?:InverseKinematics|IK)::(?:ErrorMethod|TaskSpaceRegion)::\s*(?:Properties|UniqueProperties)",
        source,
    ):
        property_headers.add("detail/ik_properties.hpp")
    for header in sorted(property_headers):
        if header not in includes:
            errors.append(f"missing property caster selection: {header}")
    if (
        "Eigen::" in source
        or name in {"collision/RaycastResult.cpp", "collision/DistanceResult.cpp"}
    ) and not any(
        header in includes
        for header in (
            "detail/eigen.hpp",
            "eigen.hpp",
            "eigen_pybind.h",
            "eigen_geometry_pybind.h",
        )
    ):
        errors.append("Eigen bindings require detail/eigen.hpp")
    return errors


def self_test() -> None:
    valid = (
        '#include "detail/dart_nb.hpp"\n#include "detail/eigen.hpp"\nEigen::MatrixXd;'
    )
    assert not violations(valid, "probe.cpp")
    assert violations(valid.replace(FIRST_HEADER, "nanobind/nanobind.h"), "probe.cpp")
    assert violations(valid + "\n#include <nanobind/stl/shared_ptr.h>", "probe.cpp")
    assert violations(valid + "\n#include <nanobind/stl/array.h>", "probe.cpp")
    assert violations(valid + "\n#include <nanobind/eigen/dense.h>", "probe.cpp")
    assert violations(valid.replace('#include "detail/eigen.hpp"', ""), "probe.cpp")
    assert violations('#include "detail/dart_nb.hpp"', "collision/RaycastResult.cpp")
    assert not violations(valid, "collision/RaycastResult.cpp")
    assert violations('#include "detail/dart_nb.hpp"', "collision/DistanceResult.cpp")
    assert not violations(valid, "collision/DistanceResult.cpp")
    assert violations(valid, "optimizer/Solver.cpp")
    assert violations(valid + "Solver::Properties;", "new_binding.cpp")
    assert violations(valid + "IK::ErrorMethod::Properties;", "new_binding.cpp")
    assert not violations(
        valid + '\n#include "detail/optimizer_properties.hpp"', "optimizer/Solver.cpp"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", nargs="?", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    if args.root is None:
        if args.self_test:
            return 0
        parser.error("root is required unless --self-test is used")
    failures = []
    for path in sorted(args.root.rglob("*.cpp")):
        name = path.relative_to(args.root).as_posix()
        failures.extend(
            f"{name}: {error}" for error in violations(path.read_text(), name)
        )
    if failures:
        print("\n".join(failures))
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
