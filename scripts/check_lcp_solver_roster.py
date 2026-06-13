#!/usr/bin/env python3
"""Check that the LCP solver roster stays synchronized across public surfaces."""

from __future__ import annotations

import ast
import csv
import math
import re
import shlex
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "tests/common/lcpsolver/lcp_solver_manifest.hpp"
SELECTION_GUIDE_PATH = ROOT / "docs/background/lcp/07_selection-guide.md"
LCP_DEMO_PATH = ROOT / "python/examples/demos/scenes/lcp_physics.py"
BM_LCP_COMPARE_PATH = ROOT / "tests/benchmark/lcpsolver/bm_lcp_compare.cpp"
LCP_DOCS_DIR = ROOT / "docs/background/lcp"
LCP_PROFILE_CSV_PATHS = {
    "standard": ROOT / "docs/background/lcp/figures/performance_profile_standard.csv",
    "boxed": ROOT / "docs/background/lcp/figures/performance_profile_boxed.csv",
    "findex": ROOT
    / "docs/background/lcp/figures/performance_profile_frictionindex.csv",
}
LCP_PROFILE_EVIDENCE_CSV_PATH = (
    ROOT / "docs/background/lcp/figures/performance_profile_evidence.csv"
)
PROFILE_KEY_BY_CATEGORY = {
    "Standard": "standard",
    "Boxed": "boxed",
    "FrictionIndex": "findex",
}
PROBLEM_TYPE_COUNTER_BY_CATEGORY = {
    "Standard": "problem_type_standard",
    "Boxed": "problem_type_boxed",
    "FrictionIndex": "problem_type_friction_index",
}
PROFILE_CATEGORIES = tuple(PROFILE_KEY_BY_CATEGORY)
FORM_SUPPORT_COUNTER_BY_CATEGORY = {
    "Standard": "solver_supports_standard",
    "Boxed": "solver_supports_boxed",
    "FrictionIndex": "solver_supports_friction_index",
}
FORM_SUPPORT_COUNTERS = tuple(FORM_SUPPORT_COUNTER_BY_CATEGORY.values())
PROBLEM_TYPE_COUNTERS = tuple(PROBLEM_TYPE_COUNTER_BY_CATEGORY.values()) + (
    "problem_type_invalid",
)
SOLVER_IDENTITY_SCHEMA_VERSION = 1
SOLVER_IDENTITY_COUNTERS = (
    "solver_identity_schema_version",
    "solver_manifest_index",
)
SOLVER_FAMILY_COUNTER_BY_FAMILY = {
    "Pivoting": "solver_family_pivoting",
    "Projection": "solver_family_projection",
    "Newton": "solver_family_newton",
    "Other": "solver_family_other",
}
SOLVER_FAMILY_COUNTERS = tuple(SOLVER_FAMILY_COUNTER_BY_FAMILY.values())
REQUIRED_EVIDENCE_COLUMNS = (
    "category",
    "solver",
    "problem_size",
    "lcp_dimension",
    "contact_count",
    *SOLVER_IDENTITY_COUNTERS,
    *SOLVER_FAMILY_COUNTERS,
    "time_ns",
    "contract_ok",
    "iterations",
    "residual",
    "complementarity",
    "bound_violation",
    *FORM_SUPPORT_COUNTERS,
    "solver_supports_problem",
    *PROBLEM_TYPE_COUNTERS,
)
DARTPY_BINDING_PATH = ROOT / "python/dartpy/math/lcp.cpp"
DARTPY_MATH_STUB_PATH = ROOT / "python/stubs/dartpy/math.pyi"
DARTPY_INIT_STUB_PATH = ROOT / "python/stubs/dartpy/__init__.pyi"
DOCUMENTED_LCP_PATH_PATTERN = re.compile(
    r"(?P<path>dart/math/lcp/(?:[A-Za-z0-9_]+/)*[A-Za-z0-9_]+\.(?:hpp|cpp)"
    r"|(?:pivoting|projection|newton|other)/(?:[A-Za-z0-9_]+/)*"
    r"[A-Za-z0-9_]+\.(?:hpp|cpp))(?:/cpp)?"
)


@dataclass(frozen=True)
class SolverEntry:
    name: str
    family: str
    standard: bool
    boxed: bool
    findex: bool
    class_name: str


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _display_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _bool(value: str) -> bool:
    if value == "true":
        return True
    if value == "false":
        return False
    raise ValueError(f"unexpected C++ bool literal: {value}")


def parse_cpp_manifest() -> list[SolverEntry]:
    pattern = re.compile(
        r'\{\s*"(?P<name>[^"]+)"\s*,\s*'
        r'"(?P<family>[^"]+)"\s*,\s*'
        r"(?P<standard>true|false)\s*,\s*"
        r"(?P<boxed>true|false)\s*,\s*"
        r"(?P<findex>true|false)\s*,\s*"
        r"&createLcpSolver<dart::math::(?P<class_name>[A-Za-z0-9_]+)>\s*\}",
        re.DOTALL,
    )
    entries = [
        SolverEntry(
            name=match.group("name"),
            family=match.group("family"),
            standard=_bool(match.group("standard")),
            boxed=_bool(match.group("boxed")),
            findex=_bool(match.group("findex")),
            class_name=match.group("class_name"),
        )
        for match in pattern.finditer(_read(MANIFEST_PATH))
    ]
    if not entries:
        raise AssertionError(f"no solver entries parsed from {MANIFEST_PATH}")
    return entries


def parse_documented_manifest_names() -> list[str]:
    text = _read(SELECTION_GUIDE_PATH)
    begin = "<!-- dart-lcp-solver-manifest: begin -->"
    end = "<!-- dart-lcp-solver-manifest: end -->"
    try:
        block = text.split(begin, 1)[1].split(end, 1)[0]
    except IndexError as exc:
        raise AssertionError(
            f"{SELECTION_GUIDE_PATH} is missing the LCP solver manifest block"
        ) from exc

    names: list[str] = []
    for line in block.splitlines():
        if line.startswith("- "):
            match = re.search(r"`([^`]+)`", line)
            if match:
                names.append(match.group(1))
    return names


def _assignment_value(module: ast.Module, name: str) -> ast.AST:
    for node in module.body:
        if isinstance(node, ast.Assign):
            if any(
                isinstance(target, ast.Name) and target.id == name
                for target in node.targets
            ):
                return node.value
        if (
            isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id == name
            and node.value is not None
        ):
            return node.value
    raise AssertionError(f"{LCP_DEMO_PATH} is missing assignment {name}")


def _literal_assignment(module: ast.Module, name: str) -> Any:
    return ast.literal_eval(_assignment_value(module, name))


def _demo_benchmark_filter_union(
    rows: list[dict[str, str]] | tuple[dict[str, str], ...],
) -> str:
    tokens: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for token in str(row["benchmark_filter"]).split("|"):
            if token and token not in seen:
                seen.add(token)
                tokens.append(token)
    return "|".join(tokens)


def _evaluate_demo_expression(node: ast.AST, env: dict[str, Any]) -> Any:
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.JoinedStr):
        parts: list[str] = []
        for value in node.values:
            if isinstance(value, ast.Constant):
                parts.append(str(value.value))
            elif isinstance(value, ast.FormattedValue):
                parts.append(str(_evaluate_demo_expression(value.value, env)))
            else:
                raise AssertionError(
                    f"cannot evaluate demo f-string part: {ast.dump(value)}"
                )
        return "".join(parts)
    if isinstance(node, ast.Tuple):
        values: list[Any] = []
        for element in node.elts:
            if isinstance(element, ast.Starred):
                values.extend(_evaluate_demo_expression(element.value, env))
            else:
                values.append(_evaluate_demo_expression(element, env))
        return tuple(values)
    if isinstance(node, ast.List):
        values = []
        for element in node.elts:
            if isinstance(element, ast.Starred):
                values.extend(_evaluate_demo_expression(element.value, env))
            else:
                values.append(_evaluate_demo_expression(element, env))
        return values
    if isinstance(node, ast.Dict):
        return {
            _evaluate_demo_expression(key, env): _evaluate_demo_expression(value, env)
            for key, value in zip(node.keys, node.values, strict=True)
            if key is not None
        }
    if isinstance(node, ast.Name):
        if node.id in env:
            return env[node.id]
        raise AssertionError(f"cannot evaluate unknown demo name {node.id!r}")
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = _evaluate_demo_expression(node.left, env)
        right = _evaluate_demo_expression(node.right, env)
        if isinstance(left, tuple) and isinstance(right, tuple):
            return left + right
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
        if node.func.id == "tuple" and len(node.args) == 1 and not node.keywords:
            return tuple(_evaluate_demo_expression(node.args[0], env))
        if (
            node.func.id == "_benchmark_filter_union"
            and len(node.args) == 1
            and not node.keywords
        ):
            rows = _evaluate_demo_expression(node.args[0], env)
            return _demo_benchmark_filter_union(rows)
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "values"
        and not node.args
        and not node.keywords
    ):
        value = _evaluate_demo_expression(node.func.value, env)
        if isinstance(value, dict):
            return tuple(value.values())
    raise AssertionError(f"cannot evaluate demo expression: {ast.dump(node)}")


def parse_demo_roster() -> tuple[list[dict[str, Any]], dict[str, str]]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    rows = list(_literal_assignment(module, "_SOLVER_SUPPORT_ROWS"))
    class_names = dict(_literal_assignment(module, "_SOLVER_CLASS_NAMES"))
    return rows, class_names


def parse_demo_profile_evidence_schema() -> dict[str, Any]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    targets = {
        "_PROFILE_CATEGORY_SUPPORT_FIELDS",
        "_PROFILE_EVIDENCE_REQUIRED_SURFACES",
        "_PROFILE_SOLVER_SUPPORT_FIELDS",
        "_PROFILE_CATEGORY_PROBLEM_TYPE_FIELDS",
        "_PROFILE_PROBLEM_TYPE_FIELDS",
        "_SOLVER_IDENTITY_SCHEMA_VERSION",
        "_PROFILE_SOLVER_FAMILY_COUNTER_BY_FAMILY",
        "_PROFILE_SOLVER_FAMILY_COUNTER_FIELDS",
        "_PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS",
    }
    env: dict[str, Any] = {}

    for node in module.body:
        target_name: str | None = None
        value: ast.AST | None = None
        if isinstance(node, ast.Assign):
            target_names = [
                target.id
                for target in node.targets
                if isinstance(target, ast.Name) and target.id in targets
            ]
            if target_names:
                target_name = target_names[0]
                value = node.value
        elif (
            isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id in targets
            and node.value is not None
        ):
            target_name = node.target.id
            value = node.value

        if target_name is not None and value is not None:
            env[target_name] = _evaluate_demo_expression(value, env)

    missing = sorted(targets - set(env))
    if missing:
        raise AssertionError(
            f"{LCP_DEMO_PATH} is missing profile evidence schema assignments: "
            f"{missing}"
        )
    return env


def parse_demo_profile_evidence_required_columns() -> tuple[str, ...]:
    columns = parse_demo_profile_evidence_schema()[
        "_PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS"
    ]
    return tuple(columns)


def parse_demo_profile_evidence_schema_rows() -> list[dict[str, str]]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    return list(
        _literal_assignment(module, "_PERFORMANCE_PROFILE_EVIDENCE_SCHEMA_ROWS")
    )


def parse_demo_command_metadata() -> dict[str, Any]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    targets = {
        "_BENCHMARK_SMOKE_FILTER",
        "_BENCHMARK_COMMAND",
        "_BENCHMARK_PACKET_ROWS",
        "_REPRESENTATIVE_BENCHMARK_FILTER",
        "_REPRESENTATIVE_BENCHMARK_COMMAND",
        "_PERFORMANCE_PROFILE_REFRESH_COMMAND",
        "_PERFORMANCE_PROFILE_SMOKE_COMMAND",
    }
    env: dict[str, Any] = {}
    for node in module.body:
        target_name: str | None = None
        value: ast.AST | None = None
        if isinstance(node, ast.Assign):
            target_names = [
                target.id
                for target in node.targets
                if isinstance(target, ast.Name) and target.id in targets
            ]
            if target_names:
                target_name = target_names[0]
                value = node.value
        elif (
            isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id in targets
            and node.value is not None
        ):
            target_name = node.target.id
            value = node.value

        if target_name is not None and value is not None:
            env[target_name] = _evaluate_demo_expression(value, env)

    missing = sorted(targets - set(env))
    if missing:
        raise AssertionError(
            f"{LCP_DEMO_PATH} is missing demo command assignments: {missing}"
        )
    return env


def _benchmark_filter_base(token: str) -> str | None:
    match = re.search(r"\b(?:BM_LCP_[A-Za-z0-9_]+|BM_Lcp[A-Za-z0-9_]*)\b", token)
    return match.group(0) if match else None


def _split_benchmark_filter_tokens(filter_text: str) -> list[str]:
    return [token for token in filter_text.split("|") if token]


def parse_demo_benchmark_filter_tokens() -> list[str]:
    command_metadata = parse_demo_command_metadata()
    packet_rows = list(command_metadata["_BENCHMARK_PACKET_ROWS"])
    smoke_filter = str(command_metadata["_BENCHMARK_SMOKE_FILTER"])
    profile_smoke_command = str(command_metadata["_PERFORMANCE_PROFILE_SMOKE_COMMAND"])

    tokens = [smoke_filter]
    for row in packet_rows:
        tokens.extend(_split_benchmark_filter_tokens(row["benchmark_filter"]))

    profile_filter_match = re.search(
        r"(?:^|\s)--benchmark-filter\s+(?P<filter>\S+)",
        profile_smoke_command,
    )
    if profile_filter_match is None:
        raise AssertionError(
            "lcp_physics performance profile smoke command is missing "
            "--benchmark-filter"
        )
    tokens.extend(_split_benchmark_filter_tokens(profile_filter_match.group("filter")))
    return tokens


def check_demo_command_metadata() -> None:
    command_metadata = parse_demo_command_metadata()
    benchmark_rows = list(command_metadata["_BENCHMARK_PACKET_ROWS"])
    smoke_filter = str(command_metadata["_BENCHMARK_SMOKE_FILTER"])
    representative_filter = _demo_benchmark_filter_union(benchmark_rows)
    expected_benchmark_command = (
        "pixi run bm lcp_compare -- " f"--benchmark_filter={smoke_filter}"
    )
    expected_representative_command = (
        "pixi run bm lcp_compare -- --benchmark_filter=" f"'{representative_filter}'"
    )
    expected_profile_refresh_command = (
        "pixi run python scripts/lcp_performance_profile.py --run "
        "--cache build/lcp_profile_full.json "
        "--output docs/background/lcp/figures "
        "--benchmark-timeout 900"
    )
    expected_profile_smoke_command = (
        "pixi run python scripts/lcp_performance_profile.py --run "
        "--allow-partial "
        "--benchmark-filter BM_LcpCompare/Standard/Dantzig/12 "
        "--benchmark-min-time 0.01 "
        "--cache build/lcp_profile_smoke.json "
        "--output build/lcp_profile_smoke "
        "--benchmark-timeout 120"
    )
    expected = {
        "_BENCHMARK_COMMAND": expected_benchmark_command,
        "_REPRESENTATIVE_BENCHMARK_FILTER": representative_filter,
        "_REPRESENTATIVE_BENCHMARK_COMMAND": expected_representative_command,
        "_PERFORMANCE_PROFILE_REFRESH_COMMAND": expected_profile_refresh_command,
        "_PERFORMANCE_PROFILE_SMOKE_COMMAND": expected_profile_smoke_command,
    }

    errors: list[str] = []
    for name, expected_value in expected.items():
        actual_value = str(command_metadata[name])
        if actual_value != expected_value:
            errors.append(
                f"{name} is stale: expected {expected_value!r}, got "
                f"{actual_value!r}"
            )

    for name in (
        "_BENCHMARK_COMMAND",
        "_REPRESENTATIVE_BENCHMARK_COMMAND",
        "_PERFORMANCE_PROFILE_REFRESH_COMMAND",
        "_PERFORMANCE_PROFILE_SMOKE_COMMAND",
    ):
        try:
            shlex.split(str(command_metadata[name]))
        except ValueError as exc:
            errors.append(f"{name} is not shell-parseable: {exc}")

    if errors:
        raise AssertionError(
            "lcp_physics demo command metadata is out of sync:\n  - "
            + "\n  - ".join(errors)
        )


def parse_demo_live_packet_rows() -> list[dict[str, str]]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    return list(_literal_assignment(module, "_LIVE_PACKET_ROWS"))


def parse_demo_benchmark_packet_rows() -> list[dict[str, str]]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    return list(_literal_assignment(module, "_BENCHMARK_PACKET_ROWS"))


def parse_demo_performance_profile_rows() -> list[dict[str, str]]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    env = {
        "_PERFORMANCE_PROFILE_EVIDENCE_ARTIFACT": _literal_assignment(
            module, "_PERFORMANCE_PROFILE_EVIDENCE_ARTIFACT"
        )
    }
    rows = _evaluate_demo_expression(
        _assignment_value(module, "_PERFORMANCE_PROFILE_ROWS"),
        env,
    )
    return list(rows)


def parse_demo_standalone_problem_suite_label() -> str:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    return str(_literal_assignment(module, "_STANDALONE_PROBLEM_SUITE_LABEL"))


def parse_demo_function_names() -> set[str]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    return {node.name for node in module.body if isinstance(node, ast.FunctionDef)}


def parse_demo_standalone_problem_case_rows() -> list[dict[str, Any]]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    value = _assignment_value(module, "_STANDALONE_PROBLEM_CASES")
    if not isinstance(value, ast.Tuple):
        raise AssertionError(
            f"{LCP_DEMO_PATH} _STANDALONE_PROBLEM_CASES is not a tuple literal"
        )

    rows: list[dict[str, Any]] = []
    for element in value.elts:
        if (
            not isinstance(element, ast.Call)
            or not isinstance(element.func, ast.Name)
            or element.func.id != "_StandaloneProblemCase"
        ):
            raise AssertionError(
                f"{LCP_DEMO_PATH} _STANDALONE_PROBLEM_CASES contains "
                f"unsupported entry: {ast.dump(element)}"
            )
        keywords = {keyword.arg: keyword.value for keyword in element.keywords}
        required_keywords = (
            "name",
            "label",
            "surface",
            "support_key",
            "challenge",
            "make_problem",
        )
        missing_keywords = [
            keyword for keyword in required_keywords if keyword not in keywords
        ]
        if missing_keywords:
            raise AssertionError(
                f"{LCP_DEMO_PATH} _STANDALONE_PROBLEM_CASES contains an entry "
                f"missing required keywords {missing_keywords}: {ast.dump(element)}"
            )
        make_problem_node = keywords.get("make_problem")
        if not isinstance(make_problem_node, ast.Name):
            raise AssertionError(
                f"{LCP_DEMO_PATH} _STANDALONE_PROBLEM_CASES contains an entry "
                "without a make_problem function reference"
            )
        tolerance_node = keywords.get("tolerance")
        rows.append(
            {
                "name": _evaluate_demo_expression(keywords["name"], {}),
                "label": _evaluate_demo_expression(keywords["label"], {}),
                "surface": _evaluate_demo_expression(keywords["surface"], {}),
                "support_key": _evaluate_demo_expression(keywords["support_key"], {}),
                "challenge": _evaluate_demo_expression(keywords["challenge"], {}),
                "make_problem": make_problem_node.id,
                "tolerance": (
                    _evaluate_demo_expression(tolerance_node, {})
                    if tolerance_node is not None
                    else 1e-4
                ),
            }
        )
    return rows


def parse_demo_standalone_problem_case_names() -> list[str]:
    return [str(row["name"]) for row in parse_demo_standalone_problem_case_rows()]


def parse_demo_representative_requirement_rows() -> list[dict[str, str]]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    env = {
        "_STANDALONE_PROBLEM_SUITE_LABEL": _literal_assignment(
            module, "_STANDALONE_PROBLEM_SUITE_LABEL"
        )
    }
    rows = _evaluate_demo_expression(
        _assignment_value(module, "_REPRESENTATIVE_REQUIREMENT_ROWS"),
        env,
    )
    return list(rows)


def parse_demo_solver_guidance_rows() -> list[dict[str, str]]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    return list(_literal_assignment(module, "_SOLVER_GUIDANCE_ROWS"))


def parse_demo_advanced_solver_parameter_rows() -> list[dict[str, str]]:
    module = ast.parse(_read(LCP_DEMO_PATH), filename=str(LCP_DEMO_PATH))
    function = next(
        (
            node
            for node in module.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "_advanced_solver_parameter_rows"
        ),
        None,
    )
    if function is None:
        raise AssertionError(
            f"{LCP_DEMO_PATH} is missing _advanced_solver_parameter_rows()"
        )

    env: dict[str, Any] = {}
    for statement in function.body:
        if isinstance(statement, ast.Assign):
            if len(statement.targets) == 1 and isinstance(
                statement.targets[0], ast.Name
            ):
                env[statement.targets[0].id] = _evaluate_demo_expression(
                    statement.value, env
                )
            continue

        if isinstance(statement, ast.Return):
            if not isinstance(statement.value, ast.List):
                raise AssertionError(
                    f"{LCP_DEMO_PATH} _advanced_solver_parameter_rows() "
                    "must return a list literal"
                )

            rows: list[dict[str, str]] = []
            for item in statement.value.elts:
                if (
                    not isinstance(item, ast.Call)
                    or not isinstance(item.func, ast.Name)
                    or item.func.id != "make_row"
                    or len(item.args) != 5
                ):
                    raise AssertionError(
                        f"{LCP_DEMO_PATH} _advanced_solver_parameter_rows() "
                        f"contains unsupported row expression: {ast.dump(item)}"
                    )

                parameter_names = _evaluate_demo_expression(item.args[3], env)
                rows.append(
                    {
                        "solver": str(_evaluate_demo_expression(item.args[0], env)),
                        "surface": str(_evaluate_demo_expression(item.args[1], env)),
                        "parameters": ", ".join(str(name) for name in parameter_names),
                        "benchmark_filter": str(
                            _evaluate_demo_expression(item.args[4], env)
                        ),
                    }
                )
            return rows

    raise AssertionError(
        f"{LCP_DEMO_PATH} _advanced_solver_parameter_rows() is missing a return"
    )


def _split_demo_reference_list(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _split_demo_solver_list(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def parse_lcp_compare_benchmark_bases() -> set[str]:
    return set(
        re.findall(
            r"\b(?:BM_LCP_[A-Za-z0-9_]+|BM_Lcp[A-Za-z0-9_]*)\b",
            _read(BM_LCP_COMPARE_PATH),
        )
    )


def check_demo_benchmark_filters() -> None:
    registered_bases = parse_lcp_compare_benchmark_bases()
    if not registered_bases:
        raise AssertionError(f"no benchmark names parsed from {BM_LCP_COMPARE_PATH}")

    demo_bases: list[str] = []
    unknown_tokens: list[str] = []
    for token in parse_demo_benchmark_filter_tokens():
        base = _benchmark_filter_base(token)
        if base is None or not any(
            registered_base.startswith(base) for registered_base in registered_bases
        ):
            unknown_tokens.append(token)
        else:
            demo_bases.append(base)

    if unknown_tokens:
        raise AssertionError(
            "lcp_physics benchmark filters do not match BM_LCP_COMPARE "
            f"benchmarks: {unknown_tokens}"
        )

    uncovered_bases = sorted(
        registered_base
        for registered_base in registered_bases
        if not any(registered_base.startswith(demo_base) for demo_base in demo_bases)
    )
    if uncovered_bases:
        raise AssertionError(
            "BM_LCP_COMPARE registered benchmarks are not covered by "
            f"lcp_physics benchmark filters: {uncovered_bases}"
        )


def check_demo_live_packets() -> None:
    live_rows = parse_demo_live_packet_rows()
    requirement_rows = parse_demo_representative_requirement_rows()
    required_fields = ("packet", "metric", "benchmark")

    errors: list[str] = []
    if not live_rows:
        errors.append("no live packet rows")

    live_packets = [str(row.get("packet", "")) for row in live_rows]
    duplicate_packets = [
        packet
        for index, packet in enumerate(live_packets)
        if packet and packet in live_packets[:index]
    ]
    if duplicate_packets:
        errors.append(f"duplicate live packet rows {duplicate_packets}")

    for index, row in enumerate(live_rows, start=1):
        packet = str(row.get("packet", f"row {index}"))
        missing_fields = [
            field for field in required_fields if not str(row.get(field, "")).strip()
        ]
        if missing_fields:
            errors.append(f"{packet}: missing fields {missing_fields}")

    referenced_live_packets = {
        packet
        for row in requirement_rows
        for packet in _split_demo_reference_list(str(row.get("live_packet", "")))
    }
    unreferenced_live_packets = sorted(
        packet
        for packet in set(live_packets)
        if packet and packet not in referenced_live_packets
    )
    if unreferenced_live_packets:
        errors.append(
            "live packets missing representative requirement coverage "
            f"{unreferenced_live_packets}"
        )

    if errors:
        raise AssertionError(
            "lcp_physics live packet rows are out of sync:\n  - "
            + "\n  - ".join(errors)
        )


def check_demo_representative_requirements() -> None:
    live_rows = parse_demo_live_packet_rows()
    benchmark_rows = parse_demo_benchmark_packet_rows()
    requirement_rows = parse_demo_representative_requirement_rows()
    standalone_problem_names = set(parse_demo_standalone_problem_case_names())
    standalone_suite_label = parse_demo_standalone_problem_suite_label()

    live_packets = [row["packet"] for row in live_rows]
    benchmark_packets = [row["packet"] for row in benchmark_rows]
    requirements = [row["requirement"] for row in requirement_rows]
    assert_unique(live_packets, "lcp_physics live packet rows")
    assert_unique(benchmark_packets, "lcp_physics benchmark packet rows")
    assert_unique(requirements, "lcp_physics representative requirement rows")

    allowed_live_packets = set(live_packets) | {standalone_suite_label}
    allowed_benchmark_packets = set(benchmark_packets) | standalone_problem_names
    required_fields = (
        "requirement",
        "live_packet",
        "benchmark_packet",
        "metrics",
        "evidence",
    )
    errors: list[str] = []
    for index, row in enumerate(requirement_rows, start=1):
        requirement = row.get("requirement", f"row {index}")
        missing_fields = [
            field for field in required_fields if not str(row.get(field, "")).strip()
        ]
        if missing_fields:
            errors.append(f"{requirement}: missing fields {missing_fields}")
            continue

        unknown_live_packets = [
            packet
            for packet in _split_demo_reference_list(row["live_packet"])
            if packet not in allowed_live_packets
        ]
        if unknown_live_packets:
            errors.append(
                f"{requirement}: unknown live packet references "
                f"{unknown_live_packets}"
            )

        unknown_benchmark_packets = [
            packet
            for packet in _split_demo_reference_list(row["benchmark_packet"])
            if packet not in allowed_benchmark_packets
        ]
        if unknown_benchmark_packets:
            errors.append(
                f"{requirement}: unknown benchmark/standalone references "
                f"{unknown_benchmark_packets}"
            )

    if errors:
        raise AssertionError(
            "lcp_physics representative requirement rows are out of sync:\n  - "
            + "\n  - ".join(errors)
        )


def check_demo_standalone_problem_cases() -> None:
    rows = parse_demo_standalone_problem_case_rows()
    suite_label = parse_demo_standalone_problem_suite_label()
    function_names = parse_demo_function_names()
    expected_surfaces = {"standard", "boxed", "findex"}
    required_fields = (
        "name",
        "label",
        "surface",
        "support_key",
        "challenge",
        "make_problem",
        "tolerance",
    )

    errors: list[str] = []
    if not suite_label.strip():
        errors.append("_STANDALONE_PROBLEM_SUITE_LABEL is blank")

    names = [str(row.get("name", "")) for row in rows]
    labels = [str(row.get("label", "")) for row in rows]
    duplicate_names = [
        name for index, name in enumerate(names) if name in names[:index]
    ]
    duplicate_labels = [
        label for index, label in enumerate(labels) if label in labels[:index]
    ]
    if duplicate_names:
        errors.append(f"duplicate standalone problem names {duplicate_names}")
    if duplicate_labels:
        errors.append(f"duplicate standalone problem labels {duplicate_labels}")

    for index, row in enumerate(rows, start=1):
        case = str(row.get("name", f"row {index}"))
        missing_fields = [
            field for field in required_fields if not str(row.get(field, "")).strip()
        ]
        if missing_fields:
            errors.append(f"{case}: missing fields {missing_fields}")

        surface = str(row.get("surface", ""))
        support_key = str(row.get("support_key", ""))
        if surface and surface not in expected_surfaces:
            errors.append(f"{case}: unknown surface {surface!r}")
        if support_key and support_key not in expected_surfaces:
            errors.append(f"{case}: unknown support_key {support_key!r}")
        if surface in expected_surfaces and support_key in expected_surfaces:
            if surface != support_key:
                errors.append(
                    f"{case}: surface {surface!r} does not match "
                    f"support_key {support_key!r}"
                )

        make_problem = str(row.get("make_problem", ""))
        if make_problem and make_problem not in function_names:
            errors.append(f"{case}: unknown make_problem function {make_problem!r}")

        try:
            tolerance = float(row.get("tolerance", "nan"))
        except (TypeError, ValueError):
            tolerance = math.nan
        if not math.isfinite(tolerance) or tolerance <= 0.0:
            errors.append(f"{case}: tolerance must be positive and finite")

    present_surfaces = {str(row.get("surface", "")) for row in rows}
    missing_surfaces = sorted(expected_surfaces - present_surfaces)
    if missing_surfaces:
        errors.append(
            "standalone problem cases are missing representative surfaces "
            f"{missing_surfaces}"
        )

    if errors:
        raise AssertionError(
            "lcp_physics standalone problem cases are out of sync:\n  - "
            + "\n  - ".join(errors)
        )


def check_demo_solver_guidance(manifest: list[SolverEntry]) -> None:
    guidance_rows = parse_demo_solver_guidance_rows()
    manifest_names = [entry.name for entry in manifest]
    manifest_name_set = set(manifest_names)
    required_fields = (
        "family",
        "solvers",
        "best_fit",
        "strength",
        "tradeoff",
        "evidence",
    )

    errors: list[str] = []
    family_labels = [row.get("family", "") for row in guidance_rows]
    duplicate_families = [
        family
        for index, family in enumerate(family_labels)
        if family and family in family_labels[:index]
    ]
    if duplicate_families:
        errors.append(f"duplicate family rows {duplicate_families}")

    guidance_solvers: list[str] = []
    for index, row in enumerate(guidance_rows, start=1):
        family = row.get("family", f"row {index}")
        missing_fields = [
            field for field in required_fields if not str(row.get(field, "")).strip()
        ]
        if missing_fields:
            errors.append(f"{family}: missing fields {missing_fields}")
            continue

        solvers = _split_demo_solver_list(row["solvers"])
        unknown_solvers = [
            solver for solver in solvers if solver not in manifest_name_set
        ]
        if unknown_solvers:
            errors.append(f"{family}: unknown solvers {unknown_solvers}")
        guidance_solvers.extend(solvers)

    duplicate_solvers = [
        solver
        for index, solver in enumerate(guidance_solvers)
        if solver in guidance_solvers[:index]
    ]
    if duplicate_solvers:
        errors.append(f"duplicate solver guidance entries {duplicate_solvers}")

    missing_solvers = [
        solver for solver in manifest_names if solver not in guidance_solvers
    ]
    if missing_solvers:
        errors.append(f"missing solver guidance entries {missing_solvers}")

    if errors:
        raise AssertionError(
            "lcp_physics solver guidance rows are out of sync:\n  - "
            + "\n  - ".join(errors)
        )


def check_demo_advanced_solver_parameters(manifest: list[SolverEntry]) -> None:
    rows = parse_demo_advanced_solver_parameter_rows()
    manifest_by_name = {entry.name: entry for entry in manifest}
    registered_bases = parse_lcp_compare_benchmark_bases()
    benchmark_packet_rows = parse_demo_benchmark_packet_rows()
    solver_parameter_rows = [
        row
        for row in benchmark_packet_rows
        if row["packet"] == "solver_parameter_sweeps"
    ]
    if len(solver_parameter_rows) != 1:
        raise AssertionError(
            "lcp_physics benchmark packets must contain exactly one "
            "solver_parameter_sweeps row"
        )
    solver_parameter_tokens = set(
        _split_benchmark_filter_tokens(solver_parameter_rows[0]["benchmark_filter"])
    )
    required_fields = ("solver", "surface", "parameters", "benchmark_filter")
    surface_checks = {
        "standard": lambda entry: entry.standard,
        "boxed/findex": lambda entry: entry.boxed and entry.findex,
    }

    errors: list[str] = []
    solvers = [row.get("solver", "") for row in rows]
    duplicate_solvers = [
        solver for index, solver in enumerate(solvers) if solver in solvers[:index]
    ]
    if duplicate_solvers:
        errors.append(f"duplicate solver parameter rows {duplicate_solvers}")

    for index, row in enumerate(rows, start=1):
        solver = row.get("solver", f"row {index}")
        missing_fields = [
            field for field in required_fields if not str(row.get(field, "")).strip()
        ]
        if missing_fields:
            errors.append(f"{solver}: missing fields {missing_fields}")
            continue

        entry = manifest_by_name.get(solver)
        if entry is None:
            errors.append(f"{solver}: unknown solver")
        else:
            surface = row["surface"]
            surface_check = surface_checks.get(surface)
            if surface_check is None:
                errors.append(f"{solver}: unknown surface {surface!r}")
            elif not surface_check(entry):
                errors.append(
                    f"{solver}: surface {surface!r} is not supported by manifest"
                )

        tokens = _split_benchmark_filter_tokens(row["benchmark_filter"])
        if not tokens:
            errors.append(f"{solver}: missing benchmark filter tokens")
            continue
        unknown_tokens = [
            token
            for token in tokens
            if (base := _benchmark_filter_base(token)) is None
            or not any(
                registered_base.startswith(base) for registered_base in registered_bases
            )
        ]
        if unknown_tokens:
            errors.append(f"{solver}: unknown benchmark filters {unknown_tokens}")
        uncovered_tokens = [
            token for token in tokens if token not in solver_parameter_tokens
        ]
        if uncovered_tokens:
            errors.append(
                f"{solver}: benchmark filters missing from solver_parameter_sweeps "
                f"packet {uncovered_tokens}"
            )

    if errors:
        raise AssertionError(
            "lcp_physics advanced solver parameter rows are out of sync:\n  - "
            + "\n  - ".join(errors)
        )


def _parse_demo_problem_sizes(value: str) -> tuple[list[int], list[str]]:
    sizes: list[int] = []
    invalid: list[str] = []
    for item in _split_demo_reference_list(value):
        try:
            size = int(item)
        except ValueError:
            invalid.append(item)
            continue
        if size <= 0:
            invalid.append(item)
            continue
        sizes.append(size)
    return sizes, invalid


def _performance_profile_evidence_problem_sizes_by_category() -> dict[str, set[int]]:
    problem_sizes: dict[str, set[int]] = {
        category: set() for category in PROFILE_KEY_BY_CATEGORY
    }
    with LCP_PROFILE_EVIDENCE_CSV_PATH.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            category = row.get("category", "")
            if category not in problem_sizes:
                continue
            size = _csv_counter_as_int(row, "problem_size")
            if size is not None and size > 0:
                problem_sizes[category].add(size)
    return problem_sizes


def check_demo_performance_profiles() -> None:
    rows = parse_demo_performance_profile_rows()
    required_fields = (
        "surface",
        "artifact",
        "evidence_artifact",
        "problem_sizes",
        "current_leaders",
        "current_laggards",
        "takeaway",
    )
    expected_artifacts = {
        surface: _display_path(LCP_PROFILE_CSV_PATHS[profile_key])
        for surface, profile_key in PROFILE_KEY_BY_CATEGORY.items()
    }
    expected_evidence_artifact = _display_path(LCP_PROFILE_EVIDENCE_CSV_PATH)
    expected_problem_sizes = _performance_profile_evidence_problem_sizes_by_category()

    errors: list[str] = []
    if not rows:
        errors.append("no performance profile rows")

    surfaces = [str(row.get("surface", "")) for row in rows]
    duplicate_surfaces = [
        surface
        for index, surface in enumerate(surfaces)
        if surface and surface in surfaces[:index]
    ]
    if duplicate_surfaces:
        errors.append(f"duplicate performance profile rows {duplicate_surfaces}")

    for index, row in enumerate(rows, start=1):
        surface = str(row.get("surface", f"row {index}"))
        missing_fields = [
            field for field in required_fields if not str(row.get(field, "")).strip()
        ]
        if missing_fields:
            errors.append(f"{surface}: missing fields {missing_fields}")

        if surface not in PROFILE_KEY_BY_CATEGORY:
            errors.append(f"{surface}: unknown profile surface")
            continue

        artifact = str(row.get("artifact", ""))
        expected_artifact = expected_artifacts[surface]
        if artifact != expected_artifact:
            errors.append(
                f"{surface}: artifact {artifact!r} does not match "
                f"{expected_artifact!r}"
            )

        evidence_artifact = str(row.get("evidence_artifact", ""))
        if evidence_artifact != expected_evidence_artifact:
            errors.append(
                f"{surface}: evidence_artifact {evidence_artifact!r} does "
                f"not match {expected_evidence_artifact!r}"
            )

        problem_sizes, invalid_problem_sizes = _parse_demo_problem_sizes(
            str(row.get("problem_sizes", ""))
        )
        if invalid_problem_sizes:
            errors.append(f"{surface}: invalid problem_sizes {invalid_problem_sizes}")
        expected_sizes = sorted(expected_problem_sizes[surface])
        if problem_sizes != expected_sizes:
            errors.append(
                f"{surface}: problem_sizes {problem_sizes} do not match "
                f"evidence {expected_sizes}"
            )

    missing_surfaces = [
        surface for surface in PROFILE_KEY_BY_CATEGORY if surface not in set(surfaces)
    ]
    if missing_surfaces:
        errors.append(f"missing performance profile surfaces {missing_surfaces}")

    if errors:
        raise AssertionError(
            "lcp_physics performance profile rows are out of sync:\n  - "
            + "\n  - ".join(errors)
        )


def check_demo_profile_evidence_required_columns() -> None:
    demo_schema = parse_demo_profile_evidence_schema()
    demo_columns = tuple(demo_schema["_PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS"])
    expected_solver_support_fields = {
        counter: PROFILE_KEY_BY_CATEGORY[category]
        for category, counter in FORM_SUPPORT_COUNTER_BY_CATEGORY.items()
    }
    expected = {
        "_PROFILE_CATEGORY_SUPPORT_FIELDS": FORM_SUPPORT_COUNTER_BY_CATEGORY,
        "_PROFILE_EVIDENCE_REQUIRED_SURFACES": PROFILE_CATEGORIES,
        "_PROFILE_SOLVER_SUPPORT_FIELDS": expected_solver_support_fields,
        "_PROFILE_CATEGORY_PROBLEM_TYPE_FIELDS": PROBLEM_TYPE_COUNTER_BY_CATEGORY,
        "_PROFILE_PROBLEM_TYPE_FIELDS": PROBLEM_TYPE_COUNTERS,
        "_SOLVER_IDENTITY_SCHEMA_VERSION": SOLVER_IDENTITY_SCHEMA_VERSION,
        "_PROFILE_SOLVER_FAMILY_COUNTER_BY_FAMILY": SOLVER_FAMILY_COUNTER_BY_FAMILY,
        "_PROFILE_SOLVER_FAMILY_COUNTER_FIELDS": SOLVER_FAMILY_COUNTERS,
        "_PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS": REQUIRED_EVIDENCE_COLUMNS,
    }
    mismatches = {
        name: {"checker": expected_value, "demo": demo_schema[name]}
        for name, expected_value in expected.items()
        if demo_schema[name] != expected_value
        and name != "_PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS"
    }
    if demo_columns != REQUIRED_EVIDENCE_COLUMNS:
        raise AssertionError(
            "lcp_physics profile evidence required columns do not match the "
            "roster checker.\n"
            f"checker={REQUIRED_EVIDENCE_COLUMNS}\ndemo={demo_columns}"
        )
    if mismatches:
        raise AssertionError(
            "lcp_physics profile evidence schema does not match the roster "
            f"checker: {mismatches}"
        )


def check_demo_profile_evidence_schema_rows() -> None:
    rows = parse_demo_profile_evidence_schema_rows()
    errors: list[str] = []
    if not rows:
        errors.append("no profile evidence schema rows")

    documented_fields: list[str] = []
    for index, row in enumerate(rows, start=1):
        missing_fields = [
            field
            for field in ("fields", "meaning")
            if not str(row.get(field, "")).strip()
        ]
        row_label = str(row.get("fields", f"row {index}")).strip() or f"row {index}"
        if missing_fields:
            errors.append(f"{row_label}: missing fields {missing_fields}")
        for field in _split_demo_reference_list(str(row.get("fields", ""))):
            documented_fields.append(field)

    duplicates = [
        field
        for index, field in enumerate(documented_fields)
        if field in documented_fields[:index]
    ]
    if duplicates:
        errors.append(f"duplicate documented profile evidence fields {duplicates}")

    if documented_fields != list(REQUIRED_EVIDENCE_COLUMNS):
        errors.append(
            "profile evidence schema rows do not match required columns. "
            f"rows={tuple(documented_fields)} required={REQUIRED_EVIDENCE_COLUMNS}"
        )

    if errors:
        raise AssertionError(
            "lcp_physics profile evidence schema rows are out of sync:\n  - "
            + "\n  - ".join(errors)
        )


def parse_bound_solver_classes() -> dict[str, str]:
    pattern = re.compile(
        r"bind(?:Parameterized)?LcpSolverClass"
        r"<\s*(?P<class_name>[A-Za-z0-9_]+)\s*>\s*"
        r'\(\s*m\s*,\s*"(?P<python_name>[A-Za-z0-9_]+)"\s*\)',
        re.DOTALL,
    )
    return {
        match.group("class_name"): match.group("python_name")
        for match in pattern.finditer(_read(DARTPY_BINDING_PATH))
    }


def check_bound_solver_classes(manifest_classes: list[str]) -> None:
    expected_classes = set(manifest_classes)
    binding_names = parse_bound_solver_classes()
    bound_classes = set(binding_names)
    extra_bound_classes = sorted(bound_classes - expected_classes)
    if extra_bound_classes:
        raise AssertionError(
            "dartpy bindings contain non-manifest LCP solver classes: "
            f"{extra_bound_classes}"
        )

    for class_name in manifest_classes:
        bound_name = binding_names.get(class_name)
        if bound_name != class_name:
            raise AssertionError(
                f"dartpy binding mismatch for {class_name}: {bound_name!r}"
            )


def parse_math_stub_solver_classes() -> set[str]:
    pattern = re.compile(r"^class ([A-Za-z0-9_]+)\(LcpSolver\):", re.MULTILINE)
    return set(pattern.findall(_read(DARTPY_MATH_STUB_PATH)))


def _parse_stub(path: Path) -> ast.Module:
    return ast.parse(_read(path), filename=str(path))


def _parse_stub_all_names(path: Path) -> set[str]:
    module = _parse_stub(path)
    display_path = _display_path(path)
    for node in module.body:
        value: ast.expr | None = None
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "__all__"
            for target in node.targets
        ):
            value = node.value
        elif (
            isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id == "__all__"
        ):
            value = node.value

        if value is None:
            continue
        if not isinstance(value, (ast.List, ast.Tuple)):
            raise AssertionError(f"{display_path} __all__ is not a literal list")
        names: set[str] = set()
        for item in value.elts:
            if not isinstance(item, ast.Constant) or not isinstance(item.value, str):
                raise AssertionError(
                    f"{display_path} __all__ contains non-string entries"
                )
            names.add(item.value)
        return names

    raise AssertionError(f"{display_path} is missing __all__")


def parse_math_stub_all_names() -> set[str]:
    return _parse_stub_all_names(DARTPY_MATH_STUB_PATH)


def _parse_init_stub() -> ast.Module:
    return _parse_stub(DARTPY_INIT_STUB_PATH)


def parse_init_stub_math_imports() -> set[str]:
    module = _parse_init_stub()
    return {
        alias.asname or alias.name
        for node in module.body
        if isinstance(node, ast.ImportFrom)
        and node.level == 1
        and node.module == "math"
        for alias in node.names
    }


def parse_init_stub_all_names() -> set[str]:
    return _parse_stub_all_names(DARTPY_INIT_STUB_PATH)


def check_python_stub_solver_classes(manifest_classes: list[str]) -> None:
    expected_classes = set(manifest_classes)
    math_stub_classes = parse_math_stub_solver_classes()
    missing_math_stub = sorted(expected_classes - math_stub_classes)
    if missing_math_stub:
        raise AssertionError(
            "python/stubs/dartpy/math.pyi is missing solver classes: "
            f"{missing_math_stub}"
        )
    extra_math_stub = sorted(math_stub_classes - expected_classes)
    if extra_math_stub:
        raise AssertionError(
            "python/stubs/dartpy/math.pyi contains non-manifest solver classes: "
            f"{extra_math_stub}"
        )

    math_stub_all_names = parse_math_stub_all_names()
    missing_math_all = [
        class_name
        for class_name in manifest_classes
        if class_name not in math_stub_all_names
    ]
    if missing_math_all:
        raise AssertionError(
            "python/stubs/dartpy/math.pyi __all__ is missing solver classes: "
            f"{missing_math_all}"
        )

    init_stub_math_imports = parse_init_stub_math_imports()
    missing_init_imports = [
        class_name
        for class_name in manifest_classes
        if class_name not in init_stub_math_imports
    ]
    if missing_init_imports:
        raise AssertionError(
            "python/stubs/dartpy/__init__.pyi is missing .math imports: "
            f"{missing_init_imports}"
        )

    init_stub_all_names = parse_init_stub_all_names()
    missing_init_all = [
        class_name
        for class_name in manifest_classes
        if class_name not in init_stub_all_names
    ]
    if missing_init_all:
        raise AssertionError(
            "python/stubs/dartpy/__init__.pyi __all__ is missing solver classes: "
            f"{missing_init_all}"
        )


def assert_unique(values: list[str], label: str) -> None:
    seen: set[str] = set()
    duplicates: list[str] = []
    for value in values:
        if value in seen:
            duplicates.append(value)
        seen.add(value)
    if duplicates:
        raise AssertionError(f"{label} contains duplicate entries: {duplicates}")


def _documented_lcp_path_candidates(raw_path: str, token: str) -> list[Path]:
    paths = [raw_path]
    if token.endswith(".hpp/cpp"):
        paths.append(f"{raw_path.removesuffix('.hpp')}.cpp")
    if token.endswith(".cpp/hpp"):
        paths.append(f"{raw_path.removesuffix('.cpp')}.hpp")

    candidates: list[Path] = []
    for path in paths:
        if path.startswith("dart/math/lcp/"):
            candidates.append(ROOT / path)
        else:
            candidates.append(ROOT / "dart/math/lcp" / path)
    return candidates


def check_documented_lcp_paths() -> None:
    missing: list[str] = []
    for doc_path in sorted(LCP_DOCS_DIR.glob("*.md")):
        text = _read(doc_path)
        for match in DOCUMENTED_LCP_PATH_PATTERN.finditer(text):
            token = match.group(0)
            raw_path = match.group("path")
            for candidate in _documented_lcp_path_candidates(raw_path, token):
                if not candidate.is_file():
                    rel_doc = doc_path.relative_to(ROOT)
                    rel_candidate = candidate.relative_to(ROOT)
                    missing.append(f"{rel_doc}: {token} -> {rel_candidate}")

    if missing:
        formatted = "\n".join(f"  - {entry}" for entry in missing)
        raise AssertionError(
            "documented LCP header/source paths do not exist:\n" f"{formatted}"
        )


def _duplicate_values(values: list[str]) -> list[str]:
    duplicates: list[str] = []
    seen: set[str] = set()
    seen_duplicates: set[str] = set()
    for value in values:
        if value in seen and value not in seen_duplicates:
            duplicates.append(value)
            seen_duplicates.add(value)
        seen.add(value)
    return duplicates


def _csv_finite_number(value: str) -> float | None:
    try:
        numeric = float(value)
    except ValueError:
        return None
    if not math.isfinite(numeric):
        return None
    return numeric


def check_performance_profile_headers(manifest: list[SolverEntry]) -> None:
    manifest_by_name = {entry.name: entry for entry in manifest}
    supported_by_profile = {
        "standard": {entry.name for entry in manifest if entry.standard},
        "boxed": {entry.name for entry in manifest if entry.boxed},
        "findex": {entry.name for entry in manifest if entry.findex},
    }
    unsupported_by_profile = {
        "standard": {entry.name for entry in manifest if not entry.standard},
        "boxed": {entry.name for entry in manifest if not entry.boxed},
        "findex": {entry.name for entry in manifest if not entry.findex},
    }

    errors: list[str] = []
    for profile, path in LCP_PROFILE_CSV_PATHS.items():
        with path.open(newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        header = rows[0] if rows else []
        if not header or header[0] != "tau":
            errors.append(f"{_display_path(path)} has invalid header {header!r}")
            continue

        solvers = header[1:]
        duplicate_solvers = _duplicate_values(solvers)
        unknown = sorted(set(solvers) - set(manifest_by_name))
        missing = sorted(supported_by_profile[profile] - set(solvers))
        unsupported = sorted(set(solvers) & unsupported_by_profile[profile])
        if duplicate_solvers:
            errors.append(
                f"{_display_path(path)} contains duplicate solver columns: "
                f"{duplicate_solvers}"
            )
        if unknown:
            errors.append(f"{_display_path(path)} contains unknown solvers: {unknown}")
        if missing:
            errors.append(
                f"{_display_path(path)} is missing native {profile} solvers: "
                f"{missing}"
            )
        if unsupported:
            errors.append(
                f"{_display_path(path)} contains non-native {profile} solvers: "
                f"{unsupported}"
            )
        if len(rows) == 1:
            errors.append(f"{_display_path(path)} has no profile rows")
        previous_tau: float | None = None
        for row_number, row in enumerate(rows[1:], start=2):
            if len(row) != len(header):
                errors.append(
                    f"{_display_path(path)} row {row_number} has {len(row)} "
                    f"columns; expected {len(header)}"
                )
                continue
            tau = _csv_finite_number(row[0])
            if tau is None or tau < 1.0:
                errors.append(
                    f"{_display_path(path)} row {row_number} has invalid tau "
                    f"{row[0]!r}"
                )
            elif previous_tau is not None and tau <= previous_tau:
                errors.append(
                    f"{_display_path(path)} row {row_number} has non-increasing "
                    f"tau {row[0]!r}"
                )
            else:
                previous_tau = tau
            for solver, value_text in zip(solvers, row[1:], strict=True):
                value = _csv_finite_number(value_text)
                if value is None or value < 0.0 or value > 1.0:
                    errors.append(
                        f"{_display_path(path)} row {row_number} has invalid "
                        f"profile value {value_text!r} for {solver}"
                    )

    if errors:
        raise AssertionError(
            "LCP performance profile headers are out of sync with native "
            "solver support:\n  - " + "\n  - ".join(errors)
        )


def _csv_counter_as_int(row: dict[str, str], key: str) -> int | None:
    value = row.get(key, "")
    if value == "":
        return None
    try:
        numeric = float(value)
    except ValueError:
        return None
    if not math.isfinite(numeric):
        return None
    rounded = int(round(numeric))
    if abs(numeric - rounded) > 1e-9:
        return None
    return rounded


def _csv_finite_float(row: dict[str, str], key: str) -> float | None:
    value = row.get(key, "")
    if value == "":
        return None
    try:
        numeric = float(value)
    except ValueError:
        return None
    if not math.isfinite(numeric):
        return None
    return numeric


def _solver_support(entry: SolverEntry, category: str) -> bool:
    if category == "Standard":
        return entry.standard
    if category == "Boxed":
        return entry.boxed
    if category == "FrictionIndex":
        return entry.findex
    raise AssertionError(f"unknown LCP profile category: {category}")


def check_performance_profile_evidence(
    manifest: list[SolverEntry],
    path: Path = LCP_PROFILE_EVIDENCE_CSV_PATH,
) -> None:
    manifest_by_name = {entry.name: entry for entry in manifest}
    manifest_index_by_name = {
        entry.name: index for index, entry in enumerate(manifest, start=1)
    }

    errors: list[str] = []
    observed_native_solvers_by_category: dict[str, set[str]] = {
        category: set() for category in PROFILE_KEY_BY_CATEGORY
    }
    observed_evidence_keys: set[tuple[str, str, int]] = set()
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []
        duplicate_columns = _duplicate_values(header)
        if duplicate_columns:
            raise AssertionError(
                f"{_display_path(path)} contains duplicate evidence columns: "
                f"{duplicate_columns}"
            )
        missing_columns = [
            column for column in REQUIRED_EVIDENCE_COLUMNS if column not in header
        ]
        if missing_columns:
            raise AssertionError(
                f"{_display_path(path)} is missing required columns: {missing_columns}"
            )

        row_count = 0
        for row_number, row in enumerate(reader, start=2):
            row_count += 1
            category = row["category"]
            solver_name = row["solver"]
            entry = manifest_by_name.get(solver_name)

            if category not in PROFILE_KEY_BY_CATEGORY:
                errors.append(f"row {row_number}: unknown category {category!r}")
                continue
            if entry is None:
                errors.append(f"row {row_number}: unknown solver {solver_name!r}")
                continue

            problem_size = _csv_counter_as_int(row, "problem_size")
            if problem_size is None or problem_size <= 0:
                errors.append(
                    f"row {row_number}: invalid problem_size {row['problem_size']!r}"
                )
            else:
                evidence_key = (category, solver_name, problem_size)
                if evidence_key in observed_evidence_keys:
                    errors.append(
                        f"row {row_number}: duplicate evidence row for "
                        f"{category}/{solver_name}/{problem_size}"
                    )
                else:
                    observed_evidence_keys.add(evidence_key)

            lcp_dimension = _csv_counter_as_int(row, "lcp_dimension")
            if lcp_dimension is None or lcp_dimension <= 0:
                errors.append(
                    f"row {row_number}: invalid lcp_dimension "
                    f"{row['lcp_dimension']!r}"
                )
            elif problem_size is not None:
                if category == "FrictionIndex":
                    expected_dimension = 3 * problem_size
                    if lcp_dimension != expected_dimension:
                        errors.append(
                            f"row {row_number}: lcp_dimension "
                            f"{row['lcp_dimension']!r} does not match "
                            f"3 * friction contact count {problem_size}"
                        )
                elif lcp_dimension != problem_size:
                    errors.append(
                        f"row {row_number}: lcp_dimension "
                        f"{row['lcp_dimension']!r} does not match "
                        f"problem_size {row['problem_size']!r}"
                    )

            contact_count = _csv_counter_as_int(row, "contact_count")
            if category == "FrictionIndex":
                if contact_count != problem_size:
                    errors.append(
                        f"row {row_number}: contact_count "
                        f"{row['contact_count']!r} does not match "
                        f"friction profile size {row['problem_size']!r}"
                    )
            elif row["contact_count"] != "" and (
                contact_count is None or contact_count < 0
            ):
                errors.append(
                    f"row {row_number}: invalid contact_count "
                    f"{row['contact_count']!r}"
                )

            time_ns = _csv_finite_float(row, "time_ns")
            if time_ns is None or time_ns <= 0.0:
                errors.append(f"row {row_number}: invalid time_ns {row['time_ns']!r}")

            contract_ok = _csv_counter_as_int(row, "contract_ok")
            if contract_ok != 1:
                errors.append(
                    f"row {row_number}: contract_ok {row['contract_ok']!r} != 1"
                )

            iterations = _csv_counter_as_int(row, "iterations")
            if iterations is None or iterations < 0:
                errors.append(
                    f"row {row_number}: invalid iterations {row['iterations']!r}"
                )

            for metric in ("residual", "complementarity", "bound_violation"):
                value = _csv_finite_float(row, metric)
                if value is None or value < 0.0:
                    errors.append(f"row {row_number}: invalid {metric} {row[metric]!r}")

            identity_version = _csv_counter_as_int(
                row, "solver_identity_schema_version"
            )
            if identity_version != SOLVER_IDENTITY_SCHEMA_VERSION:
                errors.append(
                    f"row {row_number}: solver_identity_schema_version "
                    f"{row['solver_identity_schema_version']!r} != "
                    f"{SOLVER_IDENTITY_SCHEMA_VERSION}"
                )

            manifest_index = _csv_counter_as_int(row, "solver_manifest_index")
            expected_manifest_index = manifest_index_by_name[solver_name]
            if manifest_index != expected_manifest_index:
                errors.append(
                    f"row {row_number}: solver_manifest_index "
                    f"{row['solver_manifest_index']!r} does not match "
                    f"{solver_name} index {expected_manifest_index}"
                )

            expected_family_counter = SOLVER_FAMILY_COUNTER_BY_FAMILY[entry.family]
            family_counter_sum = 0
            for key in SOLVER_FAMILY_COUNTERS:
                expected_value = 1 if key == expected_family_counter else 0
                actual = _csv_counter_as_int(row, key)
                if actual == 1:
                    family_counter_sum += 1
                if actual != expected_value:
                    errors.append(
                        f"row {row_number}: {key} {row[key]!r} != "
                        f"{expected_value} for {solver_name}/{entry.family}"
                    )
            if family_counter_sum != 1:
                errors.append(
                    f"row {row_number}: solver family counters are not one-hot "
                    f"for {solver_name}"
                )

            native_category_supported = _solver_support(entry, category)
            if not native_category_supported:
                errors.append(
                    f"row {row_number}: {solver_name}/{category} is not "
                    "native-supported and should not appear in profile evidence"
                )
            else:
                observed_native_solvers_by_category[category].add(solver_name)

            expected_support = {
                counter: _solver_support(entry, support_category)
                for support_category, counter in FORM_SUPPORT_COUNTER_BY_CATEGORY.items()
            }
            expected_support["solver_supports_problem"] = native_category_supported
            for key, expected in expected_support.items():
                expected_value = 1 if expected else 0
                actual = _csv_counter_as_int(row, key)
                if actual != expected_value:
                    errors.append(
                        f"row {row_number}: {key} {row[key]!r} != "
                        f"{expected_value} for {solver_name}/{category}"
                    )

            expected_problem_type = {key: 0 for key in PROBLEM_TYPE_COUNTERS}
            expected_problem_type[PROBLEM_TYPE_COUNTER_BY_CATEGORY[category]] = 1
            for key, expected_value in expected_problem_type.items():
                actual = _csv_counter_as_int(row, key)
                if actual != expected_value:
                    errors.append(
                        f"row {row_number}: {key} {row[key]!r} != "
                        f"{expected_value} for {category}"
                    )

    if row_count == 0:
        errors.append(f"{_display_path(path)} has no evidence rows")

    for category in PROFILE_KEY_BY_CATEGORY:
        expected_solvers = {
            entry.name for entry in manifest if _solver_support(entry, category)
        }
        missing_solvers = sorted(
            expected_solvers - observed_native_solvers_by_category[category]
        )
        if missing_solvers:
            errors.append(
                f"{_display_path(path)} {category} is missing native profile "
                f"evidence solvers: {missing_solvers}"
            )

    if errors:
        raise AssertionError(
            "LCP performance profile evidence is out of sync:\n  - "
            + "\n  - ".join(errors)
        )


def check_roster() -> None:
    check_documented_lcp_paths()

    manifest = parse_cpp_manifest()
    check_performance_profile_headers(manifest)
    check_performance_profile_evidence(manifest)
    check_demo_command_metadata()
    check_demo_benchmark_filters()
    check_demo_live_packets()
    check_demo_representative_requirements()
    check_demo_standalone_problem_cases()
    check_demo_solver_guidance(manifest)
    check_demo_advanced_solver_parameters(manifest)
    check_demo_performance_profiles()
    check_demo_profile_evidence_required_columns()
    check_demo_profile_evidence_schema_rows()
    manifest_names = [entry.name for entry in manifest]
    manifest_classes = [entry.class_name for entry in manifest]
    manifest_by_name = {entry.name: entry for entry in manifest}

    assert len(manifest) == 24, f"expected 24 LCP solvers, found {len(manifest)}"
    assert_unique(manifest_names, "C++ LCP solver manifest")
    assert_unique(manifest_classes, "C++ LCP solver class names")

    standard_count = sum(1 for entry in manifest if entry.standard)
    boxed_count = sum(1 for entry in manifest if entry.boxed)
    findex_count = sum(1 for entry in manifest if entry.findex)
    if (standard_count, boxed_count, findex_count) != (23, 15, 16):
        raise AssertionError(
            "unexpected LCP support counts: "
            f"standard={standard_count}, boxed={boxed_count}, findex={findex_count}"
        )

    documented_names = parse_documented_manifest_names()
    if documented_names != manifest_names:
        raise AssertionError(
            "docs/background/lcp/07_selection-guide.md solver block does not "
            f"match the C++ manifest.\nmanifest={manifest_names}\ndocs={documented_names}"
        )

    demo_rows, demo_class_names = parse_demo_roster()
    demo_names = [row["name"] for row in demo_rows]
    if demo_names != manifest_names:
        raise AssertionError(
            f"lcp_physics solver rows do not match manifest.\n"
            f"manifest={manifest_names}\ndemo={demo_names}"
        )
    if list(demo_class_names) != manifest_names:
        raise AssertionError(
            f"lcp_physics class-name map keys do not match manifest.\n"
            f"manifest={manifest_names}\nmap={list(demo_class_names)}"
        )
    for row in demo_rows:
        entry = manifest_by_name[row["name"]]
        expected = {
            "family": entry.family,
            "standard": entry.standard,
            "boxed": entry.boxed,
            "findex": entry.findex,
        }
        actual = {key: row[key] for key in expected}
        if actual != expected:
            raise AssertionError(
                f"lcp_physics metadata mismatch for {entry.name}: "
                f"expected {expected}, got {actual}"
            )
        if demo_class_names[entry.name] != entry.class_name:
            raise AssertionError(
                f"lcp_physics class map mismatch for {entry.name}: "
                f"expected {entry.class_name}, got {demo_class_names[entry.name]}"
            )

    check_bound_solver_classes(manifest_classes)
    check_python_stub_solver_classes(manifest_classes)


def main() -> int:
    try:
        check_roster()
    except AssertionError as exc:
        print(f"LCP solver roster check failed: {exc}", file=sys.stderr)
        return 1

    print(
        "LCP solver roster check passed: 24 solvers, 23 standard, 15 boxed, 16 findex."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
