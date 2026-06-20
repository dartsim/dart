#!/usr/bin/env python3
"""Check the DART 7 freeze on DART 6 legacy public surfaces."""

from __future__ import annotations

import argparse
import re
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASELINE = REPO_ROOT / "scripts" / "check_dart7_legacy_freeze_baseline.txt"
FREEZE_TAG = "dart7-legacy-freeze: bugfix-port"

LEGACY_CPP_ROOTS = (
    Path("dart/dynamics"),
    Path("dart/constraint"),
)
LEGACY_BINDING_ROOTS = (
    Path("python/dartpy/dynamics"),
    Path("python/dartpy/constraint"),
)
LEGACY_STUBS = (
    Path("python/stubs/dartpy/__init__.pyi"),
    Path("python/stubs/dartpy/dynamics.pyi"),
    Path("python/stubs/dartpy/constraint.pyi"),
)

CPP_SUFFIXES = {".h", ".hpp"}
BINDING_SUFFIXES = {".cpp", ".hpp", ".h"}

CPP_TYPE_PATTERN = re.compile(
    r"^\s*(?:template\s*<[^>]+>\s*)?"
    r"(?P<kind>class|struct|enum(?:\s+class)?|using|typedef)\s+"
    r"(?:(?:DART|DARTPY)_[A-Z0-9_()\".,\s]+\s+)*"
    r"(?P<name>[A-Za-z_]\w*)"
)
CPP_DART_API_FUNCTION_PATTERN = re.compile(
    r"^\s*(?:(?:DART|DARTPY)_[A-Z0-9_()\".,\s]+\s+)+"
    r"(?:static\s+)?(?:inline\s+)?(?:[A-Za-z_:<>~*&,\s]+)\s+"
    r"(?P<name>[A-Za-z_]\w*)\s*\([^;{}]*\)\s*(?:const\s*)?(?:noexcept\s*)?;"
)
CPP_ACCESS_PATTERN = re.compile(r"^\s*(?P<access>public|protected|private)\s*:\s*$")
CPP_NAMESPACE_PATTERN = re.compile(
    r"^\s*namespace\s+(?P<name>[A-Za-z_:]\w*(?:::\w+)*)\s*\{"
)
CPP_MEMBER_FUNCTION_PATTERN = re.compile(
    r"(?:^|[\s*&:<>,])(?P<name>operator\s*[^\s(]+|~?[A-Za-z_]\w*)\s*\("
)
CPP_MEMBER_DATA_PATTERN = re.compile(
    r"(?P<name>[A-Za-z_]\w*)\s*(?:\[[^\]]*\])?(?:\s*=\s*[^;]+)?\s*;"
)
CPP_ENUM_VALUE_PATTERN = re.compile(r"^\s*(?P<name>[A-Za-z_]\w*)\b")
BINDING_CLASS_PATTERN = re.compile(
    r"\bnb::class_<[^;]*\(\s*[^,]+,\s*\"(?P<name>[^\"]+)\""
)
BINDING_ENUM_PATTERN = re.compile(
    r"\bnb::enum_<[^;]*\(\s*[^,]+,\s*\"(?P<name>[^\"]+)\""
)
BINDING_ENUM_VALUE_PATTERN = re.compile(r"\.value\(\s*\"(?P<name>[^\"]+)\"")
BINDING_MODULE_ATTR_PATTERN = re.compile(
    r"\b(?:m|module)\.attr\(\s*\"(?P<name>[^\"]+)\""
)
BINDING_MODULE_DEF_PATTERN = re.compile(r"\b(?:m|module)\.def\(\s*\"(?P<name>[^\"]+)\"")
BINDING_MEMBER_PATTERN = re.compile(
    r"\.(?P<kind>def(?:_[A-Za-z_]+)?|def_property(?:_[A-Za-z_]+)?|def_readwrite|def_readonly)"
    r"\(\s*\"(?P<name>[^\"]+)\""
)
STUB_CLASS_PATTERN = re.compile(r"^(?P<indent>\s*)class\s+(?P<name>[A-Za-z_]\w*)\b")
STUB_DEF_PATTERN = re.compile(r"^(?P<indent>\s*)def\s+(?P<name>[A-Za-z_]\w*)\b")
STUB_ATTR_PATTERN = re.compile(r"^(?P<indent>\s*)(?P<name>[A-Za-z_]\w*)\s*:")
STUB_ALIAS_PATTERN = re.compile(
    r"^(?P<indent>\s*)(?P<name>[A-Za-z_]\w*)\s*=\s*(?P<target>[A-Za-z_]\w*)\b"
)
STUB_LEGACY_IMPORT_PATTERN = re.compile(
    r"^from \.(?P<module>dynamics|constraint) import"
)
STUB_IMPORT_NAME_PATTERN = re.compile(r"^\s*(?P<name>[A-Za-z_]\w*)\s*,?\s*$")


@dataclass(frozen=True, order=True)
class LegacyEntry:
    kind: str
    path: str
    name: str
    tagged: bool = False

    @property
    def baseline_line(self) -> str:
        return f"{self.kind}|{self.path}|{self.name}"


@dataclass(frozen=True)
class Violation:
    message: str


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument(
        "--update-baseline",
        action="store_true",
        help="Rewrite the legacy freeze baseline from the current repository.",
    )
    return parser.parse_args(argv)


def read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8")
    except OSError:
        return None


def relative_path(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def nearby_tag(lines: list[str], line_index: int) -> bool:
    return FREEZE_TAG in lines[line_index] or (
        line_index > 0 and FREEZE_TAG in lines[line_index - 1]
    )


def iter_files(root: Path, rel_root: Path, suffixes: set[str]) -> list[Path]:
    path = root / rel_root
    if not path.exists():
        return []
    return sorted(
        child
        for child in path.rglob("*")
        if child.is_file()
        and child.suffix in suffixes
        and "/detail/" not in child.relative_to(root).as_posix()
    )


def code_without_comment(line: str) -> str:
    return line.split("//", 1)[0]


def brace_delta(line: str) -> int:
    code = code_without_comment(line)
    return code.count("{") - code.count("}")


def add_cpp_enum_values(
    entries: list[LegacyEntry],
    rel_path: str,
    enum_name: str,
    line: str,
    lines: list[str],
    index: int,
) -> None:
    code = code_without_comment(line)
    if "{" in code:
        code = code.split("{", 1)[1]
    if "}" in code:
        code = code.split("}", 1)[0]
    for candidate in code.split(","):
        match = CPP_ENUM_VALUE_PATTERN.search(candidate)
        if match:
            entries.append(
                LegacyEntry(
                    "cpp-enum-value",
                    rel_path,
                    f"{enum_name}.{match.group('name')}",
                    nearby_tag(lines, index),
                )
            )


def member_function_name(signature: str) -> str | None:
    for match in CPP_MEMBER_FUNCTION_PATTERN.finditer(signature):
        name = re.sub(r"\s+", "", match.group("name"))
        if name in {
            "if",
            "for",
            "while",
            "switch",
            "return",
            "static_cast",
            "const_cast",
            "dynamic_cast",
            "reinterpret_cast",
        }:
            continue
        return name
    return None


def member_data_name(signature: str) -> str | None:
    if "(" in signature or signature.startswith(("friend ", "using ", "typedef ")):
        return None
    match = CPP_MEMBER_DATA_PATTERN.search(signature)
    return match.group("name") if match else None


def add_public_cpp_member_entry(
    entries: list[LegacyEntry],
    rel_path: str,
    class_name: str,
    signature: str,
    lines: list[str],
    line_index: int,
) -> None:
    stripped = signature.strip()
    if not stripped or stripped.startswith(("#", "//", "/*", "*")):
        return

    function_name = member_function_name(stripped)
    if function_name:
        entries.append(
            LegacyEntry(
                "cpp-member-function",
                rel_path,
                f"{class_name}.{function_name}",
                nearby_tag(lines, line_index),
            )
        )
        return

    data_name = member_data_name(stripped)
    if data_name:
        entries.append(
            LegacyEntry(
                "cpp-member-data",
                rel_path,
                f"{class_name}.{data_name}",
                nearby_tag(lines, line_index),
            )
        )


def collect_cpp_entries(root: Path) -> list[LegacyEntry]:
    entries: list[LegacyEntry] = []
    for rel_root in LEGACY_CPP_ROOTS:
        for path in iter_files(root, rel_root, CPP_SUFFIXES):
            rel_path = relative_path(path, root)
            lines = path.read_text(encoding="utf-8").splitlines()
            brace_depth = 0
            namespace_stack: list[tuple[str, int]] = []
            class_stack: list[dict[str, object]] = []
            enum_stack: list[tuple[str, int]] = []
            member_buffer: list[str] = []
            member_start = 0
            for index, line in enumerate(lines):
                stripped = line.strip()
                if not stripped or stripped.startswith(("//", "*", "/*", "#")):
                    brace_depth += brace_delta(line)
                    continue

                while namespace_stack and brace_depth <= namespace_stack[-1][1]:
                    namespace_stack.pop()
                while class_stack and brace_depth <= int(class_stack[-1]["exit_depth"]):
                    class_stack.pop()
                while enum_stack and brace_depth <= enum_stack[-1][1]:
                    enum_stack.pop()

                namespace_match = CPP_NAMESPACE_PATTERN.search(line)
                if namespace_match and "{" in line:
                    namespace_stack.append((namespace_match.group("name"), brace_depth))

                in_detail_namespace = any(
                    namespace.endswith("detail")
                    or namespace == "detail"
                    or "::detail" in namespace
                    for namespace, _ in namespace_stack
                )

                if enum_stack and not in_detail_namespace:
                    add_cpp_enum_values(
                        entries, rel_path, enum_stack[-1][0], line, lines, index
                    )

                type_match = CPP_TYPE_PATTERN.search(line)
                if type_match and not in_detail_namespace:
                    kind = type_match.group("kind").replace(" ", "-")
                    type_name = type_match.group("name")
                    entries.append(
                        LegacyEntry(
                            "cpp-" + kind,
                            rel_path,
                            type_name,
                            nearby_tag(lines, index),
                        )
                    )
                    if "{" in line and kind in {"class", "struct"}:
                        class_stack.append(
                            {
                                "name": type_name,
                                "exit_depth": brace_depth,
                                "access": "private" if kind == "class" else "public",
                            }
                        )
                    elif "{" in line and kind.startswith("enum"):
                        enum_stack.append((type_name, brace_depth))
                        add_cpp_enum_values(
                            entries, rel_path, type_name, line, lines, index
                        )

                function_match = (
                    None
                    if in_detail_namespace
                    else CPP_DART_API_FUNCTION_PATTERN.search(line)
                )
                if function_match and not class_stack:
                    entries.append(
                        LegacyEntry(
                            "cpp-function",
                            rel_path,
                            function_match.group("name"),
                            nearby_tag(lines, index),
                        )
                    )

                access_match = CPP_ACCESS_PATTERN.search(line)
                if access_match and class_stack:
                    class_stack[-1]["access"] = access_match.group("access")

                current_class = class_stack[-1] if class_stack else None
                if (
                    current_class is not None
                    and current_class["access"] == "public"
                    and not in_detail_namespace
                    and not type_match
                    and not access_match
                ):
                    code = code_without_comment(line).strip()
                    if member_buffer:
                        member_buffer.append(code)
                    elif "(" in code or (
                        code.endswith(";") and not code.startswith("enum ")
                    ):
                        member_buffer = [code]
                        member_start = index

                    if member_buffer and (";" in code or "{" in code):
                        signature = " ".join(member_buffer)
                        add_public_cpp_member_entry(
                            entries,
                            rel_path,
                            str(current_class["name"]),
                            signature,
                            lines,
                            member_start,
                        )
                        member_buffer = []

                brace_depth += brace_delta(line)
    return entries


def collect_binding_entries(root: Path) -> list[LegacyEntry]:
    entries: list[LegacyEntry] = []
    for rel_root in LEGACY_BINDING_ROOTS:
        for path in iter_files(root, rel_root, BINDING_SUFFIXES):
            rel_path = relative_path(path, root)
            lines = path.read_text(encoding="utf-8").splitlines()
            current_class = "module"
            current_enum = ""
            for index, line in enumerate(lines):
                class_match = BINDING_CLASS_PATTERN.search(line)
                if class_match:
                    current_class = class_match.group("name")
                    current_enum = ""
                    entries.append(
                        LegacyEntry(
                            "binding-class",
                            rel_path,
                            current_class,
                            nearby_tag(lines, index),
                        )
                    )

                enum_match = BINDING_ENUM_PATTERN.search(line)
                if enum_match:
                    current_enum = enum_match.group("name")
                    entries.append(
                        LegacyEntry(
                            "binding-enum",
                            rel_path,
                            current_enum,
                            nearby_tag(lines, index),
                        )
                    )

                enum_value_match = BINDING_ENUM_VALUE_PATTERN.search(line)
                if enum_value_match and current_enum:
                    entries.append(
                        LegacyEntry(
                            "binding-enum-value",
                            rel_path,
                            f"{current_enum}.{enum_value_match.group('name')}",
                            nearby_tag(lines, index),
                        )
                    )

                module_attr_match = BINDING_MODULE_ATTR_PATTERN.search(line)
                if module_attr_match:
                    entries.append(
                        LegacyEntry(
                            "binding-attr",
                            rel_path,
                            f"module.{module_attr_match.group('name')}",
                            nearby_tag(lines, index),
                        )
                    )

                module_def_match = BINDING_MODULE_DEF_PATTERN.search(line)
                if module_def_match:
                    entries.append(
                        LegacyEntry(
                            "binding-function",
                            rel_path,
                            f"module.{module_def_match.group('name')}",
                            nearby_tag(lines, index),
                        )
                    )

                member_match = BINDING_MEMBER_PATTERN.search(line)
                if member_match:
                    entries.append(
                        LegacyEntry(
                            "binding-" + member_match.group("kind"),
                            rel_path,
                            f"{current_class}.{member_match.group('name')}",
                            nearby_tag(lines, index),
                        )
                    )
    return entries


def stub_context(stack: list[tuple[int, str]], indent: int) -> str:
    while stack and stack[-1][0] >= indent:
        stack.pop()
    return ".".join(name for _, name in stack) or "module"


def collect_stub_reexports(
    entries: list[LegacyEntry], rel: str, lines: list[str]
) -> None:
    import_module = ""
    for index, line in enumerate(lines):
        import_match = STUB_LEGACY_IMPORT_PATTERN.search(line)
        if import_match:
            import_module = import_match.group("module")
            continue

        if import_module:
            if line.strip() == ")":
                import_module = ""
                continue
            name_match = STUB_IMPORT_NAME_PATTERN.search(line)
            if name_match:
                entries.append(
                    LegacyEntry(
                        "stub-reexport",
                        rel,
                        f"{import_module}.{name_match.group('name')}",
                        nearby_tag(lines, index),
                    )
                )


def collect_stub_entries(root: Path) -> list[LegacyEntry]:
    entries: list[LegacyEntry] = []
    for rel_path in LEGACY_STUBS:
        path = root / rel_path
        if not path.exists():
            continue
        rel = rel_path.as_posix()
        lines = path.read_text(encoding="utf-8").splitlines()
        collect_stub_reexports(entries, rel, lines)
        class_stack: list[tuple[int, str]] = []
        for index, line in enumerate(lines):
            if not line.strip() or line.lstrip().startswith("#"):
                continue

            class_match = STUB_CLASS_PATTERN.search(line)
            if class_match:
                indent = len(class_match.group("indent"))
                context = stub_context(class_stack, indent)
                name = class_match.group("name")
                entries.append(
                    LegacyEntry(
                        "stub-class",
                        rel,
                        f"{context}.{name}",
                        nearby_tag(lines, index),
                    )
                )
                class_stack.append((indent, name))
                continue

            def_match = STUB_DEF_PATTERN.search(line)
            if def_match:
                indent = len(def_match.group("indent"))
                entries.append(
                    LegacyEntry(
                        "stub-def",
                        rel,
                        f"{stub_context(class_stack, indent)}.{def_match.group('name')}",
                        nearby_tag(lines, index),
                    )
                )
                continue

            alias_match = STUB_ALIAS_PATTERN.search(line)
            if alias_match:
                indent = len(alias_match.group("indent"))
                entries.append(
                    LegacyEntry(
                        "stub-alias",
                        rel,
                        f"{stub_context(class_stack, indent)}.{alias_match.group('name')}",
                        nearby_tag(lines, index),
                    )
                )
                continue

            attr_match = STUB_ATTR_PATTERN.search(line)
            if attr_match:
                indent = len(attr_match.group("indent"))
                name = attr_match.group("name")
                if name in {"self", "cls", "args", "kwargs"}:
                    continue
                entries.append(
                    LegacyEntry(
                        "stub-attr",
                        rel,
                        f"{stub_context(class_stack, indent)}.{name}",
                        nearby_tag(lines, index),
                    )
                )
    return entries


def collect_entries(root: Path) -> list[LegacyEntry]:
    return [
        *collect_cpp_entries(root),
        *collect_binding_entries(root),
        *collect_stub_entries(root),
    ]


def untagged_counter(entries: list[LegacyEntry]) -> Counter[str]:
    return Counter(entry.baseline_line for entry in entries if not entry.tagged)


def read_baseline(path: Path) -> Counter[str]:
    text = read_text(path)
    if text is None:
        return Counter()
    return Counter(
        line.strip()
        for line in text.splitlines()
        if line.strip() and not line.startswith("#")
    )


def write_baseline(path: Path, entries: list[LegacyEntry]) -> None:
    lines = [
        "# Generated by scripts/check_dart7_legacy_freeze.py --update-baseline.",
        "# Do not add entries by hand. New legacy public surface must carry",
        f"# `{FREEZE_TAG}` near the declaration, binding, or stub line.",
        "",
    ]
    for line, count in sorted(untagged_counter(entries).items()):
        lines.extend([line] * count)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plan_042_path(root: Path) -> Path:
    return root / "docs" / "plans" / "042-dart7-public-api-and-source-layout.md"


def post_promotion_decision_path(root: Path) -> Path:
    return (
        root
        / "docs"
        / "plans"
        / "042-dart7-public-api-and-source-layout"
        / "post-promotion-source-layout-decision.md"
    )


def api_source_layout_audit_path(root: Path) -> Path:
    return (
        root
        / "docs"
        / "plans"
        / "042-dart7-public-api-and-source-layout"
        / "api-source-layout-audit.md"
    )


def display_path(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def normalized_policy_text(text: str) -> str:
    return re.sub(r"\s+", " ", text.replace("\\*", "*").replace("`", ""))


def require_snippets(
    path: Path, snippets: tuple[str, ...], root: Path
) -> list[Violation]:
    text = read_text(path)
    if text is None:
        return [Violation(f"Could not read {path}")]
    normalized = normalized_policy_text(text)
    return [
        Violation(f"{display_path(path, root)} missing required policy text: {snippet}")
        for snippet in snippets
        if normalized_policy_text(snippet) not in normalized
    ]


def decision_violations(root: Path = REPO_ROOT) -> list[Violation]:
    return [
        *require_snippets(
            plan_042_path(root),
            (
                "DART 6 legacy removal staging",
                "completely remove DART 6 legacy from the DART 7 public contract",
                "DART 6.20+ port lane",
                "release-6.* branches remain",
                FREEZE_TAG,
                "orphaned cylindrical joint constraint",
            ),
            root,
        ),
        *require_snippets(
            post_promotion_decision_path(root),
            (
                "DART 6.20+ port lane",
                "eventual removal from the DART 7 public contract",
                FREEZE_TAG,
            ),
            root,
        ),
        *require_snippets(
            api_source_layout_audit_path(root),
            (
                "quarantine now, eventual removal",
                "DART 6.20+ port lane",
                FREEZE_TAG,
            ),
            root,
        ),
    ]


def freeze_violations(root: Path, baseline: Path) -> list[Violation]:
    baseline_counter = read_baseline(baseline)
    if not baseline_counter:
        return [Violation(f"Legacy freeze baseline is missing or empty: {baseline}")]

    entries = collect_entries(root)
    current_counter = untagged_counter(entries)
    violations: list[Violation] = []
    for line, count in sorted(current_counter.items()):
        allowed = baseline_counter.get(line, 0)
        if count > allowed:
            violations.append(
                Violation(
                    "New untagged DART 6 legacy public surface: "
                    f"{line} (add `{FREEZE_TAG}` only for release-6.* bugfix ports)"
                )
            )
    return violations


def find_violations(
    root: Path = REPO_ROOT, baseline: Path = DEFAULT_BASELINE
) -> list[Violation]:
    return [*decision_violations(root), *freeze_violations(root, baseline)]


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.update_baseline:
        write_baseline(args.baseline, collect_entries(args.repo_root))
        print(f"Updated DART 7 legacy freeze baseline: {args.baseline}")
        return 0

    violations = find_violations(args.repo_root, args.baseline)
    if not violations:
        print("DART 7 legacy freeze check passed.")
        return 0

    print("DART 7 legacy freeze check failed:", file=sys.stderr)
    for violation in violations:
        print(f"- {violation.message}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
