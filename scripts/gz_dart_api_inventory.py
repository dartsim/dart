#!/usr/bin/env python3
"""Inventory the DART C++ declarations referenced by Gazebo C++ sources.

This is a lexical compatibility aid, not C++ overload resolution. Unresolved
receivers retain conservative declaration candidates and an explicit count.
"""

import argparse
import bisect
import difflib
import json
import re
import subprocess
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "tools/gazebo/compat/dart_api_inventory.json"
CPP_SUFFIXES = {
    ".h",
    ".hh",
    ".hpp",
    ".hxx",
    ".c",
    ".cc",
    ".cpp",
    ".cxx",
    ".ipp",
    ".tpp",
}
IDENTIFIER = re.compile(r"^[A-Za-z_]\w*$")
TOKEN = re.compile(r"[A-Za-z_]\w*|::|->|##|\.\.\.|&&|[^\s]")
IGNORED = re.compile(
    r'//[^\n]*|/\*[\s\S]*?\*/|(?:u8|u|U|L)?R"([^ ()\\\t\r\n]*)\([\s\S]*?\)\1"'
    r'|(?:u8|u|U|L)?"(?:\\[\s\S]|[^"\\])*"|(?:u|U|L)?\'(?:\\[\s\S]|[^\'\\])*\''
)


def strip_comments_and_strings(text):
    """Keep byte offsets and line numbers while hiding comments and literals."""
    return IGNORED.sub(lambda match: re.sub(r"[^\n]", " ", match.group()), text)


def strip_directives(text):
    result = []
    continuation = False
    for line in text.splitlines(keepends=True):
        directive = continuation or line.lstrip().startswith("#")
        continuation = directive and line.rstrip().endswith("\\")
        result.append(re.sub(r"[^\n]", " ", line) if directive else line)
    return "".join(result)


def header_macros(text):
    macros = {}
    text = text.replace("\\\n", " ")
    for match in re.finditer(
        r"^[ \t]*#[ \t]*define[ \t]+(_?DART_\w+)(\(([^)]*)\))?[ \t]*(.*)$",
        text,
        re.MULTILINE,
    ):
        parameters = (
            None if match[2] is None else [p.strip() for p in match[3].split(",")]
        )
        macros[match[1]] = (parameters, TOKEN.findall(match[4]))
    return macros


def expand_macros(values, macros, active=()):
    """Expand DART declaration macros, including token-pasted accessor names."""
    result = []
    i = 0
    while i < len(values):
        name = values[i]
        if name not in macros or name in active or len(active) >= 16:
            result.append(name)
            i += 1
            continue
        parameters, body = macros[name]
        end = i + 1
        arguments = []
        if parameters is not None:
            if end == len(values) or values[end] != "(":
                result.append(name)
                i += 1
                continue
            end += 1
            depth = 1
            argument = []
            while end < len(values) and depth:
                token = values[end]
                depth += (token == "(") - (token == ")")
                if depth == 0 or (token == "," and depth == 1):
                    arguments.append(argument)
                    argument = []
                else:
                    argument.append(token)
                end += 1
            if depth or len(arguments) != len(parameters):
                result.append(name)
                i += 1
                continue
        substitutions = dict(zip(parameters or [], arguments))
        replacement = []
        for token in body:
            replacement.extend(substitutions.get(token, [token]))
        pasted = []
        j = 0
        while j < len(replacement):
            if replacement[j] == "##" and pasted and j + 1 < len(replacement):
                pasted[-1] += replacement[j + 1]
                j += 2
            else:
                pasted.append(replacement[j])
                j += 1
        result.extend(expand_macros(pasted, macros, active + (name,)))
        i = end
    return result


def source_files(root):
    return sorted(
        path
        for path in root.rglob("*")
        if path.is_file()
        and ".git" not in path.parts
        and (path.suffix in CPP_SUFFIXES or path.name.endswith(".hpp.in"))
    )


@dataclass
class Tokens:
    text: str
    values: list = field(init=False)
    offsets: list = field(init=False)
    pairs: dict = field(init=False)
    scopes: dict = field(default_factory=dict)
    bindings: list = field(default_factory=list)

    def __post_init__(self):
        matches = list(TOKEN.finditer(self.text))
        self.values = [match.group() for match in matches]
        self.offsets = [match.start() for match in matches]
        self.pairs = {}
        stack = []
        for i, value in enumerate(self.values):
            if value in ("(", "{", "["):
                stack.append((value, i))
            elif value in (")", "}", "]") and stack:
                opening, index = stack[-1]
                if opening == {")": "(", "}": "{", "]": "["}[value]:
                    stack.pop()
                    self.pairs[index] = i
                    self.pairs[i] = index

    def line(self, index):
        return self.text.count("\n", 0, self.offsets[index]) + 1


def qualified(values):
    return "".join(values).lstrip(":")


def declaration(values, owner):
    """Read a declaration prefix, before its semicolon or inline body."""
    for opening, value in enumerate(values):
        if value != "(" or opening == 0:
            continue
        name_index = opening - 1
        name = values[name_index]
        if not IDENTIFIER.match(name) or name in {"decltype", "noexcept", "alignas"}:
            continue
        if name.isupper() or name.startswith("DART_"):
            continue
        depth = 1
        closing = opening + 1
        while closing < len(values) and depth:
            depth += (values[closing] == "(") - (values[closing] == ")")
            closing += 1
        if depth:
            continue
        prefix = values[:name_index]
        suffix = values[closing:]
        angle = 0
        assignment = False
        for token in prefix:
            angle += (token == "<") - (token == ">")
            assignment |= token == "=" and angle == 0
        if assignment or "return" in prefix or "->" in prefix or "." in prefix:
            return None
        return {
            "class": owner,
            "name": name,
            "signature": " ".join(values),
            "const": "const" in suffix,
            "virtual": "virtual" in prefix or "override" in suffix,
            "override": "override" in suffix,
            "return_tokens": prefix,
            "name_index": name_index,
        }
    return None


def scopes(tokens):
    """Collect namespace/class spans and direct member declaration prefixes."""
    values = tokens.values
    classes = []
    functions = []
    aliases = []

    def walk(start, end, namespace=(), owner=None, access="public"):
        i = start
        segment = start
        while i < end:
            tokens.scopes[i] = namespace + ((owner.split("::")[-1],) if owner else ())
            value = values[i]
            if (
                value in {"public", "protected", "private"}
                and i + 1 < end
                and values[i + 1] == ":"
            ):
                access = value
                i += 2
                segment = i
                continue
            if value == "namespace":
                j = i + 1
                while j < end and (IDENTIFIER.match(values[j]) or values[j] == "::"):
                    j += 1
                if j < end and values[j] == "{" and j in tokens.pairs:
                    names = tuple(v for v in values[i + 1 : j] if v != "::")
                    walk(j + 1, tokens.pairs[j], namespace + names)
                    i = tokens.pairs[j] + 1
                    segment = i
                    continue
            if value in {"class", "struct", "enum"} and i + 1 < end:
                j = i + 1
                if values[j] == "class":
                    j += 1
                while j < end and values[j].startswith("DART_"):
                    j += 1
                if j + 1 < end and values[j] == "alignas" and j + 1 in tokens.pairs:
                    j = tokens.pairs[j + 1] + 1
                if j < end and IDENTIFIER.match(values[j]):
                    name = values[j]
                    k = j + 1
                    if k < end and values[k] == "final":
                        k += 1
                    if k < end and values[k] in {"{", ":", ";"}:
                        while k < end and values[k] not in {"{", ";"}:
                            k += 1
                        full_name = "::".join(
                            (owner or "::".join(namespace), name)
                        ).strip(":")
                        if k < end and values[k] == "{" and k in tokens.pairs:
                            parents = []
                            parent_start = j + 2
                            angle = 0
                            for p in range(parent_start, k + 1):
                                current = values[p] if p < k else ","
                                angle += (current == "<") - (current == ">")
                                if current == "," and angle == 0:
                                    parents.append(" ".join(values[parent_start:p]))
                                    parent_start = p + 1
                            entry = {
                                "name": full_name,
                                "bases": parents,
                                "members": [],
                                "index": i,
                                "body": k,
                                "end": tokens.pairs[k],
                            }
                            classes.append(entry)
                            if value != "enum":
                                before = len(functions)
                                walk(
                                    k + 1,
                                    tokens.pairs[k],
                                    namespace,
                                    full_name,
                                    "public" if value == "struct" else "private",
                                )
                                entry["members"] = [
                                    m
                                    for m in functions[before:]
                                    if m["class"] == full_name
                                ]
                            i = tokens.pairs[k] + 1
                            segment = i
                            continue
                        if k < end and values[k] == ";":
                            aliases.append((full_name, [full_name], i))
            if value == "(" and i in tokens.pairs:
                for p in range(i, tokens.pairs[i] + 1):
                    tokens.scopes[p] = namespace + (
                        (owner.split("::")[-1],) if owner else ()
                    )
                i = tokens.pairs[i] + 1
                continue
            if value in {";", "{"}:
                prefix = values[segment:i]
                method = declaration(prefix, owner)
                if method and (access != "private" or method["virtual"]):
                    method["index"] = segment
                    method["name_index"] += segment
                    method["access"] = access
                    if owner is None:
                        method["name"] = "::".join(namespace + (method["name"],))
                    functions.append(method)
                if prefix and "using" in prefix and "=" in prefix:
                    equals = prefix.index("=")
                    using = prefix.index("using")
                    aliases.append(
                        (
                            "::".join(
                                (owner or "::".join(namespace), prefix[using + 1])
                            ).strip(":"),
                            prefix[equals + 1 :],
                            segment,
                        )
                    )
                elif prefix and prefix[0] == "typedef" and len(prefix) > 2:
                    aliases.append(
                        (
                            "::".join(
                                (owner or "::".join(namespace), prefix[-1])
                            ).strip(":"),
                            prefix[1:-1],
                            segment,
                        )
                    )
                if value == "{" and i in tokens.pairs:
                    for p in range(i, tokens.pairs[i] + 1):
                        tokens.scopes[p] = namespace + (
                            (owner.split("::")[-1],) if owner else ()
                        )
                    i = tokens.pairs[i] + 1
                else:
                    i += 1
                segment = i
                continue
            i += 1

    walk(0, len(values))
    return classes, functions, aliases


@dataclass
class ApiIndex:
    types: set = field(default_factory=set)
    classes: dict = field(default_factory=dict)
    members: dict = field(default_factory=lambda: defaultdict(list))
    free_functions: dict = field(default_factory=lambda: defaultdict(list))
    aliases: dict = field(default_factory=dict)
    macros: set = field(default_factory=set)
    dependencies: dict = field(default_factory=dict)
    standard_pointers: set = field(default_factory=set)

    def resolve(self, name, scope=(), imports=(), aliases=None):
        name = re.sub(r"\s+", "", name).lstrip(":")
        candidates = [name]
        for size in range(len(scope), -1, -1):
            candidates.append("::".join(scope[:size] + (name,)))
        candidates.extend(namespace + "::" + name for namespace in imports)
        for candidate in candidates:
            if aliases and candidate in aliases:
                return aliases[candidate]
            if candidate in self.types:
                return candidate
        return None

    def owner(self, name):
        seen = set()
        while name in self.aliases and name not in seen:
            seen.add(name)
            name = self.aliases[name]
        if name in self.classes:
            return name
        return None

    def ancestors(self, name):
        result = set()
        seen = set()
        pending = [name]
        while pending:
            current = pending.pop()
            if not current or current in seen:
                continue
            seen.add(current)
            pending.extend(self.dependencies.get(current, ()))
            if current in self.aliases:
                pending.append(self.aliases[current])
            if current in self.classes:
                result.add(current)
                pending.extend(self.classes[current]["bases"])
        return result


def names_in(values):
    i = 0
    while i < len(values):
        if IDENTIFIER.match(values[i]):
            end = i + 1
            while (
                end + 1 < len(values)
                and values[end] == "::"
                and IDENTIFIER.match(values[end + 1])
                and values[i]
                not in {
                    "const",
                    "volatile",
                    "typename",
                    "return",
                    "class",
                    "struct",
                    "static",
                    "virtual",
                    "using",
                    "namespace",
                    "typedef",
                    "new",
                    "delete",
                }
            ):
                end += 2
            yield i, end, qualified(values[i:end])
            i = end
        else:
            i += 1


def variable_after_type(values, end):
    j = end
    if j < len(values) and values[j] == "<":
        depth = 1
        j += 1
        while j < len(values) and depth:
            depth += (values[j] == "<") - (values[j] == ">")
            j += 1
    while j < len(values) and values[j] in {"const", "*", "&", "&&", ">"}:
        j += 1
    if (
        j + 1 < len(values)
        and IDENTIFIER.match(values[j])
        and values[j + 1] in {";", "=", ",", ")", "(", "{"}
    ):
        return j
    return None


def in_template_argument(values, start):
    angle = 0
    for token in reversed(values[:start]):
        if token in {";", "{", "}"}:
            break
        angle += (token == "<") - (token == ">")
    return angle > 0


def first_template_argument(values, start):
    if start >= len(values) or values[start] != "<":
        return []
    depth = 1
    end = start + 1
    while end < len(values):
        token = values[end]
        if depth == 1 and token in {",", ">"}:
            break
        depth += (token == "<") - (token == ">")
        end += 1
    return values[start + 1 : end]


def bind_variable(tokens, index, types):
    """Explicit declarations shadow same-named fields and other functions."""
    values = tokens.values
    start, end = 0, len(values)
    for opening, closing in tokens.pairs.items():
        if opening >= closing or not opening < index < closing:
            continue
        if values[opening] == "{" and closing - opening < end - start:
            start, end = opening, closing
        elif (
            values[opening] == "(" and opening and IDENTIFIER.match(values[opening - 1])
        ):
            body = closing + 1
            while body < len(values) and values[body] in {
                "const",
                "override",
                "final",
                "noexcept",
            }:
                body += 1
            if body < len(values) and values[body] == "{" and body in tokens.pairs:
                if tokens.pairs[body] - opening < end - start:
                    start, end = opening, tokens.pairs[body]
    tokens.bindings.append((start, end, index, values[index], types))


def variable_types(tokens, index, fields):
    if index and tokens.values[index - 1] in {"->", "."}:
        return set(fields.get(tokens.values[index], ()))
    bindings = [
        b
        for b in tokens.bindings
        if b[0] <= index <= b[1] and b[2] <= index and b[3] == tokens.values[index]
    ]
    if bindings:
        return min(bindings, key=lambda b: (b[1] - b[0], -b[2]))[4]
    return set(fields.get(tokens.values[index], ()))


def is_field(tokens, index, classes):
    start = tokens.bindings[-1][0]
    return any(start == cls["body"] for cls in classes) and not any(
        opening < index < closing and tokens.values[opening] == "("
        for opening, closing in tokens.pairs.items()
        if opening < closing
    )


def index_api(header_root):
    api = ApiIndex()
    pending_aliases = []
    macros = {}
    headers = []
    for path in source_files(Path(header_root)):
        if path.suffix not in {
            ".h",
            ".hh",
            ".hpp",
            ".hxx",
            ".ipp",
            ".tpp",
        } and not path.name.endswith(".hpp.in"):
            continue
        clean = strip_comments_and_strings(path.read_text(encoding="utf-8"))
        api.macros.update(
            re.findall(r"^\s*#\s*define\s+(DART_\w+)", clean, re.MULTILINE)
        )
        macros.update(header_macros(clean))
        headers.append(strip_directives(clean))
    for clean in headers:
        tokens = Tokens(" ".join(expand_macros(TOKEN.findall(clean), macros)))
        classes, functions, aliases = scopes(tokens)
        for cls in classes:
            if cls["name"].startswith("dart::"):
                api.types.add(cls["name"])
                api.classes[cls["name"]] = {"bases_raw": cls["bases"], "bases": []}
        for name, values, _ in aliases:
            if name.startswith("dart::"):
                api.types.add(name)
                pending_aliases.append((name, values))
                if any(
                    value in {"std::shared_ptr", "std::unique_ptr", "std::weak_ptr"}
                    for _, _, value in names_in(values)
                ):
                    api.standard_pointers.add(name)
        for function in functions:
            owner = function["class"]
            if owner and owner.startswith("dart::"):
                if function["name"] != owner.split("::")[-1]:
                    api.members[function["name"]].append(function)
            elif owner is None and function["name"].startswith("dart::"):
                api.free_functions[function["name"]].append(function)
    for name, values in pending_aliases:
        for _, _, value in names_in(values):
            resolved = api.resolve(value, tuple(name.split("::")[:-1]))
            if resolved and resolved != name:
                api.aliases.setdefault(name, resolved)
                api.dependencies.setdefault(name, set()).add(resolved)
    for name, cls in api.classes.items():
        scope = tuple(name.split("::")[:-1])
        for raw in cls.pop("bases_raw"):
            for _, _, value in names_in(TOKEN.findall(raw)):
                resolved = api.resolve(value, scope)
                if resolved and resolved != name:
                    cls["bases"].append(resolved)
    api.free_functions = {
        name: declarations
        for name, declarations in api.free_functions.items()
        if name not in api.types
    }
    return api


def classify_member(name, declarations):
    if declarations and all(item["const"] for item in declarations):
        return "accessor"
    getter = re.match(r"^(get|is|has|num)(?:[A-Z_]|$)", name)
    effect = re.search(
        r"(?:Create|Reset|Clear|Remove|Add|Update|Set|Release|Claim|Erase)", name
    )
    return "accessor" if getter and not effect else "mutation"


def public_declaration(declaration):
    return {key: declaration[key] for key in ("class", "signature", "const", "virtual")}


def scan_sources(api, source_root):
    source_root = Path(source_root)
    files = []
    aliases = {}
    fields = defaultdict(set)
    local_bases = {}
    source_returns = defaultdict(list)
    source_macros = set()
    referenced = set()
    tables = {
        section: {}
        for section in ("types", "members", "free_functions", "macros", "overrides")
    }

    def add(section, name, path, line, **metadata):
        row = tables[section].setdefault(
            name, {"name": name, "count": 0, "examples": set(), **metadata}
        )
        row["count"] += 1
        row["examples"].add(f"{path}:{line}")
        return row

    for path in source_files(source_root):
        clean = strip_comments_and_strings(path.read_text(encoding="utf-8"))
        source_macros.update(
            re.findall(r"^\s*#\s*define\s+(DART_\w+)", clean, re.MULTILINE)
        )
        tokens = Tokens(clean)
        classes, functions, source_aliases = scopes(tokens)
        for function in functions:
            source_returns[function["name"].split("::")[-1]].append(function)
        imports = re.findall(r"\busing\s+namespace\s+([\w:]+)\s*;", clean)
        if "dart" in imports:
            imports += [
                "dart::" + namespace for namespace in imports if namespace != "dart"
            ]
        namespace_aliases = dict(
            re.findall(r"\bnamespace\s+(\w+)\s*=\s*([\w:]+)\s*;", clean)
        )
        files.append(
            (
                path.relative_to(source_root).as_posix(),
                tokens,
                classes,
                functions,
                source_aliases,
                imports,
                namespace_aliases,
            )
        )
    for path in sorted(source_root.rglob("CMakeLists.txt")):
        for definitions in re.findall(
            r"\btarget_compile_definitions\s*\(([^)]*)\)",
            path.read_text(encoding="utf-8"),
            re.DOTALL,
        ):
            source_macros.update(re.findall(r"\bDART_\w+", definitions))

    def resolve(name, tokens, index, imports, namespace_aliases):
        head, _, tail = name.partition("::")
        if head in namespace_aliases:
            name = namespace_aliases[head] + ("::" + tail if tail else "")
        scope = tokens.scopes.get(index, ())
        resolved = api.resolve(name, scope, imports, aliases)
        if resolved:
            return resolved
        for size in range(len(scope), 0, -1):
            for base in sorted(local_bases.get("::".join(scope[:size]), ())):
                resolved = api.resolve(name, tuple(base.split("::")), imports, aliases)
                if resolved:
                    return resolved
        return None

    # Aliases and member fields are shared across headers and implementation files.
    for _ in range(3):
        for _, tokens, classes, _, source_aliases, imports, namespace_aliases in files:
            for name, values, index in source_aliases:
                for _, _, value in names_in(values):
                    resolved = resolve(value, tokens, index, imports, namespace_aliases)
                    if resolved:
                        aliases[name] = resolved
                        aliases[name.split("::")[-1]] = resolved
                        break
            for cls in classes:
                bases = set()
                for raw in cls["bases"]:
                    raw = (
                        re.sub(r"\b(public|protected|private|virtual)\b", "", raw)
                        .strip()
                        .split("<")[0]
                    )
                    resolved = resolve(
                        raw, tokens, cls["index"], imports, namespace_aliases
                    )
                    if resolved:
                        bases.update(api.ancestors(resolved))
                    bases.update(local_bases.get(raw, set()))
                if bases:
                    local_bases[cls["name"]] = bases
                    local_bases[cls["name"].split("::")[-1]] = bases

    source_types = {
        name
        for _, _, classes, _, source_aliases, _, _ in files
        for name in [
            *(cls["name"] for cls in classes),
            *(alias[0] for alias in source_aliases),
        ]
    }
    source_types.update(name.split("::")[-1] for name in tuple(source_types))
    for path, tokens, classes, _, _, imports, namespace_aliases in files:
        values = tokens.values
        tokens.source_relevant = path.startswith("dartsim/")
        for start, end, name in names_in(values):
            resolved = resolve(name, tokens, start, imports, namespace_aliases)
            type_end = end
            while not resolved and "::" in name:
                name = name.rsplit("::", 1)[0]
                type_end -= 2
                resolved = resolve(name, tokens, start, imports, namespace_aliases)
            if not resolved:
                continue
            add("types", resolved, path, tokens.line(start))
            tokens.source_relevant = True
            referenced.update(api.ancestors(resolved))
            j = (
                None
                if in_template_argument(values, start)
                else variable_after_type(values, type_end)
            )
            if j is not None:
                owner = api.owner(resolved)
                if owner:
                    bind_variable(tokens, j, {resolved})
                    if is_field(tokens, j, classes):
                        fields[values[j]].add(resolved)
        for start, end, name in names_in(values):
            if resolve(name, tokens, start, imports, namespace_aliases):
                continue
            if (
                ("::" in name and not name.startswith("dart::"))
                or name.startswith("bt")
                or name.split("::")[-1] in source_types
                or name in {"int", "double", "float", "bool", "char", "size_t"}
            ):
                j = variable_after_type(values, end)
                if j is None:
                    continue
                contained = set(local_bases.get(name, ()))
                if name in {"std::shared_ptr", "std::unique_ptr", "std::weak_ptr"}:
                    for offset, _, value in names_in(values[end:j]):
                        resolved = resolve(
                            value, tokens, end + offset, imports, namespace_aliases
                        )
                        if api.owner(resolved):
                            contained.add(resolved)
                if not contained:
                    for _, _, value in names_in(first_template_argument(values, end)):
                        resolved = resolve(
                            value, tokens, start, imports, namespace_aliases
                        )
                        if api.owner(resolved):
                            contained.add("__element__:" + resolved)
                bind_variable(tokens, j, contained or {"__non_dart__"})
                if is_field(tokens, j, classes):
                    fields[values[j]].update(contained or {"__non_dart__"})

    def candidates(name, receiver_types, operator=None):
        # shortcut: lexical matching retains overload candidates, use compiler analysis for exact selection.
        if operator == ".":
            receiver_types = receiver_types - api.standard_pointers
            if not receiver_types:
                return []
        owners = (
            set().union(*(api.ancestors(t) for t in receiver_types))
            if receiver_types
            else referenced
        )
        if operator == "." and name in {"get", "lock"}:
            owners = {api.owner(t) for t in receiver_types}
        return [d for d in api.members.get(name, []) if d["class"] in owners]

    def returns(declarations):
        result = set()
        for item in declarations:
            scope = (
                tuple(item["class"].split("::"))
                if item["class"]
                else tuple(item["name"].split("::")[:-1])
            )
            for _, _, value in names_in(item["return_tokens"]):
                resolved = api.resolve(value, scope)
                owner = api.owner(resolved)
                if owner:
                    result.add(resolved)
        return result

    def receiver(tokens, before, imports, namespace_aliases, depth=0):
        if before < 0 or depth > 8:
            return set()
        values = tokens.values
        value = values[before]
        if value == ")" and before in tokens.pairs:
            opening = tokens.pairs[before]
            function = opening - 1
            if function >= 0 and values[function] == ">":
                angle = 1
                function -= 1
                while function >= 0 and angle:
                    angle += (values[function] == ">") - (values[function] == "<")
                    function -= 1
            if function >= 0:
                method = values[function]
                if function > 0 and values[function - 1] in {"->", "."}:
                    types = receiver(
                        tokens, function - 2, imports, namespace_aliases, depth + 1
                    )
                    if method == "get" and types & api.standard_pointers:
                        return {api.owner(t) for t in types if api.owner(t)}
                    if method in {"at", "back", "front"}:
                        elements = {
                            t.removeprefix("__element__:")
                            for t in types
                            if t.startswith("__element__:")
                        }
                        if elements:
                            return elements
                    declarations = candidates(method, types, values[function - 1])
                    if not declarations:
                        explicit = values[function + 1 : opening]
                        if "std" in explicit and "vector" in explicit:
                            return {"__non_dart__"}
                        source_types = returns(source_returns.get(method, ()))
                        if source_types:
                            return source_types
                        if types == {"__non_dart__"}:
                            return types
                    return returns(declarations)
                if function > 0 and values[function - 1] == "::":
                    start = function - 2
                    while start > 1 and values[start - 1] == "::":
                        start -= 2
                    resolved = resolve(
                        qualified(values[start : function - 1]),
                        tokens,
                        start,
                        imports,
                        namespace_aliases,
                    )
                    if resolved:
                        return returns(candidates(method, {resolved}))
                if method in {"dynamic_cast", "static_cast"}:
                    for _, _, value in names_in(values[function + 1 : opening]):
                        resolved = resolve(
                            value, tokens, function, imports, namespace_aliases
                        )
                        if api.owner(resolved):
                            return {resolved}
                        if value in local_bases:
                            return set(local_bases[value])
        if value == "]" and before in tokens.pairs:
            return receiver(
                tokens, tokens.pairs[before] - 1, imports, namespace_aliases, depth + 1
            )
        return variable_types(tokens, before, fields)

    # Propagate ordinary auto aliases from explicit DART fields and return values.
    for _ in range(3):
        for _, tokens, _, _, _, imports, namespace_aliases in files:
            values = tokens.values
            for i, value in enumerate(values):
                if value != "auto":
                    continue
                j = i + 1
                while j < len(values) and values[j] in {"const", "&", "&&", "*"}:
                    j += 1
                if j + 1 >= len(values) or values[j + 1] != "=":
                    continue
                end = j + 2
                while end < len(values) and values[end] != ";":
                    end += 1
                if end < len(values):
                    inferred = receiver(tokens, end - 1, imports, namespace_aliases)
                    bind_variable(tokens, j, inferred)

    for path, tokens, classes, functions, _, imports, namespace_aliases in files:
        values = tokens.values
        declaration_indices = {function["name_index"] for function in functions}
        for cls in classes:
            bases = local_bases.get(cls["name"], set())
            for item in cls["members"]:
                declarations = [
                    d
                    for d in api.members.get(item["name"], [])
                    if d["class"] in bases and d["virtual"]
                ]
                if declarations and (item["override"] or item["virtual"]):
                    row = add(
                        "overrides",
                        cls["name"] + "::" + item["name"],
                        path,
                        tokens.line(item["index"]),
                        base_classes=sorted(bases),
                        signatures=set(),
                        declarations=[],
                    )
                    row["signatures"].add(item["signature"])
                    row["declarations"].extend(
                        public_declaration(d) for d in declarations
                    )
        for i, name in enumerate(values):
            if name.startswith("DART_"):
                start = tokens.text.rfind("\n", 0, tokens.offsets[i]) + 1
                prefix = tokens.text[start : tokens.offsets[i]]
                if not re.fullmatch(r"\s*#\s*define\s+", prefix):
                    add(
                        "macros",
                        name,
                        path,
                        tokens.line(i),
                        declared_in_dart=name in api.macros,
                        defined_in_source=name in source_macros,
                    )
            if i in declaration_indices:
                continue
            j = i + 1
            if j < len(values) and values[j] == "<":
                depth = 1
                j += 1
                while j < len(values) and depth:
                    depth += (values[j] == "<") - (values[j] == ">")
                    j += 1
            if j >= len(values) or values[j] != "(" or not IDENTIFIER.match(name):
                continue
            declarations = []
            inferred = set()
            if i > 0 and values[i - 1] in {".", "->"}:
                if not tokens.source_relevant:
                    continue
                inferred = receiver(tokens, i - 2, imports, namespace_aliases)
                if inferred or tokens.source_relevant:
                    declarations = candidates(
                        name, inferred, values[i - 1] if inferred else None
                    )
            elif i > 0 and values[i - 1] == "::":
                start = i - 2
                while start > 1 and values[start - 1] == "::":
                    start -= 2
                qualifier = qualified(values[start : i - 1])
                resolved = resolve(qualifier, tokens, start, imports, namespace_aliases)
                if resolved:
                    inferred = {resolved}
                    declarations = candidates(name, inferred)
                else:
                    bases = local_bases.get(qualifier, set()) or local_bases.get(
                        qualifier.split("::")[-1], set()
                    )
                    local_members = {
                        m["name"]
                        for _, _, classes, _, _, _, _ in files
                        for cls in classes
                        if cls["name"] == qualifier
                        or cls["name"].endswith("::" + qualifier)
                        for m in cls["members"]
                    }
                    if bases and name not in local_members:
                        inferred = bases
                        declarations = candidates(name, inferred)
                    head, _, tail = qualifier.partition("::")
                    if head in namespace_aliases:
                        qualifier = namespace_aliases[head] + (
                            "::" + tail if tail else ""
                        )
                    free_name = qualifier + "::" + name
                    if free_name in api.free_functions:
                        add("free_functions", free_name, path, tokens.line(i))
            else:
                scope = tokens.scopes.get(i, ())
                for size in range(len(scope), 0, -1):
                    inferred.update(local_bases.get("::".join(scope[:size]), set()))
                if inferred:
                    declarations = candidates(name, inferred)
            if declarations:
                if j == i + 1:
                    declarations = [
                        d
                        for d in declarations
                        if not (name == "get" and d["signature"].startswith("template"))
                    ]
                if not declarations:
                    continue
                row = add(
                    "members",
                    name,
                    path,
                    tokens.line(i),
                    declarations=[],
                    receiver_types=set(),
                    unresolved_count=0,
                )
                row["declarations"].extend(public_declaration(d) for d in declarations)
                dart_receivers = {t for t in inferred if t.startswith("dart::")}
                row["receiver_types"].update(dart_receivers)
                row["unresolved_count"] += not bool(dart_receivers)

    result = {}
    for section, table in tables.items():
        result[section] = []
        for name in sorted(table):
            row = table[name]
            row["examples"] = sorted(row["examples"])[:3]
            for key in ("receiver_types", "signatures"):
                if key in row:
                    row[key] = sorted(row[key])
            if "declarations" in row:
                row["declarations"] = sorted(
                    {
                        json.dumps(d, sort_keys=True): d for d in row["declarations"]
                    }.values(),
                    key=lambda d: (d["class"], d["signature"]),
                )
            if section == "members":
                row["classification"] = classify_member(name, row["declarations"])
            result[section].append(row)
    return result


def revision(checkout):
    checkout = Path(checkout)
    sha = subprocess.check_output(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True
    ).strip()
    status = subprocess.check_output(
        ["git", "-C", str(checkout), "status", "--porcelain", "--untracked-files=all"],
        text=True,
    )
    if status:
        raise ValueError("source checkout has local changes; scan a clean revision")
    return sha


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "checkout", type=Path, nargs="?", help="clean gz-physics checkout"
    )
    parser.add_argument("--gz-sim", type=Path, help="clean gz-sim checkout")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--check", action="store_true", help="compare without writing")
    args = parser.parse_args(argv)
    if args.checkout is None:
        print("SKIPPED: no gz-physics checkout supplied; inventory was not checked.")
        return 0
    try:
        recorded = json.loads(args.output.read_text()) if args.output.exists() else None
        if args.check and recorded is None:
            raise ValueError("no recorded inventory to check")
        api = index_api(ROOT / "dart")
        sources = {}
        for name, checkout in (("gz-physics", args.checkout), ("gz-sim", args.gz_sim)):
            if checkout is not None:
                sources[name] = {
                    "sha": revision(checkout),
                    **scan_sources(api, checkout),
                }
            elif recorded and name in recorded["sources"]:
                sources[name] = recorded["sources"][name]
                print(
                    f"SKIPPED: {name} was not rescanned; retaining its recorded inventory."
                )
            else:
                raise ValueError(
                    "initial generation requires --gz-sim to record its revision and direct DART use"
                )
        generated = {"schema_version": 1, "sources": sources}
        output = json.dumps(generated, indent=2, sort_keys=True) + "\n"
        if args.check:
            expected = args.output.read_text()
            if output != expected:
                print(
                    "FAILED: DART API inventory differs from the recorded declarations, uses or revisions."
                )
                print(
                    "".join(
                        difflib.unified_diff(
                            expected.splitlines(True),
                            output.splitlines(True),
                            fromfile="recorded",
                            tofile="regenerated",
                        )
                    ),
                    end="",
                )
                return 1
            print(
                "PASS: DART API inventory matches the recorded revision and declarations."
            )
        else:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(output)
            print("Wrote DART API inventory.")
        return 0
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"FAILED: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
