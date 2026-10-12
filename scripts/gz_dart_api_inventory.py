#!/usr/bin/env python3
"""Inventory the DART C++ declarations referenced by Gazebo C++ sources.

This is a lexical compatibility aid, not C++ overload resolution. Unresolved
receivers retain conservative declaration candidates and an explicit count.
Names resolve as C++ lookup finds them, from the innermost namespace or class,
including the class of an out-of-line member definition and the bases of
enclosing classes. A type alias is visible only in its file and in the files
that include it. Type aliases declared inside function bodies are not collected.

A construction lists the constructors that the type declares, including those
of its class template specializations, or, when it declares none, those of the
bases that its implicit constructor calls. A Gazebo class derived from a DART
class constructs that base, so its base clause counts as a construction.

Preprocessor conditions are not evaluated, except that a literal `#if 0` or
`#if 1` drops the branches it rules out. Every other branch is scanned, so the
inventory also lists uses that only a branch inactive in a given build makes.
A use takes the types of every declaration that some build lets it see, so a
variable declared in a branch does not hide other declarations of it from uses
outside that branch. A type alias defined differently in alternative branches
resolves to one of its definitions.
"""

import argparse
import difflib
import json
import posixpath
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
CONDITIONAL = re.compile(r"[ \t]*#[ \t]*(if|ifdef|ifndef|elif|else|endif)\b(.*)")
INCLUDE = re.compile(r'^[ \t]*#[ \t]*include[ \t]*[<"]([^>"\n]+)[>"]', re.MULTILINE)
THIRD_PARTY = re.compile(r"(?:bt|d|ai)[A-Z]")  # Bullet, ODE and Assimp types
NOT_DATA = {
    "(",
    ":",
    "~",
    "class",
    "enum",
    "friend",
    "namespace",
    "operator",
    "return",
    "static_assert",
    "struct",
    "template",
    "typedef",
    "union",
    "using",
}


def strip_comments_and_strings(text):
    """Keep byte offsets and line numbers while hiding comments and literals."""
    return IGNORED.sub(lambda match: re.sub(r"[^\n]", " ", match.group()), text)


def preprocess(text):
    """Blank the branches that a literal `#if 0` or `#if 1` rules out.

    Other conditions are not evaluated, so all their branches stay. Returns the
    text and, for each line, the (conditional, branch) pairs that enclose it.
    """
    frames = []  # [conditional, branch, active, decided]
    lines, branches = [], []
    dead = continued = False
    for line in text.split("\n"):
        match = None if continued else CONDITIONAL.match(line)
        if match and (match[1].startswith("if") or frames):
            keyword, condition = match[1], match[2].strip()
            if keyword.startswith("if"):
                frames.append([len(branches), -1, True, False])
            frame = frames[-1]
            dead = not all(outer[2] for outer in frames[:-1])
            if keyword == "endif":
                frames.pop()
            else:
                literal = condition if keyword in {"if", "elif"} else None
                literal = "1" if keyword == "else" else literal
                frame[1] += 1
                frame[2] = not frame[3] and literal != "0"
                frame[3] = frame[3] or literal == "1"
        elif not continued:
            dead = not all(frame[2] for frame in frames)
        continued = line.rstrip().endswith("\\")
        lines.append(" " * len(line) if dead else line)
        branches.append(tuple((frame[0], frame[1]) for frame in frames))
    return "\n".join(lines), branches


def compatible(first, second):
    """Whether lines in these conditional branches can compile together."""
    branches = dict(first)
    return all(branches.get(key, branch) == branch for key, branch in second)


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
    enclosing: list = field(init=False)
    scopes: dict = field(default_factory=dict)
    bodies: set = field(default_factory=set)
    bindings: list = field(default_factory=list)
    branches: list = field(default_factory=list)
    aliases: dict = field(default_factory=dict)
    wrapped: set = field(default_factory=set)
    using: list = field(default_factory=list)

    def __post_init__(self):
        matches = list(TOKEN.finditer(self.text))
        self.values = [match.group() for match in matches]
        self.offsets = [match.start() for match in matches]
        self.pairs = {}
        self.enclosing = []
        stack = []
        for i, value in enumerate(self.values):
            self.enclosing.append(stack[-1][1] if stack else None)
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
        if ":" in suffix:  # a constructor's member initializers
            values = values[: closing + suffix.index(":")]
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


def data_member(values, owner):
    """Read a data-member declaration, before its semicolon or brace initializer."""
    declarator = values
    angle = 0
    for end, token in enumerate(values):
        angle += (token == "<") - (token == ">")
        if token == "=" and angle == 0:
            declarator = values[:end]
            break
    if (
        len(declarator) < 2
        or not IDENTIFIER.match(declarator[-1])
        or not NOT_DATA.isdisjoint(declarator)
    ):
        return None
    top, angle = [], 0  # the type's tokens outside template arguments
    for token in declarator[:-1]:
        if angle == 0:
            top.append(token)
        angle += (token == "<") - (token == ">")
    pointer = max((k for k, t in enumerate(top) if t in {"*", "&", "&&"}), default=-1)
    return {
        "class": owner,
        "name": declarator[-1],
        "signature": " ".join(values),
        "const": "constexpr" in top or "const" in top[pointer + 1 :],
        "virtual": False,
    }


def qualifier_path(values):
    """The `A::B::` qualifier that ends a declaration prefix, as ("A", "B")."""
    path = []
    end = len(values) - (values[-1:] == ["~"])
    while end > 1 and values[end - 1] == "::":
        k = end - 2
        if values[k] == ">":  # the template arguments of `A<T>::`
            depth = 1
            while k > 0 and depth:
                k -= 1
                depth += (values[k] == ">") - (values[k] == "<")
            k -= 1
        if k < 0 or not IDENTIFIER.match(values[k]):
            break
        path.insert(0, values[k])
        end = k
    return tuple(path)


def scopes(tokens):
    """Collect namespace/class spans and direct member declaration prefixes.

    Members of every access are returned with their `access`; out-of-line member
    definitions get their class's scope for their parameters and bodies.
    """
    values = tokens.values
    classes = []
    functions = []
    aliases = []
    fields = []

    def walk(start, end, namespace=(), owner=None, access="public"):
        here = tuple(owner.split("::")) if owner else namespace
        i = start
        segment = start
        while i < end:
            tokens.scopes[i] = here
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
                    tokens.bodies.add(j)
                    walk(j + 1, tokens.pairs[j], namespace + names)
                    i = tokens.pairs[j] + 1
                    segment = i
                    continue
            if value in {"class", "struct", "enum"} and i + 1 < end:
                j = i + 1
                if values[j] == "class":
                    j += 1
                while j + 1 < end and (  # export macros such as `DART_API`
                    values[j].startswith("DART_")
                    or values[j].isupper()
                    and IDENTIFIER.match(values[j + 1])
                    and values[j + 1] != "final"
                ):
                    j += 1
                if j + 1 < end and values[j] == "alignas" and j + 1 in tokens.pairs:
                    j = tokens.pairs[j + 1] + 1
                if j < end and IDENTIFIER.match(values[j]):
                    name = values[j]
                    k = template_end(values, j + 1)  # a specialization's arguments
                    if k < end and values[k] == "final":
                        k += 1
                    if k < end and values[k] in {"{", ":", ";"}:
                        parent_start = k + 1
                        while k < end and values[k] not in {"{", ";"}:
                            k += 1
                        full_name = "::".join(
                            (owner or "::".join(namespace), name)
                        ).strip(":")
                        if k < end and values[k] == "{" and k in tokens.pairs:
                            tokens.bodies.add(k)
                            parents = []
                            angle = 0
                            for p in range(parent_start, k + 1):
                                current = values[p] if p < k else ","
                                angle += (current == "<") - (current == ">")
                                if current == "," and angle == 0:
                                    parents.append(" ".join(values[parent_start:p]))
                                    parent_start = p + 1
                            entry = {
                                "name": full_name,
                                "kind": value,
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
                    tokens.scopes[p] = here
                i = tokens.pairs[i] + 1
                continue
            if value in {";", "{"}:
                prefix = values[segment:i]
                method = declaration(prefix, owner)
                inside, scope = i, here
                if method:
                    method["index"] = segment
                    method["name_index"] += segment
                    method["access"] = access
                    path = qualifier_path(method["return_tokens"])
                    if owner is None:
                        method["name"] = "::".join(namespace + path + (method["name"],))
                        if path:  # an out-of-line definition looks names up in A::B
                            inside, scope = method["name_index"] + 1, namespace + path
                    functions.append(method)
                elif owner and access != "private":
                    member = data_member(prefix, owner)
                    if member:
                        fields.append(member)
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
                    for p in range(inside, tokens.pairs[i] + 1):
                        tokens.scopes[p] = scope
                    i = tokens.pairs[i] + 1
                else:
                    i += 1
                segment = i
                continue
            i += 1

    walk(0, len(values))
    return classes, functions, aliases, fields


@dataclass
class ApiIndex:
    types: set = field(default_factory=set)
    classes: dict = field(default_factory=dict)
    members: dict = field(default_factory=lambda: defaultdict(list))
    constructors: dict = field(default_factory=dict)
    declares_constructor: set = field(default_factory=set)
    data_members: dict = field(default_factory=lambda: defaultdict(list))
    free_functions: dict = field(default_factory=lambda: defaultdict(list))
    aliases: dict = field(default_factory=dict)
    macros: set = field(default_factory=set)
    dependencies: dict = field(default_factory=dict)
    standard_pointers: set = field(default_factory=set)
    wrappers: set = field(default_factory=set)

    def spellings(self, name, scope=(), imports=()):
        """Qualified spellings of `name`, from the innermost scope outward."""
        name = re.sub(r"\s+", "", name).lstrip(":")
        return [
            *("::".join(scope[:size] + (name,)) for size in range(len(scope), -1, -1)),
            *(namespace + "::" + name for namespace in imports),
        ]

    def find(self, name, scope=(), imports=(), aliases=None):
        """The type that `name` names in `scope`, and the alias it names."""
        for candidate in self.spellings(name, scope, imports):
            if aliases and candidate in aliases:
                return aliases[candidate], candidate
            if candidate in self.types:
                return candidate, None
        return None, None

    def resolve(self, name, scope=(), imports=(), aliases=None):
        return self.find(name, scope, imports, aliases)[0]

    def owner(self, name):
        seen = set()
        while name in self.aliases and name not in seen:
            seen.add(name)
            name = self.aliases[name]
        if name in self.classes:
            return name
        return None

    def constructible(self, name):
        """The class that `name` constructs, unless an alias only wraps it."""
        seen = set()
        while name in self.aliases and name not in seen:
            if name in self.wrappers:
                return None
            seen.add(name)
            name = self.aliases[name]
        return name if name in self.constructors else None

    def constructor_declarations(self, name):
        """The constructors a construction of class `name` can call.

        A class that declares no constructor has an implicit one, which calls
        the constructors of its bases.
        """
        result, pending, seen = [], [name], set()
        while pending:
            current = pending.pop(0)
            if current in seen:
                continue
            seen.add(current)
            result += self.constructors.get(current, [])
            if current not in self.declares_constructor:
                bases = self.classes.get(current, {}).get("bases", ())
                pending += [base for base in map(self.owner, bases) if base]
        return result

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
                    "public",
                    "protected",
                    "private",
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


def wraps(values, direct):
    """Whether an alias definition is a pointer, reference or non-DART template."""
    named = [v for _, _, v in names_in(values) if v not in {"const", "typename"}]
    return not named or not direct(named[0]) or not {"*", "&", "&&"}.isdisjoint(values)


def base_name(raw):
    """The class named in a base clause entry such as `public virtual A<B>`."""
    raw = re.sub(r"\b(public|protected|private|virtual)\b", "", raw)
    return re.sub(r"\s+", "", raw).split("<")[0]


def expand(name, namespace_aliases):
    """Replace a namespace alias at the head of a qualified name."""
    head, _, tail = name.partition("::")
    if head not in namespace_aliases:
        return name
    return namespace_aliases[head] + ("::" + tail if tail else "")


def template_end(values, index):
    """Skip the template argument list, if any, that starts at `index`."""
    if index < len(values) and values[index] == "<":
        depth = 1
        index += 1
        while index < len(values) and depth:
            depth += (values[index] == "<") - (values[index] == ">")
            index += 1
    return index


def variable_after_type(values, end):
    j = template_end(values, end)
    while j < len(values) and values[j] in {"const", "*", "&", "&&", ">"}:
        j += 1
    while j + 2 < len(values) and IDENTIFIER.match(values[j]) and values[j + 1] == "::":
        j += 2  # the `Class::` of an out-of-line static member definition
    if (
        j + 1 < len(values)
        and IDENTIFIER.match(values[j])
        and values[j + 1] in {";", "=", ",", ")", "(", "{"}
    ):
        return j
    return None


def in_template_argument(values, start):
    angle = 0
    for index in range(start - 1, -1, -1):
        if values[index] in {";", "{", "}"}:
            break
        angle += (values[index] == "<") - (values[index] == ">")
    return angle > 0


def constructs(tokens, start, end, declarations, heads):
    """Whether the type named by values[start:end] is constructed there."""
    values = tokens.values
    lead = start - (start > 0 and values[start - 1] == "::")  # `::dart::...`
    before = values[lead - 1] if lead else None
    after = template_end(values, end)
    for head, body in heads:
        if head < start < body:  # a base's constructors run for the class's
            return before in {
                ":",
                ",",
                "public",
                "protected",
                "private",
                "virtual",
            } and (values[after : after + 1] in (["{"], [","]))
    if before in {".", "->", "~", "operator"}:
        return False
    if (
        before == "<"
        and lead > 1
        and values[lead - 2] in {"make_shared", "make_unique"}
    ):
        return values[after : after + 2] == [">", "("]
    if before == "using":  # inherited constructors
        return values[end : end + 2] == ["::", values[end - 1]]
    if in_template_argument(values, start):
        return False
    statement = lead
    while statement and values[statement - 1] not in {";", "{", "}", ":"}:
        statement -= 1
    specifiers = set(values[statement:lead])
    if specifiers & {"typedef", "extern", "friend"} or (
        "static" in specifiers  # an in-class declaration, defined out of line
        and not specifiers & {"inline", "constexpr"}
        and any(tokens.enclosing[start] == body for _, body in heads)
    ):
        return False
    if before == "new" or values[after : after + 1] in (["("], ["{"]):
        return True
    variable = variable_after_type(values, end)
    opening = tokens.enclosing[start]  # parameters are not constructions
    return (
        variable is not None
        and variable not in declarations
        and (
            opening is None
            or values[opening] != "("
            or values[opening - 1] in {"if", "for", "while", "switch", "constexpr"}
        )
        and {"*", "&", "&&"}.isdisjoint(values[after:variable])
    )


def using_declarations(tokens):
    """(name, target, scope, start, stop, depth) for each `using a::b::name;`.

    A declaration is visible at tokens start..stop within `scope`, and a function
    of that name declared in a scope deeper than `depth` hides it.
    """
    values = tokens.values
    result = []
    for i in range(len(values) - 1):
        if values[i] != "using" or values[i + 1] in {"namespace", "typename"}:
            continue
        j = i + 1
        while j < len(values) and (IDENTIFIER.match(values[j]) or values[j] == "::"):
            j += 1
        if values[j : j + 1] != [";"] or "::" not in values[i:j]:
            continue
        name, target = values[j - 1], qualified(values[i + 1 : j])
        scope = tokens.scopes.get(i, ())
        opening = tokens.enclosing[i]
        if opening is None or opening in tokens.bodies:
            result.append((name, target, scope, i, len(values), len(scope)))
        elif values[opening] == "{":  # a block's declaration hides outer functions
            stop = tokens.pairs.get(opening, len(values))
            result.append((name, target, scope, i, stop, float("inf")))
    return result


def elements(types):
    """Element types of the DART containers among `types`."""
    return {
        t.removeprefix("__element__:") for t in types if t.startswith("__element__:")
    }


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
    branches = tokens.branches
    bindings = sorted(
        (
            b
            for b in tokens.bindings
            if b[0] <= index <= b[1]
            and b[2] <= index
            and b[3] == tokens.values[index]
            and compatible(branches[b[2]], branches[index])
        ),
        key=lambda b: (b[1] - b[0], -b[2]),
    )
    types = set()
    for b in bindings:  # innermost first
        types |= b[4]
        # A declaration that only some builds compile hides the others in those
        # builds alone: alternatives and outer declarations stay visible.
        if set(branches[b[2]]) <= set(branches[index]):
            return types
    return types | set(fields.get(tokens.values[index], ()))


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
        text = path.read_text(encoding="utf-8")
        clean = preprocess(strip_comments_and_strings(text))[0]
        api.macros.update(
            re.findall(r"^\s*#\s*define\s+(DART_\w+)", clean, re.MULTILINE)
        )
        macros.update(header_macros(clean))
        headers.append(strip_directives(clean))
    for clean in headers:
        tokens = Tokens(" ".join(expand_macros(TOKEN.findall(clean), macros)))
        classes, functions, aliases, fields = scopes(tokens)
        for cls in classes:
            if cls["name"].startswith("dart::"):
                api.types.add(cls["name"])
                # Specializations and alternative definitions share one entry.
                api.classes.setdefault(cls["name"], {"bases_raw": [], "bases": []})[
                    "bases_raw"
                ] += cls["bases"]
                if cls["kind"] != "enum":
                    api.constructors.setdefault(cls["name"], [])
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
            public = function["access"] != "private"
            if owner and owner.startswith("dart::"):
                if function["name"] != owner.split("::")[-1]:
                    if public or function["virtual"]:
                        api.members[function["name"]].append(function)
                elif "~" not in function["return_tokens"]:
                    api.declares_constructor.add(owner)
                    if public:
                        api.constructors.setdefault(owner, []).append(function)
            elif (
                owner is None
                and function["name"].startswith("dart::")
                and function["return_tokens"][-1:] != ["::"]  # not Class::member
            ):
                api.free_functions[function["name"]].append(function)
        for member in fields:
            if member["class"].startswith("dart::"):
                api.data_members[member["name"]].append(member)
    for name, values in pending_aliases:
        scope = tuple(name.split("::")[:-1])
        for _, _, value in names_in(values):
            resolved = api.resolve(value, scope)
            if resolved and resolved != name:
                api.aliases.setdefault(name, resolved)
                api.dependencies.setdefault(name, set()).add(resolved)
        if wraps(values, lambda value: api.resolve(value, scope)):
            api.wrappers.add(name)
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
    includes = {}
    targets = defaultdict(dict)
    fields = defaultdict(set)
    local_bases = {}
    source_returns = defaultdict(list)
    source_macros = set()
    referenced = set()
    tables = {
        section: {}
        for section in (
            "types",
            "members",
            "constructors",
            "data_members",
            "free_functions",
            "macros",
            "overrides",
        )
    }

    def add(section, name, path, line, **metadata):
        row = tables[section].setdefault(
            name, {"name": name, "count": 0, "examples": set(), **metadata}
        )
        row["count"] += 1
        row["examples"].add(f"{path}:{line}")
        return row

    for path in source_files(source_root):
        relative = path.relative_to(source_root).as_posix()
        text = path.read_text(encoding="utf-8")
        clean, branches = preprocess(strip_comments_and_strings(text))
        includes[relative] = [
            match[1]
            for match in INCLUDE.finditer(text)
            if "include" in clean[match.start() : match.start(1)]
        ]
        source_macros.update(
            re.findall(r"^\s*#\s*define\s+(DART_\w+)", clean, re.MULTILINE)
        )
        tokens = Tokens(clean)
        line = 0
        newlines = [match.start() for match in re.finditer("\n", clean)]
        for offset in tokens.offsets:
            while line < len(newlines) and newlines[line] < offset:
                line += 1
            tokens.branches.append(branches[line])
        classes, functions, source_aliases, _ = scopes(tokens)
        tokens.using = using_declarations(tokens)
        for function in functions:
            source_returns[function["name"].split("::")[-1]].append(function)
        imports = re.findall(r"\busing\s+namespace\s+([\w:]+)\s*;", clean)
        namespace_aliases = dict(
            re.findall(r"\bnamespace\s+(\w+)\s*=\s*([\w:]+)\s*;", clean)
        )
        files.append(
            (
                relative,
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

    # A file sees the declarations of the scanned headers it includes, transitively.
    suffixes = defaultdict(list)
    for relative in includes:
        parts = relative.split("/")
        for start in range(len(parts)):
            suffixes["/".join(parts[start:])].append(relative)
    closure = {}
    for relative in includes:
        seen, pending = set(), [relative]
        while pending:
            current = pending.pop()
            for name in includes[current]:
                local = posixpath.normpath(
                    posixpath.join(posixpath.dirname(current), name)
                )
                for header in [local] if local in includes else suffixes[name]:
                    if header not in seen:
                        seen.add(header)
                        pending.append(header)
        closure[relative] = sorted(seen - {relative}) + [relative]
    own = {
        path: (list(imports), dict(namespace_aliases), list(tokens.using))
        for path, tokens, _, _, _, imports, namespace_aliases in files
    }
    for path, tokens, _, _, _, imports, namespace_aliases in files:
        imports.clear()
        for header in reversed(closure[path]):  # the file's own first
            spaces, renamed, used = own[header]
            imports += [space for space in spaces if space not in imports]
            for key, value in renamed.items():
                namespace_aliases.setdefault(key, value)
            if header != path:  # an included header's namespace-scope declarations
                tokens.using += [
                    (*entry[:3], -1, len(tokens.values), entry[5])
                    for entry in used
                    if entry[5] != float("inf")
                ]
        imports[:] = [expand(space.lstrip(":"), namespace_aliases) for space in imports]
        if "dart" in imports:
            imports += ["dart::" + space for space in imports if space != "dart"]
    declared = defaultdict(lambda: defaultdict(set))
    for path, _, _, functions, *_ in files:
        for function in functions:
            scope = function["class"] or function["name"].rpartition("::")[0]
            declared[path][function["name"].split("::")[-1]].add(
                tuple(scope.split("::")) if scope else ()
            )

    def hidden(name, path, scope, depth):
        """Whether a function declared deeper than `depth` around the call hides it."""
        return any(
            len(owner) > depth and scope[: len(owner)] == owner
            for header in closure[path]
            for owner in declared[header].get(name, ())
        )

    # Base classes declared in the scanned sources, for inherited names.
    source_classes = {cls["name"] for _, _, classes, *_ in files for cls in classes}
    parents = defaultdict(list)
    for _, tokens, classes, _, _, imports, namespace_aliases in files:
        for cls in classes:
            for raw in cls["bases"]:
                spelled = expand(base_name(raw), namespace_aliases)
                scope = tokens.scopes.get(cls["index"], ())
                for candidate in api.spellings(spelled, scope, imports):
                    if candidate in source_classes and candidate != cls["name"]:
                        parents[cls["name"]].append(candidate)
                        break
    ancestry = {}
    for name in parents:
        ancestry[name], pending = [], list(parents[name])
        while pending:
            parent = pending.pop(0)
            if parent not in ancestry[name] and parent != name:
                ancestry[name].append(parent)
                pending += parents.get(parent, ())

    def publish(tokens, name, target, wrapper):
        tokens.aliases[name] = target
        (tokens.wrapped.add if wrapper else tokens.wrapped.discard)(name)

    def visible(relative, tokens):
        tokens.aliases, tokens.wrapped = {}, set()
        for owner in closure[relative]:
            for name, (target, wrapper) in targets[owner].items():
                publish(tokens, name, target, wrapper)

    def lookup(name, tokens, index, imports, namespace_aliases):
        """The DART type that `name` names there, and the source alias it uses.

        Lookup follows C++ name lookup lexically: enclosing scopes from the
        innermost, the bases of enclosing classes, then `Outer::Nested` names
        of DART types and of classes derived from them.
        """
        name = expand(re.sub(r"\s+", "", name).lstrip(":"), namespace_aliases)
        scope = tokens.scopes.get(index, ())
        found = api.find(name, scope, imports, tokens.aliases)
        if found[0]:
            return found
        for size in range(len(scope), 0, -1):
            enclosing = "::".join(scope[:size])
            for parent in ancestry.get(enclosing, ()):
                if f"{parent}::{name}" in tokens.aliases:
                    return tokens.aliases[f"{parent}::{name}"], f"{parent}::{name}"
            for base in sorted(local_bases.get(enclosing, ())):
                found = api.find(name, tuple(base.split("::")), imports, tokens.aliases)
                if found[0]:
                    return found
        head, _, nested = name.rpartition("::")
        if head:
            outer = lookup(head, tokens, index, imports, namespace_aliases)[0]
            owners = (
                [api.owner(outer), *sorted(api.ancestors(outer))]
                if outer
                else sorted(local_bases.get(head, ()))
            )
            for owner in owners:
                if f"{owner}::{nested}" in api.types:
                    return f"{owner}::{nested}", None
        return None, None

    def resolve(name, tokens, index, imports, namespace_aliases):
        return lookup(name, tokens, index, imports, namespace_aliases)[0]

    def wrapped(name, tokens, index, imports, namespace_aliases):
        """Whether `name` resolves through a source alias that wraps its type."""
        alias = lookup(name, tokens, index, imports, namespace_aliases)[1]
        return alias in tokens.wrapped

    # Member fields are shared across headers and implementation files.
    for _ in range(3):
        for (
            path,
            tokens,
            classes,
            _,
            source_aliases,
            imports,
            namespace_aliases,
        ) in files:
            visible(path, tokens)
            for name, values, index in source_aliases:
                for _, _, value in names_in(values):
                    resolved = resolve(value, tokens, index, imports, namespace_aliases)
                    if resolved:
                        wrapper = wraps(
                            values,
                            lambda value: resolve(
                                value, tokens, index, imports, namespace_aliases
                            )
                            and not wrapped(
                                value, tokens, index, imports, namespace_aliases
                            ),
                        )
                        targets[path][name] = (resolved, wrapper)
                        publish(tokens, name, resolved, wrapper)
                        break
            for cls in classes:
                bases = set()
                for raw in cls["bases"]:
                    raw = base_name(raw)
                    resolved = resolve(
                        raw, tokens, cls["index"], imports, namespace_aliases
                    )
                    if resolved:
                        bases.update(api.ancestors(resolved))
                    bases.update(local_bases.get(raw, set()))
                if bases:
                    local_bases[cls["name"]] = bases
                    local_bases[cls["name"].split("::")[-1]] = bases
    for path, tokens, *_ in files:
        visible(path, tokens)

    source_types = {
        name
        for _, _, classes, _, source_aliases, _, _ in files
        for name in [
            *(cls["name"] for cls in classes),
            *(alias[0] for alias in source_aliases),
        ]
    }
    source_types.update(name.split("::")[-1] for name in tuple(source_types))
    for path, tokens, classes, functions, _, imports, namespace_aliases in files:
        values = tokens.values
        tokens.source_relevant = path.startswith("dartsim/")
        declaration_indices = {function["name_index"] for function in functions}
        heads = [(cls["index"], cls["body"]) for cls in classes]
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
            constructed = api.constructible(resolved)
            if (
                constructed
                and not wrapped(name, tokens, start, imports, namespace_aliases)
                and constructs(tokens, start, type_end, declaration_indices, heads)
            ):
                row = add(
                    "constructors",
                    resolved,
                    path,
                    tokens.line(start),
                    declarations=[],
                )
                row["declarations"].extend(
                    public_declaration(d)
                    for d in api.constructor_declarations(constructed)
                )
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
                or THIRD_PARTY.match(name)
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

    def candidates(name, receiver_types, operator=None, table=api.members):
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
        return [d for d in table.get(name, []) if d["class"] in owners]

    def enclosing_bases(tokens, index):
        """The DART bases of the classes around `index`, which `this` reaches."""
        scope = tokens.scopes.get(index, ())
        return set().union(
            *(
                local_bases.get("::".join(scope[:size]), ())
                for size in range(1, len(scope) + 1)
            )
        )

    def record(section, name, declarations, inferred, path, index, tokens):
        if not declarations:
            return
        row = add(
            section,
            name,
            path,
            tokens.line(index),
            declarations=[],
            receiver_types=set(),
            unresolved_count=0,
        )
        row["declarations"].extend(public_declaration(d) for d in declarations)
        dart_receivers = {t for t in inferred if t.startswith("dart::")}
        row["receiver_types"].update(dart_receivers)
        row["unresolved_count"] += not bool(dart_receivers)

    def free_function(name, tokens, index, path, imports, namespace_aliases):
        """The DART function that an unqualified call reaches through imports.

        A using-declaration joins the overload set of its own scope, and a
        using-directive of a DART namespace acts at the global scope, so only a
        function declared between them and the call hides them.
        """
        scope = tokens.scopes.get(index, ())
        for short, target, where, start, stop, depth in tokens.using:
            target = expand(target, namespace_aliases)
            if (
                short == name
                and start < index < stop
                and scope[: len(where)] == where
                and target in api.free_functions
                and not hidden(name, path, scope, depth)
            ):
                return target
        for space in imports:
            if f"{space}::{name}" in api.free_functions:
                return None if hidden(name, path, scope, 0) else f"{space}::{name}"
        return None

    def qualified_function(name, imports, namespace_aliases):
        name = expand(name, namespace_aliases)
        for candidate in (name, *(f"{space}::{name}" for space in imports)):
            if candidate in api.free_functions:
                return candidate
        return None

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
                    if method in {"at", "back", "front"} and elements(types):
                        return elements(types)
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
            types = receiver(
                tokens, tokens.pairs[before] - 1, imports, namespace_aliases, depth + 1
            )
            return elements(types) or types
        types = variable_types(tokens, before, fields)
        if not types and before > 1 and values[before - 1] in {"->", "."}:
            owner = receiver(tokens, before - 2, imports, namespace_aliases, depth + 1)
            if owner == {"__non_dart__"}:  # an unknown field of a non-DART object
                return owner
        return types

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
        class_bodies = {cls["body"] for cls in classes}
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
            if i in declaration_indices or not IDENTIFIER.match(name):
                continue
            j = template_end(values, i + 1)
            call = j < len(values) and values[j] == "("
            access = values[i - 1] if i and values[i - 1] in {".", "->"} else None
            qualifier = None
            if i and values[i - 1] == "::" and (call or tokens.source_relevant):
                start = i - 2
                while start > 1 and values[start - 1] == "::":
                    start -= 2
                qualifier = qualified(values[start : i - 1])
                resolved = resolve(qualifier, tokens, start, imports, namespace_aliases)
            if not call:
                if access and tokens.source_relevant:
                    inferred = receiver(tokens, i - 2, imports, namespace_aliases)
                    declarations = candidates(
                        name, inferred, access if inferred else None, api.data_members
                    )
                    record(
                        "data_members", name, declarations, inferred, path, i, tokens
                    )
                elif qualifier is not None and resolved:
                    declarations = candidates(name, {resolved}, table=api.data_members)
                    record(
                        "data_members", name, declarations, {resolved}, path, i, tokens
                    )
                elif (
                    access is None
                    and qualifier is None
                    and name in api.data_members
                    and tokens.source_relevant
                    and tokens.enclosing[i] not in class_bodies
                    and not any(  # a local declaration hides the field
                        b[3] == name and b[0] <= i <= b[1] and b[2] <= i
                        for b in tokens.bindings
                    )
                ):
                    inferred = enclosing_bases(tokens, i)  # the implicit `this`
                    if inferred:
                        declarations = candidates(
                            name, inferred, table=api.data_members
                        )
                        record(
                            "data_members",
                            name,
                            declarations,
                            inferred,
                            path,
                            i,
                            tokens,
                        )
                continue
            declarations = []
            inferred = set()
            if access:
                if not tokens.source_relevant:
                    continue
                inferred = receiver(tokens, i - 2, imports, namespace_aliases)
                declarations = candidates(name, inferred, access if inferred else None)
            elif qualifier is not None:
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
                    free = qualified_function(
                        qualifier + "::" + name, imports, namespace_aliases
                    )
                    if free:
                        add("free_functions", free, path, tokens.line(i))
            else:
                inferred = enclosing_bases(tokens, i)
                if inferred:
                    declarations = candidates(name, inferred)
                free = not declarations and free_function(
                    name, tokens, i, path, imports, namespace_aliases
                )
                if free:
                    add("free_functions", free, path, tokens.line(i))
            if j == i + 1:
                declarations = [
                    d
                    for d in declarations
                    if not (name == "get" and d["signature"].startswith("template"))
                ]
            record("members", name, declarations, inferred, path, i, tokens)

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
