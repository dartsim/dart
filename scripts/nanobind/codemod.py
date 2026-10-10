"""Generate a mechanical non-GUI port for review in an empty destination.

The generated bindings require caster includes and manual compiler fixes.
"""

import argparse
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEST = ROOT / "python/dartpy"


def mask(text):
    return re.sub(
        r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"',
        lambda m: " " * len(m[0]),
        text,
    )


def close(text, start, left="(", right=")"):
    clean = mask(text)
    level = 0
    for i in range(start, len(text)):
        if clean[i] == left:
            level += 1
        elif clean[i] == right and not (right == ">" and clean[i - 1] == "-"):
            level -= 1
            if level == 0:
                return i
    raise ValueError(f"unbalanced {left}: {text[start:start + 100]}")


def split(text):
    clean = mask(text)
    stack = []
    start = 0
    result = []
    for i, ch in enumerate(clean):
        if ch in "(<[{":
            stack.append(ch)
        elif ch in ")>]}" and stack:
            if ch == ">" and i and clean[i - 1] == "-":
                continue
            stack.pop()
        elif ch == "," and not stack:
            result.append(text[start:i].strip())
            start = i + 1
    result.append(text[start:].strip())
    return result


def nullable(typ):
    # Only the argument itself is nullable; containers of pointers are values.
    outer = []
    depth = 0
    for ch in typ:
        if ch == "<":
            depth += 1
        elif ch == ">":
            depth -= 1
        elif not depth:
            outer.append(ch)
    return bool(re.search(r"\*|shared_ptr|unique_ptr|\w+Ptr\b", "".join(outer)))


def add_none(text, log):
    changes = []
    for match in re.finditer(r"\.(?:def|def_static)\s*\(", mask(text)):
        start = text.index("(", match.start())
        end = close(text, start)
        body = text[start + 1 : end]
        args = list(re.finditer(r'nb::arg\("([^"\n]*)"\)(?!\.none)', body))
        if not args:
            continue
        signature = re.search(r"\[[^\]]*\]\s*\(", mask(body))
        init = re.search(r"nb::init\s*<", body)
        if signature:
            sigstart = body.index("(", signature.start())
            params = split(body[sigstart + 1 : close(body, sigstart)])
            if params and re.search(r"\bself\b", params[0]):
                params = params[1:]
        elif init:
            sigstart = body.index("<", init.start())
            params = split(body[sigstart + 1 : close(body, sigstart, "<", ">")])
        else:
            log.append({"kind": "unresolved-signature", "binding": body[:140]})
            continue
        if len(params) != len(args):
            log.append({"kind": "unresolved-arity", "binding": body[:140]})
            continue
        for param, arg in zip(params, args):
            if nullable(param):
                changes.append((start + 1 + arg.end(), ".none()"))
                log.append({"kind": "none", "name": arg[1], "signature": param})
    for pos, value in reversed(changes):
        text = text[:pos] + value + text[pos:]
    return text


def translate(text, log):
    # Collapse only preprocessor continuations, leaving a valid single-line macro.
    text = re.sub(
        r"(?m)^#define[^\n]*\\\n(?:[^\n]*\\\n)*[^\n]*",
        lambda m: m[0].replace("\\\n", " ").replace("\n", " "),
        text,
    )
    text = re.sub(r'^\s*#include [<"]pybind11/[^\n]+\n', "", text, flags=re.M)
    text = re.sub(r"namespace py = pybind11;", "", text)
    text = text.replace("::pybind11::", "nb::").replace("pybind11::", "nb::")
    text = text.replace("::py::", "nb::").replace("py::", "nb::")
    text = re.sub(r"nb::\s*module\b", "nb::module_", text)
    text = re.sub(r"nb::\s*init", "nb::init", text)
    for old, new in {
        "return_value_policy": "rv_policy",
        "def_readwrite": "def_rw",
        "def_readonly_static": "def_ro_static",
        "def_readonly": "def_ro",
        "def_property_readonly": "def_prop_ro",
        "def_property": "def_prop_rw",
        "reinterpret_borrow": "borrow",
        "PYBIND11_MODULE": "NB_MODULE",
        ".none(true)": ".none()",
        "module_::import(": "module_::import_(",
    }.items():
        text = text.replace(old, new)
    # Drop holders; dart_class retains bases and detects trampoline aliases.
    edits = []
    for m in re.finditer(r"nb::\s*class_\s*<", text):
        start = text.index("<", m.start())
        end = close(text, start, "<", ">")
        types = split(text[start + 1 : end])
        kept = [
            t for t in types if not re.search(r"shared_ptr|unique_ptr|BodyNodePtr", t)
        ]
        edits.append(
            (m.start(), end + 1, "dartnb::dart_class<" + ", ".join(kept) + ">")
        )
        log.append({"kind": "class", "types": kept})
    for start, end, replacement in reversed(edits):
        text = text[:start] + replacement + text[end:]
    # nanobind factory syntax for the few native factory overloads.
    text = text.replace("nb::init(", "nb::new_(")
    text = add_none(text, log)
    edits = []
    for m in re.finditer(r"nb::enum_\s*<", text):
        end = close(text, text.index("<", m.start()), "<", ">")
        start = text.index("(", end)
        end = close(text, start)
        if "is_arithmetic" not in text[start:end]:
            edits.append(end)
    for end in reversed(edits):
        text = text[:end] + ", nb::is_arithmetic()" + text[end:]
    return '#include "detail/dart_nb.hpp"\n\n' + text


def self_check():
    assert split("A<B, C>, const D*, E") == ["A<B, C>", "const D*", "E"]
    log = []
    out = translate(
        'py::class_<T, B, std::shared_ptr<T>>(m, "T").def("f", [](T* self, B* b, int n) {}, py::arg("b"), py::arg("n"));',
        log,
    )
    assert "dart_class<T, B>" in out and 'nb::arg("b").none()' in out
    assert 'nb::arg("n").none()' not in out
    assert sum(row["kind"] == "none" for row in log) == 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--source", type=Path, help="pybind11 sources from DART 6.20")
    parser.add_argument("--output", type=Path, default=DEST)
    args = parser.parse_args()
    self_check()
    if args.self_check:
        return
    destination = args.output
    if destination.exists():
        parser.error("the destination must not exist; use --output for a scratch port")
    generated = 0
    if args.source is None or not args.source.is_dir():
        parser.error("--source must name an existing pybind11 source directory")
    for source in sorted(args.source.rglob("*")):
        relative = source.relative_to(args.source)
        if "gui" in relative.parts or source.suffix not in {".cpp", ".hpp", ".h"}:
            continue
        if source.name in {"eigen_geometry_pybind.h", "pointers.hpp"}:
            continue  # These caster/holder declarations require a hand port.
        target = destination / (
            "module.cpp" if relative == Path("dartpy.cpp") else relative
        )
        log = []
        output = translate(source.read_text(), log)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(output)
        generated += 1
    print(f"Generated {generated} files; manual review required")


if __name__ == "__main__":
    main()
