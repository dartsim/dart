"""Fast lexical and freshness checks for the DART C++ API inventory."""

import ast
import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "gz_dart_api_inventory.py"
INVENTORY = ROOT / "tools" / "gazebo" / "compat" / "dart_api_inventory.json"
SECTIONS = (
    "types",
    "headers",
    "members",
    "constructors",
    "data_members",
    "variables",
    "free_functions",
    "macros",
    "overrides",
)


def _load():
    spec = importlib.util.spec_from_file_location("gz_dart_api_inventory", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def extractor():
    return _load()


@pytest.fixture
def api(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """namespace dart {
namespace dynamics {
class Widget {
public:
  Widget();
  explicit Widget(int value);
  ~Widget();
  DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(Widget)
  DART_SYNTHETIC_PROPERTY(double, Friction)
  int getValue();
  int get() const;
  int compute() const;
  bool isReady();
  void setValue(int value);
  void getAndReset();
  void commentedOnly();
  void clear();
  template<class T> void createShapeNodeWith();
  static Widget* create();
  static constexpr int kLimit = 3;
  struct Properties { int mName; };
protected:
  bool mDirty;
private:
  void privateOnly();
};
class Solver {
public:
  explicit Solver(int flags = 0);
  virtual ~Solver() = default;
};
class DerivedSolver : public Solver { public: void solve(); };
template <class... T> struct Compose { Compose() = default; };
template <class A, class... R>
struct Compose<A, R...> : Compose<R...> { Compose(); explicit Compose(A value); };
using Props = Compose<int>;
class Unused {};
class StaticOnly { public: static void mutate(); };
template<class T> class Pointer {
public:
  T* get() const;
  T* operator->() const;
};
using WidgetPtr = Pointer<Widget>;
using WidgetSharedPtr = std::shared_ptr<Widget>;
inline int Widget::getValue() { return 0; }
}
namespace collision {
template<class T> struct Registration {};
class Detector {
public:
  template<class T> using Registrar = Registration<T>;
  static const char* getStaticType();
  static void create();
  virtual bool raycast(const dynamics::Widget& widget) const = 0;
protected:
  virtual void refresh(dynamics::Widget* widget) = 0;
};
class DerivedDetector : public Detector {};
class DART_COLLISION_NATIVE_API Exported {
public:
  void setExported(int value);
};
struct Option {
  bool enableContact;
  std::size_t maxNumContacts = 1000u;
  const Exported* exported;
  Option(bool enable = true) : enableContact(enable) {}
private:
  int hidden;
};
struct Contact {
  Eigen::Vector3d point;
  double penetrationDepth{0.0};
};
}
namespace math {
double measure(double value);
}
}
"""
    )
    (headers / "config.hpp.in").write_text(
        "#define DART_VERSION_AT_LEAST(x, y, z) (x)\n"
        "#define DART_SHARED_PTR_CREATOR_NAME createShared\n"
        "#define DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(T) "
        "static std::shared_ptr<T> createShared();\n"
        "#define DART_SYNTHETIC_PROPERTY(TYPE, NAME) "
        "TYPE get##NAME() const; void set##NAME(TYPE value);\n"
    )
    return extractor.index_api(headers)


def _scan_files(extractor, api, tmp_path, snippets):
    source = tmp_path / "source"
    for name, snippet in snippets.items():
        path = source / "dartsim" / "src" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(snippet)
    return extractor.scan_sources(api, source)


def _scan(extractor, api, tmp_path, snippet):
    return _scan_files(extractor, api, tmp_path, {"sample.cc": snippet})


def _entries(result, section):
    return {entry["name"]: entry for entry in result[section]}


@pytest.mark.parametrize(
    "annotation",
    ["DART_SYNTHETIC_API", '__attribute__((visibility("default")))', "[[nodiscard]]"],
)
@pytest.mark.parametrize(
    "declaration", ["struct {} A {{}};", "struct {} A; struct A {{}};"]
)
def test_referenced_class_annotations_survive_macro_expansion(
    extractor, tmp_path, annotation, declaration
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    definitions = "#define DART_SYNTHETIC_API\nnamespace dart { "
    definition = declaration.format(annotation) + " using Alias = A; struct B : A {}; }"
    header.write_text(definitions + definition)
    source = "void use(dart::A* a, dart::Alias* alias, dart::B* b) {}"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    for name in ("dart::A", "dart::Alias", "dart::B"):
        evidence = _entries(before, "types")[name]["annotation_declarations"]
        assert any(d["class"] == "dart::A" for d in evidence)
    header.write_text(definitions + definition.replace(annotation, ""))
    after = extractor.scan_sources(extractor.index_api(headers), tmp_path / "source")
    assert after != before
    assert all("annotation_declarations" not in row for row in after["types"])


@pytest.mark.parametrize(
    ("definition", "source", "section", "name"),
    [
        (
            "struct A { void hit() noexcept(kNoThrow); };",
            "void use(dart::A& a) { a.hit(); }",
            "members",
            "hit",
        ),
        (
            "struct A { A() noexcept(kNoThrow); };",
            "void use() { dart::A a; }",
            "constructors",
            "dart::A",
        ),
        (
            "struct A { virtual void hit() noexcept(kNoThrow); };",
            "struct B : dart::A { void hit() noexcept(false) override; };",
            "overrides",
            "B::hit",
        ),
        (
            "void hit() noexcept(kNoThrow);",
            "void use() { dart::hit(); }",
            "free_functions",
            "dart::hit",
        ),
        (
            "struct A { virtual void hit() noexcept(kNoThrow) = 0; };",
            "struct B : dart::A {};",
            "constructors",
            "dart::A",
        ),
    ],
)
def test_used_exception_conditions_retain_constant_dependencies(
    extractor, tmp_path, definition, source, section, name
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { constexpr bool kRoot = false; constexpr bool kNoThrow = kRoot; "
        + definition
        + " }"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    row = _entries(before, section)[name]
    declarations = row.get("declarations", []) + row.get(
        "pure_virtual_declarations", []
    )
    evidence = row.get("exception_specification_declarations", []) + [
        dependency
        for item in declarations
        for dependency in item.get("exception_specification_declarations", [])
    ]
    assert {d["signature"] for d in evidence} == {
        "constexpr bool kRoot = false",
        "constexpr bool kNoThrow = kRoot",
    }
    header.write_text(header.read_text().replace("kRoot = false", "kRoot = true"))
    after = extractor.scan_sources(extractor.index_api(headers), tmp_path / "source")
    assert after != before


@pytest.mark.parametrize("enum_kind", ["anonymous", "unscoped", "scoped", "private"])
@pytest.mark.parametrize("use", ["exception", "default", "width"])
@pytest.mark.parametrize("attributed", [False, True])
def test_enum_conditions_retain_initializer_and_implicit_dependencies(
    extractor, tmp_path, enum_kind, use, attributed
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    members = "kFirst = Root, kNoThrow" + (
        " [[maybe_unused]] = kFirst" if attributed else ""
    )
    enum = {
        "anonymous": "enum : unsigned { " + members + " };",
        "unscoped": "enum Flags : unsigned { " + members + " };",
        "scoped": "enum class Flags : unsigned { " + members + " };",
        "private": "private: enum : unsigned { " + members + " }; public:",
    }[enum_kind]
    name = "Flags::kNoThrow" if enum_kind == "scoped" else "kNoThrow"
    declaration, source, section, entry, key = {
        "exception": (
            "virtual void hit() noexcept(" + name + " == 2);",
            "struct B : dart::A { void hit() noexcept(false) override; };",
            "overrides",
            "B::hit",
            "exception_specification_declarations",
        ),
        "default": (
            "void run(unsigned = " + name + ");",
            "void use(dart::A& a) { a.run(); }",
            "members",
            "run",
            "default_argument_declarations",
        ),
        "width": (
            "unsigned flags : " + name + ";",
            "void use(dart::A& a) { (void)a.flags; }",
            "data_members",
            "flags",
            "bit_width_declarations",
        ),
    }[use]
    text = "namespace dart { constexpr unsigned Root = 1; "
    text += (
        "struct A { " + enum + declaration + " }; }"
        if enum_kind == "private"
        else enum + " struct A { " + declaration + " }; }"
    )
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    row = _entries(before, section)[entry]
    evidence = row.get(key, []) + [
        d for item in row.get("declarations", []) for d in item.get(key, [])
    ]
    assert any(d["signature"] == "constexpr unsigned Root = 1" for d in evidence)
    assert any(d["signature"].startswith("kFirst = Root") for d in evidence)
    assert any(d["signature"].startswith("kNoThrow") for d in evidence)
    header.write_text(text.replace("Root = 1", "Root = 2"))
    after = extractor.scan_sources(extractor.index_api(headers), tmp_path / "source")
    assert after != before


def test_anonymous_enums_have_no_phantom_types_or_readable_private_members(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { enum { Ready = 1, Waiting }; struct A { private: enum { Hidden = 1 }; public: enum : unsigned { Visible = 2 }; unsigned flags; }; }"
    )
    api = extractor.index_api(headers)
    assert api.types == {"dart::A"}
    assert "dart::Ready" in api.variables
    assert "dart::Waiting" in api.variables
    assert "Hidden" not in api.data_members
    assert "Visible" in api.data_members
    result = _scan(extractor, api, tmp_path, "void use() { dart::A a{1}; }")
    fields = _entries(result, "constructors")["dart::A"]["brace_member_declarations"]
    assert [field["name"] for field in fields] == ["flags"]


def test_exception_conditions_keep_nested_colons_and_declaration_scope(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { constexpr bool kNoThrow = false; struct A { private: static constexpr bool kNoThrow = true; public: A() noexcept(kNoThrow ? true : false) : value(0) {} void hit() noexcept(kNoThrow ? true : false); int value; }; }"
    )
    before = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::A& a) { dart::A b; a.hit(); }",
    )
    for section, name in (("constructors", "dart::A"), ("members", "hit")):
        item = _entries(before, section)[name]["declarations"][0]
        assert "? true : false )" in item["signature"]
        assert item["exception_specification_declarations"] == [
            {
                "class": "dart::A",
                "signature": "static constexpr bool kNoThrow = true",
                "const": True,
                "virtual": False,
            }
        ]
        assert "value (" not in item["signature"]


@pytest.mark.parametrize("macro", ["DART_MEMBER_API", "DART_GENERATE_API"])
def test_api_suffixed_declaration_generators_still_expand(extractor, tmp_path, macro):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    definitions = {
        "DART_MEMBER_API": "#define DART_MEMBER_API(NAME) void NAME();\nnamespace dart { struct A { DART_MEMBER_API(hit) }; }",
        "DART_GENERATE_API": "#define DART_GENERATE_API(NAME) struct NAME { void hit(); };\nnamespace dart { DART_GENERATE_API(A) }",
    }
    header.write_text(definitions[macro])
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::A& a) { a.hit(); }",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 1
    assert hit["receiver_types"] == ["dart::A"]


@pytest.mark.parametrize(
    ("declaration", "changed", "source"),
    [
        ("int run(int);", "int run(int, int);", "void use() { dart::run(1); }"),
        ("int run(int);", "void run(int);", "void use() { int value = dart::run(1); }"),
        ("int run(int);", "double run(int);", "int (*callback)(int) = dart::run;"),
        (
            "int run(int = kDefault) noexcept(kNoThrow);",
            "int run(int, int) noexcept(kNoThrow);",
            "void use() { dart::run(); }",
        ),
        (
            "int run(int); double run(double);",
            "int run(int, int); double run(double);",
            "void use() { int (*callback)(int) = dart::run; }",
        ),
    ],
)
def test_free_function_candidates_retain_signature_changes(
    extractor, tmp_path, declaration, changed, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    prefix = (
        "namespace dart { constexpr int kDefault = 1; constexpr bool kNoThrow = true; "
    )
    header.write_text(prefix + declaration + " }")
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    row = _entries(before, "free_functions")["dart::run"]
    assert row["declarations"]
    assert all(item["class"] is None for item in row["declarations"])
    if "kDefault" in declaration:
        assert (
            row["declarations"][0]["default_argument_declarations"]
            == row["default_argument_declarations"]
        )
        assert (
            row["declarations"][0]["exception_specification_declarations"]
            == row["exception_specification_declarations"]
        )
    header.write_text(prefix + changed + " }")
    after = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert (
        _entries(after, "free_functions")["dart::run"]["declarations"]
        != row["declarations"]
    )


@pytest.mark.parametrize(
    ("declaration", "source", "section", "name"),
    [
        ("void hit();", "void use(dart::A& a) { a.hit(); }", "members", "hit"),
        ("virtual void hit();", "void use(dart::A& a) { a.hit(); }", "members", "hit"),
        ("static void hit();", "void use() { dart::A::hit(); }", "members", "hit"),
        (
            "int value;",
            "void use(dart::A& a) { (void)a.value; }",
            "data_members",
            "value",
        ),
        (
            "static int value;",
            "void use() { (void)dart::A::value; }",
            "data_members",
            "value",
        ),
        (
            "unsigned value : 3;",
            "void use(dart::A& a) { (void)a.value; }",
            "data_members",
            "value",
        ),
    ],
)
def test_used_member_candidates_retain_public_and_protected_access(
    extractor, tmp_path, declaration, source, section, name
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { class A { public: " + declaration + " }; }"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    declarations = _entries(before, section)[name]["declarations"]
    assert {item["access"] for item in declarations} == {"public"}
    header.write_text(text.replace("public:", "protected:"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert {
        item["access"] for item in _entries(after, section)[name]["declarations"]
    } == {"protected"}
    assert after != before


def test_hidden_friend_signatures_ignore_class_access_sections(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { class A { private: friend int run(A); }; }"
    header.write_text(text)
    before = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::A a) { run(a); }",
    )
    row = _entries(before, "free_functions")["dart::run"]
    assert row["declarations"]
    assert all(
        item["class"] is None and "access" not in item for item in row["declarations"]
    )
    header.write_text(text.replace("private:", "public:"))
    after = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::A a) { run(a); }",
    )
    assert after == before
    header.write_text(text.replace("run(A)", "run(A, int)"))
    changed = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::A a) { run(a); }",
    )
    assert changed != before


@pytest.mark.parametrize(
    ("namespace", "class_name"),
    [("dart", "A"), ("dart", "DART_CLASS_NAME"), ("DART_NAMESPACE", "A")],
)
def test_class_annotations_retain_visibility_string_values(
    extractor, tmp_path, namespace, class_name
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "#define DART_CLASS_NAME A\n#define DART_NAMESPACE dart\nnamespace "
        + namespace
        + ' { struct __attribute__((visibility("default"))) '
        + class_name
        + " {}; }"
    )
    header.write_text(text)
    before = _scan(
        extractor, extractor.index_api(headers), tmp_path, "void use(dart::A* a) {}"
    )
    evidence = _entries(before, "types")["dart::A"]["annotation_declarations"]
    assert any('"default"' in d["signature"] for d in evidence)
    header.write_text(text.replace('"default"', '"hidden"'))
    after = extractor.scan_sources(extractor.index_api(headers), tmp_path / "source")
    assert after != before


@pytest.mark.parametrize("initializer", ["({})", "{{{}}}"])
def test_direct_auto_initializers_retain_return_shapes(
    extractor, tmp_path, initializer
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Item { void hit(); }; struct Factory { Item make(); Item* pointer(); std::vector<Item*> items(); }; }"
    )
    source = "void use(dart::Factory& factory) { "
    for name, expression, call in (
        ("value", "factory.make()", "value.hit();"),
        ("pointer", "factory.pointer()", "pointer->hit();"),
        ("items", "factory.items()", "items.front()->hit();"),
    ):
        source += "auto " + name + initializer.format(expression) + "; " + call
    source += " }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 3
    assert hit["receiver_types"] == ["dart::Item"]
    assert hit["unresolved_count"] == 0


def test_direct_auto_initializers_preserve_foreign_and_unknown_shadowing(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct Item { void hit(); }; }")
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Foreign { void hit(); };
struct Factory { Foreign make(); };
void use(dart::Item* outer, Factory& factory) {
  auto value = outer;
  { auto value(factory.make()); value.hit(); }
  { auto value{factory.make()}; value.hit(); }
  value->hit();
}
template<class Factory> void unknown(dart::Item* known, Factory factory) {
  auto value(factory()); value.hit();
}
""",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 2
    assert hit["unresolved_count"] == 1


@pytest.mark.parametrize("initializer", [" = {}", "({})", "{{{}}}"])
def test_grouped_auto_initializers_retain_each_return_shape(
    extractor, tmp_path, initializer
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { struct Item { void hit(); }; struct Factory { Item make(); Item* pointer(); std::vector<Item*> items(); }; }"
    )
    source = "void use(dart::Factory& factory) { "
    for name, expression, call in (
        ("value", "factory.make()", "{}.hit();"),
        ("pointer", "factory.pointer()", "{}->hit();"),
        ("items", "factory.items()", "{}.front()->hit();"),
    ):
        source += "auto " + name + initializer.format(expression)
        source += ", second_" + name + initializer.format(expression) + "; "
        source += call.format(name) + call.format("second_" + name)
    source += " }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 6
    assert hit["receiver_types"] == ["dart::Item"]
    assert hit["unresolved_count"] == 0
    header.write_text(header.read_text().replace("void hit();", "void hit(int);"))
    changed = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(changed, "members")["hit"]["declarations"] != hit["declarations"]


def test_grouped_auto_bindings_preserve_control_and_lambda_shadows(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct Item { void hit(); }; }")
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Foreign { void hit(); };
struct Factory { Foreign make(); };
void use(dart::Item* outer, Factory& factory) {
  auto first = outer, value = outer;
  { auto first(factory.make()), value(factory.make()); value.hit(); }
  value->hit();
  if (auto first = factory.make(), value = factory.make(); true) value.hit();
  else value.hit();
  value->hit();
  for (auto first{factory.make()}, value{factory.make()}; false;) value.hit();
  value->hit();
  auto lambda = [&] { auto first(factory.make()), value(factory.make()); value.hit(); };
  value->hit();
}
""",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 4
    assert hit["receiver_types"] == ["dart::Item"]
    assert hit["unresolved_count"] == 0


def test_grouped_auto_array_aliases_keep_raw_pointer_dispatch(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Target { void other(); }; struct Item { void hit(); Target* operator->(); }; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """void use() {
  dart::Item values[2];
  auto first = values, second = values;
  first->hit(); second->hit(); second[0]->other();
  auto third(values), fourth(values);
  third->hit(); fourth->hit();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 4
    assert members["hit"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 1
    assert members["other"]["receiver_types"] == ["dart::Target"]


def test_fixed_array_decay_keeps_builtin_arrow_and_value_elements(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Target { void other(); }; struct Item { void hit(); Target* operator->(); }; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """void use() {
  dart::Item values[2]; values->hit();
  auto pointer = values; pointer->hit(); pointer[0]->other();
  dart::Item grid[2][3]; grid[0]->hit(); grid[0][0]->other();
  std::array<dart::Item, 2> owned; owned[0]->other();
  std::vector<dart::Item> sequence; sequence.front()->other();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 3
    assert members["hit"]["receiver_types"] == ["dart::Item"]
    assert members["hit"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 4
    assert members["other"]["receiver_types"] == ["dart::Target"]


def test_namespace_variable_receivers_preserve_types_and_shadowing(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Item { void hit(); }; extern Item global; extern Item* pointer; extern std::vector<Item*> items; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Foreign { void hit(); };
namespace d = dart;
using dart::global;
void use() {
  dart::global.hit(); d::pointer->hit(); dart::items.front()->hit();
  global.hit();
  { Foreign global; global.hit(); dart::global.hit(); }
  global.hit();
}
""",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 6
    assert hit["receiver_types"] == ["dart::Item"]
    assert hit["unresolved_count"] == 0


def test_namespace_variable_imports_stop_at_the_innermost_declaration(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Item { void hit(); }; extern Item global; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """namespace foreign { struct Foreign { void hit(); }; extern Foreign global; }
using dart::global;
void use() {
  global.hit();
  { using foreign::global; global.hit(); }
  global.hit();
}
""",
    )
    assert _entries(result, "variables")["dart::global"]["count"] == 3
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 2
    assert hit["unresolved_count"] == 0


def test_type_using_declarations_keep_scopes_and_include_visibility(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Item { Item(); void hit(); }; }"
    )
    result = _scan_files(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        {
            "using.hh": "using dart::Item;",
            "included.cc": """#include "using.hh"
namespace foreign { struct Item { void hit(); }; }
void use() {
  Item first; first.hit();
  { using foreign::Item; Item local; local.hit(); }
  Item second; second.hit();
}
namespace own {
struct Item { void hit(); };
void use() { Item local; local.hit(); }
}
""",
            "unrelated.cc": """namespace other { struct Item { void hit(); }; }
using other::Item;
void use() { Item local; local.hit(); }
""",
        },
    )
    constructors = _entries(result, "constructors")
    assert set(constructors) == {"dart::Item"}
    assert constructors["dart::Item"]["count"] == 2
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 2
    assert hit["unresolved_count"] == 0


def test_namespace_and_member_array_receivers_preserve_dimensions(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """namespace dart {
struct Target { void other(); };
struct Item { void hit(); Target* operator->(); };
extern Item globals[2][3];
extern Item* pointers[2];
struct Holder {
  Item values[2][3];
  Item* pointers[2];
  std::vector<Item> sequence;
  std::array<Item, 2> owned;
};
}
"""
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """void use(dart::Holder& holder) {
  dart::globals[0]->hit(); dart::globals[0][0]->other();
  dart::pointers[0]->hit();
  holder.values[0]->hit(); holder.values[0][0]->other();
  holder.pointers[0]->hit();
  holder.sequence.front()->other(); holder.owned[0]->other();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 4
    assert members["hit"]["receiver_types"] == ["dart::Item"]
    assert members["hit"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 4
    assert members["other"]["count"] == 4
    assert members["other"]["receiver_types"] == ["dart::Target"]


def test_type_using_declarations_respect_inherited_names_and_inner_blocks(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """namespace dart {
struct Item { Item(); void hit(); };
struct Base { struct Item { Item(); void own(); }; };
}
"""
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """using dart::Item;
struct Derived : dart::Base {
  void inherited() { Item value; value.own(); }
  void imported() { using dart::Item; Item value; value.hit(); }
};
""",
    )
    members = _entries(result, "members")
    assert members["own"]["receiver_types"] == ["dart::Base::Item"]
    assert members["hit"]["receiver_types"] == ["dart::Item"]
    assert members["own"]["unresolved_count"] == 0
    assert members["hit"]["unresolved_count"] == 0
    constructors = _entries(result, "constructors")
    assert constructors["dart::Base::Item"]["count"] == 1
    assert constructors["dart::Item"]["count"] == 1


def test_value_owning_std_array_constructor_dependencies(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text("namespace dart { struct A { A(); }; }")
    snippet = """#include <array>
#include <memory>
#include <vector>
using Value = dart::A;
using Pointer = dart::A*;
std::array<dart::A, 2> global;
struct Holder {
  std::array<Value, 2> member;
  static std::array<dart::A, 2> declarationOnly;
};
extern std::array<dart::A, 2> external;
void f(std::array<dart::A, 2> parameter,
       std::array<dart::A, 2>& reference,
       std::array<dart::A, 2>* pointer) {
  std::array<dart::A, 2> local;
  auto temporary = std::array<dart::A, 2>{};
  std::array<dart::A, 0> empty;
  std::array<dart::A, (0u)> parenthesizedEmpty;
  std::array<dart::A*, 2> rawPointers;
  std::array<Pointer, 2> aliasedPointers;
  std::array<std::shared_ptr<dart::A>, 2> smartPointers;
  std::array<std::vector<dart::A>, 2> vectors;
  std::vector<dart::A> vector;
}
"""
    original = _scan(extractor, extractor.index_api(headers), tmp_path, snippet)
    row = _entries(original, "constructors")["dart::A"]
    assert row["count"] == 4
    assert row["declarations"][0]["access"] == "public"
    header.write_text("namespace dart { class A { A(); }; }")
    changed = _scan(extractor, extractor.index_api(headers), tmp_path, snippet)
    assert (
        _entries(changed, "constructors")["dart::A"]["declarations"]
        != row["declarations"]
    )


@pytest.mark.parametrize(
    "snippet, count",
    [
        ("std::array<std::array<dart::A, 2>, 3> values;", 1),
        ("std::array<std::array<dart::A, 0>, 3> values;", 0),
        ("std::array<std::array<dart::A, 2>, 0> values;", 0),
        ("std::array<dart::A const, 2> values;", 1),
        ("std::array<const dart::A, 2> values;", 1),
        ("std::array<dart::A, 0x0ULL> values;", 0),
        ("std::array<dart::A, (1 < 2 ? 2 : 0)> values;", 1),
        ("std::array<dart::A, (1 > 2 ? 0 : 2)> values;", 1),
        ("std::array<dart::A, 2> values[3];", 1),
        ("auto values = new std::array<dart::A, 2>;", 1),
        ("extern std::array<dart::A, 2> values{};", 1),
        ("struct S { inline static std::array<dart::A, 2> values; };", 1),
        ("struct S : std::array<dart::A, 2> {};", 1),
        ("struct S : std::array<std::vector<dart::A>, 2> {};", 0),
        ("std::vector<std::array<dart::A, 2>> values;", 0),
        ("std::array<dart::A, 2> factory();", 0),
        ("void f(std::array<dart::A, 2>& values) {}", 0),
        ("std::array<dart::A, 2>* values;", 0),
    ],
)
def test_std_array_construction_shapes(extractor, tmp_path, snippet, count):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct A { A(); }; }")
    result = _scan(extractor, extractor.index_api(headers), tmp_path, snippet)
    assert _entries(result, "constructors").get("dart::A", {}).get("count", 0) == count


def test_c_style_cast_receivers_preserve_target_shape(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """namespace dart {
struct Target { void other(); };
struct A { void hit() const; A* get(); Target* operator->(); };
struct Callable { A* operator()(int); };
}
"""
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Foreign { void hit(); Foreign* make(); };
struct Derived : dart::A {};
using APtr = dart::A*;
void f(Foreign* raw, dart::A* known, dart::Callable factory) {
  ((dart::A*)raw)->hit();
  (((const dart::A*)(raw)))->hit();
  ((APtr)raw)->hit();
  ((Derived*)raw)->hit();
  ((dart::A&)*raw)->other();
  ((Derived&)*raw)->other();
  ((Foreign*)known)->hit();
  ((raw))->hit();
  ((raw)->make())->hit();
  ((known))->hit();
  (((known)->get()))->hit();
  ((factory)(1))->hit();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 7
    assert members["hit"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 2
    assert members["other"]["count"] == 2
    assert members["other"]["receiver_types"] == ["dart::Target"]


def test_grouped_function_declarators(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        """namespace dart {
struct A {
  virtual void start(), stop() const;
  A* first(), *second();
  A* raw(), value();
  int number(), *pointer();
  int field, getter();
  int method(), otherField;
  A(int x, int y) : x(x), y(y) {}
  int x, y;
};
void begin(), end();
}
"""
    )
    api = extractor.index_api(headers)
    assert set(api.members) >= {"start", "stop", "first", "second", "number", "pointer"}
    assert api.members["stop"][0]["const"]
    assert not api.members["start"][0]["const"]
    assert api.members["first"][0]["return_tokens"] == ["A", "*"]
    assert api.members["second"][0]["return_tokens"] == ["A", "*"]
    assert api.members["value"][0]["return_tokens"] == ["A"]
    assert len(api.constructors["dart::A"]) == 1
    assert set(api.data_members) >= {"field", "otherField"}
    assert set(api.members) >= {"getter", "method"}
    result = _scan(
        extractor,
        api,
        tmp_path,
        "void f(dart::A& a) { a.stop(); a.second()->start(); dart::end(); }",
    )
    members = _entries(result, "members")
    assert (
        members["stop"]["declarations"][0]["signature"] == "virtual void stop ( ) const"
    )
    assert members["start"]["unresolved_count"] == 0
    assert set(_entries(result, "free_functions")) == {"dart::end"}
    header.write_text(header.read_text().replace("stop() const", "stop(int) const"))
    changed = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void f(dart::A& a) { a.stop(); a.second()->start(); dart::end(); }",
    )
    assert (
        _entries(changed, "members")["stop"]["declarations"]
        != members["stop"]["declarations"]
    )


@pytest.mark.parametrize(
    "initializer",
    ["1 < 2", "low < high", "2 > 1", "high > low", "make<int, int>(1 < 2)"],
)
def test_grouped_declarations_with_initializer_comparisons(
    extractor, tmp_path, initializer
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { constexpr int low = 1, high = 2; "
        "template<class T, class U> int make(bool); "
        f"struct A {{ int first = {initializer}, second, getter(); }}; }}"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void f(dart::A& a) { a.second = a.getter(); }",
    )
    assert (
        _entries(result, "data_members")["second"]["declarations"][0]["signature"]
        == "int second"
    )
    assert (
        _entries(result, "members")["getter"]["declarations"][0]["signature"]
        == "int getter ( )"
    )


def test_source_derived_pointer_bindings_keep_builtin_arrow(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Target { void other(); }; "
        "struct A { void hit() const; Target* operator->(); }; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Derived : dart::A {};
struct Foreign { void hit(); };
using Ptr = Derived*;
using PtrAlias = Ptr;
using Value = Derived;
using ConstPtr = const Derived*;
using ConstPtrAlias = ConstPtr;
typedef const Derived* OldPtr;
void f(Derived* raw, Derived*& pointerRef, Ptr one, PtrAlias two,
       Derived& value, Value& valueAlias, Foreign* foreign,
       ConstPtr three, OldPtr four, Derived const* five, ConstPtrAlias six) {
  raw->hit(); pointerRef->hit(); one->hit(); two->hit();
  three->hit(); four->hit(); five->hit(); six->hit();
  value->other(); valueAlias->other(); foreign->hit();
  Derived* pointers[2]; pointers[0]->hit();
  PtrAlias matrix[2][3]; matrix[0][0]->hit();
  Derived values[2]; values[0]->other();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 10
    assert members["hit"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 3
    assert members["other"]["count"] == 3


@pytest.mark.parametrize("included", [False, True])
def test_source_class_aliases_keep_file_and_include_visibility(
    extractor, tmp_path, included
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct A { void hit(); }; }")
    declarations = "namespace n { struct Derived : dart::A {}; using Alias = Derived; }"
    snippets = {
        "a.cc": ('#include "types.hpp"' if included else declarations)
        + "\nvoid a(n::Alias value) { value.hit(); }",
        "b.cc": "namespace n { struct Foreign { void hit(); }; using Alias = Foreign; }"
        "\nvoid b(n::Alias value) { value.hit(); }",
    }
    if included:
        snippets["types.hpp"] = declarations
    result = _scan_files(extractor, extractor.index_api(headers), tmp_path, snippets)
    row = _entries(result, "members")["hit"]
    assert row["count"] == 1
    assert row["unresolved_count"] == 0
    assert row["examples"] == ["dartsim/src/a.cc:2"]


def test_hidden_friends_are_namespace_functions(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """namespace dart { namespace ns {
class A { friend void inspect(A&); };
struct External { void member(); };
struct FriendOwner { friend void External::member(); };
struct Holder {
  class Nested { friend void nested(Nested&) {} friend Nested* identity(Nested&); };
};
} }
"""
    )
    api = extractor.index_api(headers)
    assert "inspect" not in api.members
    assert "nested" not in api.members
    assert [entry["class"] for entry in api.members["member"]] == ["dart::ns::External"]
    assert "dart::ns::inspect" in api.free_functions
    assert "dart::ns::nested" in api.free_functions
    assert api.free_functions["dart::ns::identity"][0]["declaration_scope"] == (
        "dart",
        "ns",
        "Holder",
        "Nested",
    )
    result = _scan(
        extractor,
        api,
        tmp_path,
        "void f(dart::ns::A& a, dart::ns::Holder::Nested& n) { inspect(a); nested(n); }",
    )
    assert set(_entries(result, "free_functions")) == {
        "dart::ns::inspect",
        "dart::ns::nested",
    }


def test_calls_counts_classification_and_ignored_text(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """namespace dyn = dart::dynamics;
void example(dyn::Widget* pointer, dyn::Widget value) {
  pointer->getValue();
  value.getValue();
  value.compute();
  pointer->isReady();
  pointer->setValue(1);
  pointer->getFriction();
  pointer->setFriction(0.5);
  pointer->getAndReset();
  pointer->createShapeNodeWith<dyn::Widget>();
  dyn::Widget::create();
  // pointer->commentedOnly();
  /* value.commentedOnly(); dart::dynamics::Unused unused; */
  const char* text = "pointer->commentedOnly(); DART_VERSION_AT_LEAST(1,2,3)";
  const char* raw = R"cpp(
    value.commentedOnly(); dart::dynamics::Unused unused;
  )cpp";
  char quote = '\\"';
  pointer->privateOnly();
}
""",
    )
    members = _entries(result, "members")
    assert members["getValue"]["count"] == 2
    assert members["getValue"]["examples"] == [
        "dartsim/src/sample.cc:3",
        "dartsim/src/sample.cc:4",
    ]
    for name in ("getValue", "compute", "isReady", "getFriction"):
        assert members[name]["classification"] == "accessor"
    for name in (
        "setValue",
        "getAndReset",
        "createShapeNodeWith",
        "create",
        "setFriction",
    ):
        assert members[name]["classification"] == "mutation"
    assert "commentedOnly" not in members
    assert "privateOnly" not in members
    assert not result["macros"]
    assert "dart::dynamics::Widget" in _entries(result, "types")
    assert "dart::dynamics::Unused" not in _entries(result, "types")
    declarations = members["compute"]["declarations"]
    assert any(d["const"] and not d["virtual"] for d in declarations)


def test_macros_and_free_functions(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """#if DART_VERSION_AT_LEAST(6, 13, 0)
auto answer = dart::math::measure(1.0);
#endif
#if DART_UNDECLARED_CAPABILITY
#endif
#define DART_LOCAL_FLAG
#if DART_LOCAL_FLAG
#endif
// DART_VERSION_AT_LEAST(1, 2, 3)
""",
    )
    macro = _entries(result, "macros")["DART_VERSION_AT_LEAST"]
    assert macro["count"] == 1
    assert macro["examples"] == ["dartsim/src/sample.cc:1"]
    assert macro["declared_in_dart"]
    macros = _entries(result, "macros")
    assert not macros["DART_UNDECLARED_CAPABILITY"]["declared_in_dart"]
    assert macros["DART_LOCAL_FLAG"]["count"] == 1
    function = _entries(result, "free_functions")["dart::math::measure"]
    assert function["count"] == 1


def test_direct_free_function_outside_the_plugin(extractor, api, tmp_path):
    (tmp_path / "use.cc").write_text(
        "double use() { return dart::math::measure(1.0); }\n"
    )
    result = extractor.scan_sources(api, tmp_path)
    assert _entries(result, "free_functions")["dart::math::measure"]["count"] == 1


def test_type_named_only_as_static_qualifier(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        "void example() { dart::dynamics::StaticOnly::mutate(); }\n",
    )
    assert "dart::dynamics::StaticOnly" in _entries(result, "types")
    assert _entries(result, "members")["mutate"]["count"] == 1


def test_macro_declaration_changes_reach_the_inventory(extractor, api, tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    (source / "use.cc").write_text(
        "void use(dart::dynamics::Widget* w) { w->setFriction(1.0); }\n"
    )
    original = extractor.scan_sources(api, source)
    header = tmp_path / "dart" / "config.hpp.in"
    header.write_text(header.read_text().replace("TYPE value", "TYPE value, int flags"))
    changed = extractor.scan_sources(extractor.index_api(header.parent), source)
    assert _entries(original, "members")["setFriction"]["declarations"] != (
        _entries(changed, "members")["setFriction"]["declarations"]
    )


def test_dart_pointer_methods_and_standard_pointer_methods(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """struct Holder {
  dart::dynamics::WidgetSharedPtr value;
  void method(dart::dynamics::WidgetPtr value);
};
void field(Holder* holder) { holder->value.get(); }
void use(::dart::dynamics::WidgetPtr custom,
         const ::dart::dynamics::WidgetSharedPtr standard,
         const ::dart::dynamics::Widget* widget) {
  custom.get();
  custom->getValue();
  standard.get();
  standard->getValue();
  widget->get();
}
""",
    )
    members = _entries(result, "members")
    assert members["get"]["count"] == 2
    assert members["getValue"]["count"] == 2


def test_using_declarations_and_unrelated_receivers(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """using namespace dart::dynamics;
using WidgetAlias = dart::dynamics::Widget;
void example(Widget* widget) {
  widget->clear();
  widget->getValue();
  std::vector<int> values;
  values.clear();
  WidgetAlias::create();
  WidgetAlias::createShared();
}
void first(Widget* shared) { shared->getValue(); }
void second(std::vector<int> shared) { shared.clear(); }
class Local { public: void clear(); };
void third(Local shared) { shared.clear(); }
void exported(dart::collision::Exported* detector) { detector->setExported(2); }
""",
    )
    members = _entries(result, "members")
    assert members["clear"]["count"] == 1
    assert members["getValue"]["count"] == 2
    assert members["create"]["count"] == 1
    assert members["createShared"]["count"] == 1
    assert members["setExported"]["count"] == 1
    assert "dart::dynamics::Widget" in _entries(result, "types")


def test_inherited_and_protected_virtual_overrides(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """class GzDetector : public dart::collision::DerivedDetector {
public:
  bool raycast(const dart::dynamics::Widget& widget) const override;
  static void create();
protected:
  void refresh(dart::dynamics::Widget* widget) override;
private:
  static Registrar<GzDetector> registration;
};
class Local { public: virtual void localOnly(); };
class GzLocal : public Local { public: void localOnly() override; };
void use() { GzDetector::getStaticType(); GzDetector::create(); }
""",
    )
    overrides = _entries(result, "overrides")
    assert set(overrides) == {"GzDetector::raycast", "GzDetector::refresh"}
    for entry in overrides.values():
        assert entry["count"] == 1
        assert entry["signatures"]
        assert all(d["virtual"] for d in entry["declarations"])
    assert any(d["const"] for d in overrides["GzDetector::raycast"]["declarations"])
    assert "dart::collision::Detector::Registrar" in _entries(result, "types")
    members = _entries(result, "members")
    assert members["getStaticType"]["count"] == 1
    assert "create" not in members


def test_implicit_virtual_overrides(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """using Widget = dart::dynamics::Widget;
class GzImplicit : public dart::collision::DerivedDetector {
public:
  bool raycast(const Widget& target) const;
protected:
  void refresh(Widget* target);
};
class GzOverloads : public dart::collision::DerivedDetector {
public:
  bool raycast(Widget& target) const;
  bool raycast(const Widget& target);
  void refresh(const Widget* target);
};
""",
    )
    overrides = _entries(result, "overrides")
    assert set(overrides) == {"GzImplicit::raycast", "GzImplicit::refresh"}
    assert all(entry["count"] == 1 for entry in overrides.values())


def test_implicit_override_parameter_and_member_qualifiers(extractor, tmp_path):
    headers = tmp_path / "headers"
    headers.mkdir()
    (headers / "base.hpp").write_text(
        """namespace dart {
class Base {
public:
  virtual void update(int value = 1) const & = 0;
  virtual void reset(unsigned int value) = 0;
  virtual void byValue(const int value) = 0;
  virtual void ping() volatile && = 0;
};
}
"""
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """class Good : public dart::Base {
public:
  void update(int renamed = 2) const &;
  void reset(unsigned int);
  void byValue(int);
  void ping() volatile &&;
};
class Wrong : public dart::Base {
public:
  void update(int renamed) const &&;
  void reset(unsigned int) const;
  void byValue(int&);
  void ping() &&;
};
""",
    )
    assert set(_entries(result, "overrides")) == {
        "Good::update",
        "Good::reset",
        "Good::byValue",
        "Good::ping",
    }


def test_explicit_virtual_override_retains_pointer_alias_candidates(
    extractor, api, tmp_path
):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """using WidgetPointer = dart::dynamics::Widget*;
class ExplicitOverride : public dart::collision::Detector {
  void refresh(WidgetPointer target) override;
};
class ExplicitVirtual : public dart::collision::Detector {
  virtual void refresh(WidgetPointer target);
};
""",
    )
    overrides = _entries(result, "overrides")
    assert set(overrides) == {"ExplicitOverride::refresh", "ExplicitVirtual::refresh"}
    assert all(entry["count"] == 1 for entry in overrides.values())


def test_constructions_record_constructor_declarations(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """struct Holder {
  dart::dynamics::Widget member;
  dart::dynamics::WidgetSharedPtr shared;
};
class Derived : public dart::dynamics::Widget {
public:
  Derived() : dart::dynamics::Widget(2) {}
};
class Inherited : public dart::dynamics::Widget {
public:
  using dart::dynamics::Widget::Widget;
};
dart::dynamics::Widget make(
    const dart::dynamics::Widget& reference, dart::dynamics::Widget copy);
namespace a { using T = dart::dynamics::Widget; }
namespace b { using T = std::shared_ptr<dart::dynamics::Widget>; }
using Pointer = std::shared_ptr<dart::collision::Detector>;
using Chained = Pointer;
void build() {
  dart::dynamics::Widget local(1);
  dart::dynamics::Widget braced{1};
  auto temporary = dart::dynamics::Widget(1);
  auto pointer = new dart::dynamics::Widget(1);
  auto shared = std::make_shared<dart::dynamics::Widget>(1);
  dart::dynamics::Widget* raw = nullptr;
  std::vector<dart::dynamics::Widget> widgets;
  dart::dynamics::WidgetSharedPtr none;
  dart::dynamics::StaticOnly implicit;
  dart::collision::Option option;
  if (dart::dynamics::Widget scoped{1}; true) {}
  a::T direct;
  b::T wrapper;
  Chained chained;
}
""",
    )
    constructors = _entries(result, "constructors")
    assert set(constructors) == {
        "dart::collision::Option",
        "dart::dynamics::StaticOnly",
        "dart::dynamics::Widget",
    }
    widget = constructors["dart::dynamics::Widget"]
    assert widget["count"] == 12  # including the two base clauses
    assert [d["signature"] for d in widget["declarations"]] == [
        "Widget ( )",
        "explicit Widget ( int value )",
    ]
    assert constructors["dart::dynamics::StaticOnly"]["declarations"] == []
    assert [
        d["signature"] for d in constructors["dart::collision::Option"]["declarations"]
    ] == ["Option ( bool enable = true )"]

    header = tmp_path / "dart" / "api.hpp"
    header.write_text(
        header.read_text().replace(
            "Widget(int value)", "Widget(double scale, int value)"
        )
    )
    changed = extractor.scan_sources(
        extractor.index_api(header.parent), tmp_path / "source"
    )
    assert (
        _entries(changed, "constructors")["dart::dynamics::Widget"]["declarations"]
        != widget["declarations"]
    )


def test_implicit_constructions_reach_base_constructors(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """class Handler : public dart::dynamics::Solver { public: void extra(); };
void build() {
  auto handler = std::make_shared<Handler>();
  auto solver = std::make_shared<dart::dynamics::DerivedSolver>();
}
struct Tuple : std::tuple<int, dart::dynamics::Solver, bool> {};
template <bool B = (sizeof(int) < sizeof(long))>
struct Checked : public dart::dynamics::Solver {};
""",
    )
    constructors = _entries(result, "constructors")
    solver = constructors["dart::dynamics::Solver"]
    assert solver["examples"] == [
        "dartsim/src/sample.cc:1",
        "dartsim/src/sample.cc:6",
        "dartsim/src/sample.cc:8",
    ]
    derived = constructors["dart::dynamics::DerivedSolver"]
    assert [(d["class"], d["signature"]) for d in derived["declarations"]] == [
        ("dart::dynamics::Solver", "explicit Solver ( int flags = 0 )")
    ]

    header = tmp_path / "dart" / "api.hpp"
    header.write_text(header.read_text().replace("int flags = 0", "int flags"))
    changed = _entries(
        extractor.scan_sources(extractor.index_api(header.parent), tmp_path / "source"),
        "constructors",
    )
    for name in ("dart::dynamics::Solver", "dart::dynamics::DerivedSolver"):
        assert changed[name]["declarations"] != constructors[name]["declarations"]


def test_constructions_spelled_from_the_global_namespace(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """struct Holder {
  Holder() : widget(new ::dart::dynamics::Widget) {}
  dart::dynamics::Widget* widget;
};
auto shared = std::make_shared<::dart::dynamics::Widget>(1);
class Inherited : public ::dart::dynamics::Widget {
  using ::dart::dynamics::Widget::Widget;
};
""",
    )
    assert _entries(result, "constructors")["dart::dynamics::Widget"]["count"] == 4


def test_declarations_are_not_constructions(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """class Holder {
  dart::dynamics::Widget makeWidget() const;
  static dart::dynamics::Widget sWidget;
public:
  dart::dynamics::Widget makePublic() const;
};
typedef const dart::dynamics::Widget ConstWidget;
extern dart::dynamics::Widget gWidget;
dart::dynamics::Widget Holder::sWidget{1};
class GzDetector {
  static dart::collision::Detector::Registrar<GzDetector> sRegistrar;
};
class Derived : public dart::collision::DerivedDetector {
  static Registrar<Derived> sRegistrar;
};
Derived::Registrar<Derived> Derived::sRegistrar{Derived::getStaticType(), []() { return 0; }};
extern dart::dynamics::Widget gDefined{};
extern void use(dart::dynamics::Widget w = dart::dynamics::Widget(1));
""",
    )
    constructors = _entries(result, "constructors")
    assert constructors["dart::dynamics::Widget"]["examples"] == [
        "dartsim/src/sample.cc:17",
        "dartsim/src/sample.cc:18",
        "dartsim/src/sample.cc:9",
    ]
    assert constructors["dart::collision::Detector::Registrar"]["examples"] == [
        "dartsim/src/sample.cc:16"
    ]
    assert _entries(result, "members")["getStaticType"]["count"] == 1


@pytest.mark.parametrize(
    ("definition", "use", "section", "entry", "owners"),
    [
        (
            "constexpr int BASE = 1; constexpr int DEFAULT = BASE * 2; void run(int = DEFAULT);",
            "dart::run();",
            "free_functions",
            "dart::run",
            {"dart"},
        ),
        (
            "struct A { static constexpr int DEFAULT = 1; void run(int = DEFAULT); };",
            "dart::A a; a.run();",
            "members",
            "run",
            {"dart::A"},
        ),
        (
            "class A { static constexpr int DEFAULT = 1; public: void run(int = DEFAULT); };",
            "dart::A a; a.run();",
            "members",
            "run",
            {"dart::A"},
        ),
        (
            "struct Base { static constexpr int DEFAULT = 1; }; struct A : Base { void run(int = DEFAULT); };",
            "dart::A a; a.run();",
            "members",
            "run",
            {"dart::Base"},
        ),
        (
            "enum class Mode { Default = 1 }; void run(Mode = Mode::Default);",
            "dart::run();",
            "free_functions",
            "dart::run",
            {"dart::Mode"},
        ),
        (
            "constexpr int DEFAULT = 1; struct A { A(int = DEFAULT); };",
            "dart::A a;",
            "constructors",
            "dart::A",
            {"dart"},
        ),
        (
            "struct A { static constexpr int DEFAULT = 1; int operator()(int = DEFAULT); };",
            "dart::A a; a();",
            "members",
            "operator()",
            {"dart::A"},
        ),
        (
            "class A { static constexpr int DEFAULT = 1; friend void run(A&, int = DEFAULT) {} };",
            "dart::A a; run(a);",
            "free_functions",
            "dart::run",
            {"dart::A"},
        ),
    ],
)
def test_used_defaults_retain_constant_dependencies(
    extractor, tmp_path, definition, use, section, entry, owners
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text("namespace dart { " + definition + " }")
    before = _scan(
        extractor, extractor.index_api(headers), tmp_path, "void f() { " + use + " }"
    )
    row = _entries(before, section)[entry]
    dependencies = (
        row.get("default_argument_declarations")
        or row["declarations"][0]["default_argument_declarations"]
    )
    assert {d["class"] for d in dependencies} == owners
    if "BASE * 2" in definition:
        assert len(dependencies) == 2
    header.write_text(header.read_text().replace("= 1", "= 2"))
    after = extractor.scan_sources(extractor.index_api(headers), tmp_path / "source")
    assert after != before
    assert _entries(after, section)[entry]["count"] == row["count"]
    assert not before["variables"]  # dependency evidence is not a source use


def test_default_constant_lookup_excludes_foreign_and_shadowed_names(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace foreign { constexpr int DEFAULT = 1; } "
        "namespace dart { constexpr int DEFAULT = 99; "
        "void foreignDefault(int = foreign::DEFAULT); "
        "namespace inner { constexpr int DEFAULT = 2; void run(int = DEFAULT); } "
        "namespace mutableScope { int DEFAULT = 2; void run(int = DEFAULT); } "
        "struct A { static int DEFAULT; void run(int = DEFAULT); }; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void f() { dart::foreignDefault(); dart::inner::run(); "
        "dart::mutableScope::run(); dart::A a; a.run(); }",
    )
    functions = _entries(result, "free_functions")
    assert "default_argument_declarations" not in functions["dart::foreignDefault"]
    assert "default_argument_declarations" not in functions["dart::mutableScope::run"]
    assert (
        "default_argument_declarations"
        not in _entries(result, "members")["run"]["declarations"][0]
    )
    dependencies = functions["dart::inner::run"]["default_argument_declarations"]
    assert [(d["class"], d["signature"]) for d in dependencies] == [
        ("dart::inner", "constexpr int DEFAULT = 2")
    ]


def test_default_constant_initializer_cycles_are_bounded(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { extern const int B; const int A = B; "
        "const int B = A; void run(int = A); }"
    )
    result = _scan(
        extractor, extractor.index_api(headers), tmp_path, "void f() { dart::run(); }"
    )
    dependencies = _entries(result, "free_functions")["dart::run"][
        "default_argument_declarations"
    ]
    assert {d["signature"] for d in dependencies} == {
        "extern const int B",
        "const int A = B",
        "const int B = A",
    }


def test_default_arguments_do_not_start_member_initializers(extractor, api, tmp_path):
    header = tmp_path / "dart" / "api.hpp"
    header.write_text(
        header.read_text().replace(
            "class Unused {};",
            "class Ternary {\npublic:\n"
            "  int pick(int n = (true) ? 1 : 2) const { return n; }\n"
            "};\nclass Unused {};",
        )
    )
    result = _scan(
        extractor,
        extractor.index_api(header.parent),
        tmp_path,
        "void f(dart::dynamics::Ternary* t) { t->pick(); }\n",
    )
    assert _entries(result, "members")["pick"]["count"] == 1


def test_constructors_of_class_template_specializations(extractor, api, tmp_path):
    result = _scan(
        extractor, api, tmp_path, "void f() { dart::dynamics::Props props(1); }\n"
    )
    declarations = _entries(result, "constructors")["dart::dynamics::Props"][
        "declarations"
    ]
    assert {d["signature"] for d in declarations} == {
        "Compose ( ) = default",
        "Compose ( )",
        "explicit Compose ( A value )",
    }


def test_nested_types_of_aliased_classes(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """namespace gz {
struct Base { using DartWidget = dart::dynamics::Widget; };
struct Feature : public virtual Base { void make(); };
void Feature::make() {
  DartWidget::Properties properties;
  properties.mName = 1;
}
}
""",
    )
    constructors = _entries(result, "constructors")
    assert constructors["dart::dynamics::Widget::Properties"]["examples"] == [
        "dartsim/src/sample.cc:5"
    ]
    assert _entries(result, "data_members")["mName"]["receiver_types"] == [
        "dart::dynamics::Widget::Properties"
    ]


def test_fields_of_dart_bases_through_implicit_this(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """class Custom : public dart::dynamics::Widget {
public:
  void inlineUse() { mDirty = true; }
  Custom();
  void shadowed();
  void conditional();
  int mCount;
};
Custom::Custom() : mCount{0} { mDirty = false; }
void Custom::shadowed() { bool mDirty = false; mDirty = true; }
void Custom::conditional() {
#ifdef LOCAL
  bool mDirty = false;
#endif
  mDirty = true;
}
void unrelated(bool mDirty) { mDirty = true; }
class Other { bool mDirty; void use() { mDirty = true; } };
""",
    )
    field = _entries(result, "data_members")["mDirty"]
    assert field["examples"] == [
        "dartsim/src/sample.cc:15",
        "dartsim/src/sample.cc:3",
        "dartsim/src/sample.cc:9",
    ]
    assert field["receiver_types"] == ["dart::dynamics::Widget"]


def test_third_party_objects_are_not_dart_receivers(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """void use(const dart::collision::Contact& real) {
  dContactGeom contact;
  double x = contact.point[0];
  aiScene* scene = new aiScene;
  scene->mMeshes[0]->point = nullptr;
  double y = real.point[0];
}
""",
    )
    point = _entries(result, "data_members")["point"]
    assert point["examples"] == ["dartsim/src/sample.cc:6"]
    assert point["unresolved_count"] == 0


def test_public_data_member_accesses(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """void use(dart::collision::Option& option,
         const dart::collision::Contact* contact) {
  option.enableContact = false;
  option.maxNumContacts = 2;
  double depth = contact->penetrationDepth;
  auto point = contact->point;
  contact->point.norm();
  option.hidden = 1;
  option.exported->setExported(1);
  int limit = dart::dynamics::Widget::kLimit;
}
struct Local { bool enableContact; };
void local(Local value) { value.enableContact = true; }
""",
    )
    members = _entries(result, "data_members")
    assert set(members) == {
        "enableContact",
        "exported",
        "kLimit",
        "maxNumContacts",
        "penetrationDepth",
        "point",
    }
    assert members["kLimit"]["receiver_types"] == ["dart::dynamics::Widget"]
    assert members["kLimit"]["declarations"][0]["const"]
    assert members["enableContact"]["count"] == 1
    assert members["point"]["count"] == 2
    assert members["enableContact"]["receiver_types"] == ["dart::collision::Option"]
    assert members["enableContact"]["declarations"] == [
        {
            "class": "dart::collision::Option",
            "access": "public",
            "const": False,
            "signature": "bool enableContact",
            "virtual": False,
        }
    ]
    assert members["penetrationDepth"]["declarations"][0]["class"] == (
        "dart::collision::Contact"
    )

    header = tmp_path / "dart" / "api.hpp"
    header.write_text(header.read_text().replace("enableContact;", "contactOn;"))
    renamed = extractor.scan_sources(
        extractor.index_api(header.parent), tmp_path / "source"
    )
    assert "enableContact" not in _entries(renamed, "data_members")


def test_aliases_are_scoped_to_their_files_and_namespaces(extractor, api, tmp_path):
    result = _scan_files(
        extractor,
        api,
        tmp_path,
        {
            "a.cc": "using Alias = dart::dynamics::Widget;\n"
            "void a() { Alias::getValue(); }\n",
            "b.cc": "using Alias = dart::collision::Detector;\n"
            "void b() { Alias::getStaticType(); }\n",
            "c.cc": """namespace first {
using Alias = dart::dynamics::Widget;
void f() { Alias::isReady(); }
}
namespace second {
using Alias = dart::collision::Detector;
void g() { Alias::getStaticType(); }
}
""",
            "d.hh": """namespace gz {
struct Base { using DartWidget = dart::dynamics::Widget; };
class GZ_VISIBLE Feature : public virtual Base { void e(); };
}
""",
            "e.cc": '#include "d.hh"\n'
            "namespace gz { void Feature::e() { DartWidget::compute(); } }\n",
            "f.cc": """namespace gz {
struct Other : Base { void f(); };
void Other::f() { DartWidget::clear(); }
}
""",
            "g.cc": """using Inner = dart::collision::Detector;
namespace inner {
using Inner = dart::dynamics::Widget;
void g() { Inner::setValue(1); }
}
""",
            "h.cc": """namespace gz {
struct S { using Ptr = dart::dynamics::Widget; void f(); };
struct T { using Ptr = dart::collision::Detector; void g(); };
void S::f() { Ptr::create(); }
void T::g() { Ptr::create(); }
}
""",
            "i.cc": """namespace first { using Alias = dart::dynamics::Widget; }
namespace third {
struct Alias { static void clear(); };
void h() { Alias::clear(); }
}
""",
            "j.cc": """namespace gz { struct U { using W = dart::dynamics::Widget; void f(); void g(); }; }
using namespace gz;
void U::f() { W::getAndReset(); }
namespace gz { void gz::U::g() { W::setFriction(1.0); } }
""",
            "k.cc": """namespace gz {
struct Root { using W = dart::dynamics::Widget; };
using Parent = Root;
struct Leaf : Parent { void f(); };
void Leaf::f() { W::getFriction(); }
}
""",
        },
    )
    members = _entries(result, "members")
    assert members["getValue"]["examples"] == ["dartsim/src/a.cc:2"]
    assert members["isReady"]["examples"] == ["dartsim/src/c.cc:3"]
    assert members["getStaticType"]["count"] == 2
    assert members["compute"]["examples"] == ["dartsim/src/e.cc:2"]
    assert members["setValue"]["examples"] == ["dartsim/src/g.cc:4"]
    assert members["create"]["receiver_types"] == [
        "dart::collision::Detector",
        "dart::dynamics::Widget",
    ]
    assert "clear" not in members
    assert members["getAndReset"]["examples"] == ["dartsim/src/j.cc:3"]
    assert members["setFriction"]["examples"] == ["dartsim/src/j.cc:4"]
    assert members["getFriction"]["examples"] == ["dartsim/src/k.cc:5"]


def test_inactive_and_alternative_preprocessor_branches(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """void hidden(dart::dynamics::Widget* w) {
#if 0
  dart::collision::Detector* w = nullptr;
#if DART_VERSION_AT_LEAST(1, 2, 3)
  dart::dynamics::Unused unused;
#endif
#else
  w->getValue();
#endif
}
#if 1
void kept(dart::dynamics::Widget* k) { k->setValue(1); }
#elif USE_OTHER
void dropped(dart::dynamics::Widget* k) { k->clear(); }
#endif
void alternatives() {
#if USE_WIDGET
  dart::dynamics::Widget* x = nullptr;
#else
  dart::collision::Detector* x = nullptr;
#endif
  x->isReady();
}
void separate() {
  dart::dynamics::Widget* z = nullptr;
  {
#ifdef USE_DETECTOR
    dart::collision::Detector* z = nullptr;
#else
    z->compute();
#endif
  }
}
void optional(dart::dynamics::Widget* y) {
  {
#ifdef USE_DETECTOR
    dart::collision::Detector* y = nullptr;
#endif
    y->getAndReset();
  }
}
""",
    )
    members = _entries(result, "members")
    assert members["getValue"]["receiver_types"] == ["dart::dynamics::Widget"]
    assert members["setValue"]["count"] == 1
    assert "clear" not in members
    assert members["isReady"]["receiver_types"] == [
        "dart::collision::Detector",
        "dart::dynamics::Widget",
    ]
    assert members["compute"]["receiver_types"] == ["dart::dynamics::Widget"]
    # Builds without USE_DETECTOR call it on the parameter.
    assert members["getAndReset"]["receiver_types"] == [
        "dart::collision::Detector",
        "dart::dynamics::Widget",
    ]
    assert "DART_VERSION_AT_LEAST" not in _entries(result, "macros")
    assert "dart::dynamics::Unused" not in _entries(result, "types")


def test_imported_free_functions(extractor, api, tmp_path):
    result = _scan_files(
        extractor,
        api,
        tmp_path,
        {
            "directive.cc": "using namespace dart::math;\nvoid a() { measure(1.0); }\n",
            "declaration.cc": "using dart::math::measure;\n"
            "void b() { measure(2.0); }\n",
            "partial.cc": "using namespace dart;\nvoid c() { math::measure(3.0); }\n",
            "unrelated.cc": "void d() { measure(4.0); }\n",
            "member.cc": "using namespace dart::dynamics;\nvoid e() { getValue(); }\n",
        },
    )
    functions = _entries(result, "free_functions")
    assert set(functions) == {"dart::math::measure"}
    assert functions["dart::math::measure"]["examples"] == [
        "dartsim/src/declaration.cc:2",
        "dartsim/src/directive.cc:2",
        "dartsim/src/partial.cc:2",
    ]

    scoped = _scan_files(
        extractor,
        api,
        tmp_path / "scoped",
        {
            "imports.hh": "using namespace dart::math;\n",
            "header.cc": '#include "imports.hh"\nvoid f() { measure(5.0); }\n',
            "shadowed.cc": """using dart::math::measure;
namespace local { double measure(double); void g() { measure(6.0); } }
void h() { measure(7.0); }
""",
        },
    )
    assert _entries(scoped, "free_functions")["dart::math::measure"]["examples"] == [
        "dartsim/src/header.cc:2",
        "dartsim/src/shadowed.cc:3",
    ]

    overloads = _scan_files(
        extractor,
        api,
        tmp_path / "overloads",
        {
            # A using-declaration adds to the overloads of its own scope.
            "same.cc": """namespace gz {
using dart::math::measure;
double measure(int);
void f() { measure(8.0); }
}
""",
            # A block's using-declaration hides the enclosing namespace's.
            "block.cc": """namespace gz {
double measure(int);
void f() {
  using dart::math::measure;
  measure(9.0);
}
}
""",
            "renamed.cc": "namespace dm = dart::math;\n"
            "using namespace dm;\nvoid f() { measure(10.0); }\n",
            # A member hides the imported function in out-of-line definitions.
            "member.cc": "struct S { double measure(double); void f(); };\n"
            "using namespace dart::math;\nvoid S::f() { measure(11.0); }\n",
            # So does a nearer using-declaration of another function.
            "nearer.cc": """namespace other { double measure(double); }
using dart::math::measure;
namespace gz { using other::measure; void f() { measure(14.0); } }
""",
            "nested.cc": """namespace other { double measure(double); }
void f() {
  using dart::math::measure;
  {
    using other::measure;
    measure(16.0);
  }
}
""",
        },
    )
    function = _entries(overloads, "free_functions")["dart::math::measure"]
    assert function["count"] == 3
    assert function["examples"] == [
        "dartsim/src/block.cc:5",
        "dartsim/src/renamed.cc:3",
        "dartsim/src/same.cc:4",
    ]

    # Code inside a DART namespace finds its functions without imports.
    enclosed = _scan(
        extractor,
        api,
        tmp_path / "enclosed",
        """namespace dart {
namespace math { void f() { measure(12.0); } }
void g() { math::measure(13.0); }
}
namespace dart { using math::measure; void h() { measure(15.0); } }
""",
    )
    assert _entries(enclosed, "free_functions")["dart::math::measure"]["count"] == 3

    # Declarations in one scope form one overload set.
    header = tmp_path / "dart" / "api.hpp"
    header.write_text(
        header.read_text().replace(
            "namespace math {",
            "namespace other {\ndouble measure(int);\n}\nnamespace math {",
        )
    )
    both = _scan(
        extractor,
        extractor.index_api(header.parent),
        tmp_path / "both",
        "using dart::other::measure;\nusing dart::math::measure;\n"
        "void f() { measure(1.0); }\n",
    )
    assert set(_entries(both, "free_functions")) == {
        "dart::math::measure",
        "dart::other::measure",
    }


def test_argument_dependent_free_function_lookup(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """namespace dart {
namespace dynamics {
struct Widget {};
void inspect(const Widget&);
struct Holder { struct Nested {}; };
void nested(const Holder::Nested&);
}
namespace collision {
struct Derived : dynamics::Widget {};
void inspect(const Derived&);
}
namespace unrelated { void inspect(int); }
}
"""
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """void f(dart::dynamics::Widget& value,
          dart::collision::Derived* derived,
          dart::dynamics::Holder::Nested value2) {
  inspect(value);
  inspect(*derived);
  inspect(dart::dynamics::Widget());
  nested(value2);
  inspect(1);
}
struct S {
  void inspect(const dart::dynamics::Widget&);
  void g(dart::dynamics::Widget value) { inspect(value); }
};
""",
    )
    functions = _entries(result, "free_functions")
    assert set(functions) == {
        "dart::dynamics::inspect",
        "dart::collision::inspect",
        "dart::dynamics::nested",
    }
    assert functions["dart::dynamics::inspect"]["count"] == 3
    assert functions["dart::collision::inspect"]["count"] == 1
    assert functions["dart::dynamics::nested"]["count"] == 1


def test_namespace_variables_preserve_declarations(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        """namespace dart {
constexpr int LIMIT = 3;
namespace dynamics {
inline constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();
extern int current;
struct Unused { static const int INVALID_INDEX = 1; };
}
}
namespace other { constexpr int LIMIT = 1; }
"""
    )
    source = """void f() {
  auto limit = dart::LIMIT;
  auto index = dart::dynamics::INVALID_INDEX;
  dart::dynamics::current = 1;
  auto other = other::LIMIT;
}
"""
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    variables = _entries(result, "variables")
    assert set(variables) == {
        "dart::LIMIT",
        "dart::dynamics::INVALID_INDEX",
        "dart::dynamics::current",
    }
    assert all(row["count"] == 1 and row["declarations"] for row in variables.values())
    assert variables["dart::dynamics::INVALID_INDEX"]["declarations"][0]["const"]
    assert not variables["dart::dynamics::current"]["declarations"][0]["const"]
    assert not result["data_members"]
    header.write_text(header.read_text().replace("LIMIT = 3", "LIMIT = 4"))
    changed = _scan(
        extractor, extractor.index_api(headers), tmp_path / "changed", source
    )
    assert (
        _entries(changed, "variables")["dart::LIMIT"]["declarations"]
        != variables["dart::LIMIT"]["declarations"]
    )


@pytest.mark.parametrize("definition", [";", "{};"])
def test_referenced_class_template_parameters_are_recorded(
    extractor, tmp_path, definition
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { template<class T> class Box" + definition + " }"
    )
    source = "void f(dart::Box<int>* box) {}"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(before, "types")["dart::Box"]["template_declarations"] == [
        "template < class T > class Box"
    ]
    header.write_text(header.read_text().replace("class T>", "class T, class U>"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "changed", source)
    assert before != after


def test_template_defaults_in_forward_headers_and_comparisons(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    forward = headers / "forward.hpp"
    forward.write_text("namespace dart { template<class T, class U = int> class Box; }")
    (headers / "api.hpp").write_text(
        """namespace dart {
template<class T, class U> class Box {};
template<bool B = (sizeof(int) < sizeof(long))> class Compared {};
}
"""
    )
    source = "void f(dart::Box<int>* box, dart::Compared<>* compared) {}"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    types = _entries(before, "types")
    assert len(types["dart::Box"]["template_declarations"]) == 2
    assert types["dart::Compared"]["template_declarations"] == [
        "template < bool B = ( sizeof ( int ) < sizeof ( long ) ) > class Compared"
    ]
    forward.write_text(forward.read_text().replace("= int", "= double"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "changed", source)
    assert before != after


def test_attributes_do_not_hide_real_declarations(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """namespace dart {
class Mesh {
public:
  [[deprecated("old")]] Mesh(int value);
  [[vendor::hint(choose(1, array[0]))]] int getMesh() const;
  [[deprecated("old")]] void setMesh(int value);
  int operator[](int value) const;
};
}
"""
    )
    api = extractor.index_api(headers)
    assert set(api.members) == {"getMesh", "setMesh", "operator[]"}
    assert len(api.constructors["dart::Mesh"]) == 1
    result = _scan(
        extractor,
        api,
        tmp_path,
        "void f() { dart::Mesh mesh(1); mesh.getMesh(); mesh.setMesh(2); mesh[0]; }",
    )
    assert set(_entries(result, "members")) == {"getMesh", "setMesh", "operator[]"}
    assert _entries(result, "constructors")["dart::Mesh"]["declarations"]


def test_free_function_returns_resolve_chained_receivers(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """namespace dart {
class Widget { public: void mutate(); int getValue() const; };
Widget* make();
Widget makeValue();
}
"""
    )
    result = _scan_files(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        {
            "../client.cc": "void f() { dart::make()->mutate(); dart::makeValue().getValue(); }"
        },
    )
    members = _entries(result, "members")
    assert set(members) == {"mutate", "getValue"}
    assert all(row["receiver_types"] == ["dart::Widget"] for row in members.values())
    assert all(row["unresolved_count"] == 0 for row in members.values())


def test_direct_dart_includes_detect_removed_headers(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "empty.hpp"
    header.write_text("// Header with no indexed declarations.\n")
    (headers / "config.hpp.in").write_text("// Generated installed header.\n")
    source = '#include <dart/empty.hpp>\n#include "dart/config.hpp"\n#if 0\n#include <dart/inactive.hpp>\n#endif\n'
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    entries = _entries(before, "headers")
    assert set(entries) == {"dart/empty.hpp", "dart/config.hpp"}
    assert all(row["source_present"] for row in entries.values())
    header.unlink()
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "changed", source)
    assert not _entries(after, "headers")["dart/empty.hpp"]["source_present"]
    assert before != after


def test_grouped_and_array_data_declarators(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """namespace dart {
struct Data {
  int x, y;
  int values[3], counts[2][4];
  int* pointer, scalar;
  std::array<int, 2> array, other;
  static int make();
  int first{make()}, second{2};
};
}
"""
    )
    api = extractor.index_api(headers)
    assert set(api.data_members) == {
        "x",
        "y",
        "values",
        "counts",
        "pointer",
        "scalar",
        "array",
        "other",
        "first",
        "second",
    }
    assert api.data_members["scalar"][0]["signature"] == "int scalar"
    assert api.data_members["counts"][0]["signature"] == "int counts [ 2 ] [ 4 ]"
    assert api.data_members["other"][0]["signature"] == "std :: array < int , 2 > other"
    result = _scan(
        extractor,
        api,
        tmp_path,
        "int init{dart::Data::make()}; void f(dart::Data& data) { data.x = 1; data.values[0] = 2; }",
    )
    assert set(_entries(result, "data_members")) == {"x", "values"}
    assert _entries(result, "members")["make"]["count"] == 1


def test_qualified_free_function_references(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        "void f() { auto callback = &dart::math::measure; auto direct = dart::math::measure; }",
    )
    assert _entries(result, "free_functions")["dart::math::measure"]["count"] == 2


def test_source_method_returns_stay_with_the_receiver_class(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        """namespace dart {
struct First { void setName(int); };
struct Second { void setName(double); };
}
"""
    )
    sources = {
        "helpers.hh": """struct A { dart::First* make(); };
struct B { dart::Second* make(); };
struct Derived : A {};
struct Override : B { dart::First* make(); };
namespace one { struct A { dart::First* make(); }; }
namespace two { struct A { dart::Second* make(); }; }
using Alias = one::A;
struct DartDerived : dart::Second { dart::First* make(); };
""",
        "helpers.cc": '#include "helpers.hh"\ndart::First* A::make() { return nullptr; }\ndart::Second* B::make() { return nullptr; }\n',
        "client.cc": '#include "helpers.hh"\nvoid f(A a, Derived derived, Override custom, Alias alias, DartDerived local) { a.make()->setName(1); derived.make()->setName(2); custom.make()->setName(3); alias.make()->setName(4); local.make()->setName(5); }\n',
    }
    before = _scan_files(
        extractor, extractor.index_api(headers), tmp_path / "before", sources
    )
    row = _entries(before, "members")["setName"]
    assert row["count"] == 5
    assert row["receiver_types"] == ["dart::First"]
    assert {item["class"] for item in row["declarations"]} == {"dart::First"}
    assert row["unresolved_count"] == 0
    header.write_text(header.read_text().replace("setName(double)", "setName(float)"))
    after = _scan_files(
        extractor, extractor.index_api(headers), tmp_path / "after", sources
    )
    assert _entries(after, "members")["setName"] == row
    assert after == before


def test_foreign_helper_return_does_not_borrow_a_dart_return(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Widget { int getValue(); }; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """#include <dart/api.hpp>
struct RealWrapper { dart::Widget* getWidget(); };
struct ForeignWidget { int getValue(); };
struct ForeignWrapper { ForeignWidget* getWidget(); };
void inspect(ForeignWrapper& foreign) {
  auto value = foreign.getWidget();
  value->getValue();
  foreign.getWidget()->getValue();
}
""",
    )
    assert not _entries(result, "members")


def test_source_members_hide_dart_base_overloads(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct A { void foo(int); }; }")
    result = _scan_files(
        extractor,
        extractor.index_api(headers),
        tmp_path / "sources",
        {
            "classes.hh": """struct Hidden : dart::A { void foo(); void run(); };
struct Inherited : Hidden { void run() { foo(); } };
struct Visible : dart::A { using dart::A::foo; void foo(); void run(); };
struct Base : dart::A {};
struct UsesBase : Base { using Base::foo; void foo(); void run() { foo(4); } };
""",
            "classes.cc": '#include "classes.hh"\nvoid Hidden::run() { foo(); Hidden::foo(); }\nvoid Visible::run() { foo(1); Visible::foo(2); }\nvoid call(Hidden& hidden, Visible& visible) { hidden.foo(); visible.foo(3); }\n',
        },
    )
    row = _entries(result, "members")["foo"]
    assert row["count"] == 4
    assert row["receiver_types"] == ["dart::A"]
    assert row["unresolved_count"] == 0


def test_aliases_of_source_classes_retain_dart_bases(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct A { void mutate(); static void staticCall(); }; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """namespace local {
struct Derived : dart::A {};
using Alias = Derived;
}
void use(local::Alias value) { value.mutate(); local::Alias::staticCall(); }
""",
    )
    members = _entries(result, "members")
    assert set(members) == {"mutate", "staticCall"}
    assert all(
        row["receiver_types"] == ["dart::A"] and row["unresolved_count"] == 0
        for row in members.values()
    )


@pytest.mark.parametrize(
    "suffix",
    [
        "mutable",
        "-> void",
        "noexcept(true) -> void",
        "mutable noexcept(true) -> decltype((void)dart::B{})",
    ],
)
def test_lambda_parameters_do_not_hide_outer_receivers(extractor, tmp_path, suffix):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct A { void hit(); }; struct B { void hit(); }; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::A* p) {\n"
        + f"  [](dart::B* p) {suffix} {{ p->hit(); }};\n"
        + """
  p->hit();
}
""",
    )
    row = _entries(result, "members")["hit"]
    assert row["count"] == 2
    assert row["receiver_types"] == ["dart::A", "dart::B"]
    assert {item["class"] for item in row["declarations"]} == {"dart::A", "dart::B"}
    assert row["unresolved_count"] == 0


def test_brace_constructions_retain_ordered_public_fields(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        """namespace dart {
struct Options { static int global; bool enabled; int count, other; int values[2]; };
struct Widget { Widget(int); int unrelated; };
struct Unused { int unrelated; };
}
"""
    )
    source = "void use() { dart::Options defaulted; auto temporary = dart::Options{true, 4, 5, {6, 7}}; dart::Options direct{true, 4}; dart::Options copied = {true, 4}; dart::Options array[2]{{true, 4}, {false, 5}}; dart::Widget widget{1}; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    constructors = _entries(before, "constructors")
    layout = constructors["dart::Options"]["brace_member_declarations"]
    assert [member["name"] for member in layout] == [
        "enabled",
        "count",
        "other",
        "values",
    ]
    assert constructors["dart::Options"]["count"] == 5
    assert constructors["dart::Options"]["declarations"] == []
    assert "brace_member_declarations" not in constructors["dart::Widget"]
    assert "dart::Unused" not in constructors
    header.write_text(
        header.read_text().replace(
            "bool enabled; int count, other;", "int count, other; bool enabled;"
        )
    )
    reordered = _scan(
        extractor, extractor.index_api(headers), tmp_path / "reordered", source
    )
    assert (
        _entries(reordered, "constructors")["dart::Options"][
            "brace_member_declarations"
        ]
        != layout
    )
    header.write_text(
        header.read_text().replace("int count, other;", "float count, other;")
    )
    retyped = _scan(
        extractor, extractor.index_api(headers), tmp_path / "retyped", source
    )
    assert retyped != reordered


def test_container_returns_retain_the_element_receiver(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """namespace dart {
struct Target { void other(); };
struct Item { void mutate(); int front(); Target* operator->(); };
struct Factory { std::vector<Item*> make(); std::vector<Item> valueMake(); };
std::vector<Item*> sequence();
}
"""
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """void use(dart::Factory* factory) {
  factory->make().front()->mutate();
  factory->make().back()->mutate();
  factory->make().at(0)->mutate();
  auto items = factory->make();
  items[0]->mutate();
  dart::sequence().front()->mutate();
  std::vector<dart::Item*> direct;
  direct.front()->mutate();
  factory->valueMake().front()->other();
}
""",
    )
    members = _entries(result, "members")
    assert members["mutate"]["count"] == 6
    assert members["mutate"]["receiver_types"] == ["dart::Item"]
    assert members["mutate"]["unresolved_count"] == 0
    assert "front" not in members
    assert members["operator->"]["count"] == 1
    assert members["other"]["receiver_types"] == ["dart::Target"]
    assert members["other"]["unresolved_count"] == 0


def test_class_attributes_preserve_the_class_and_members(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        """#define DART_DEPRECATED(version) [[deprecated]]
namespace dart {
class DART_DEPRECATED(1.0) Legacy { public: void run(); };
class alignas(16) [[vendor::hint(choose(1))]] Aligned { public: void run(); };
class __attribute__((deprecated)) Gnu { public: void run(); };
}
"""
    )
    api = extractor.index_api(headers)
    assert {"dart::Legacy", "dart::Aligned", "dart::Gnu"} <= api.types
    assert {method["class"] for method in api.members["run"]} == {
        "dart::Legacy",
        "dart::Aligned",
        "dart::Gnu",
    }
    real_api = extractor.index_api(ROOT / "dart")
    assert "dart::common::Timer" in real_api.types
    assert any(
        method["class"] == "dart::common::Timer" for method in real_api.members["start"]
    )


@pytest.mark.parametrize(
    ("definition", "change", "owner"),
    [
        ("struct alignas(16) A {};", ("16", "32"), "dart::A"),
        ("struct alignas(16) A; struct A {};", ("16", "32"), "dart::A"),
        ("struct alignas(8) alignas(16) A {};", ("16", "32"), "dart::A"),
        (
            "struct alignas(16) Align {}; struct alignas(Align) A {};",
            ("16", "32"),
            "dart::Align",
        ),
        ("constexpr int ALIGN = 16; struct alignas(ALIGN) A {};", ("16", "32"), "dart"),
        (
            "struct alignas(16) B {}; struct A { private: B member; };",
            ("16", "32"),
            "dart::B",
        ),
    ],
)
def test_alignment_metadata_retains_specifier_dependencies(
    extractor, tmp_path, definition, change, owner
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text("namespace dart { " + definition + " }")
    before = _scan(
        extractor, extractor.index_api(headers), tmp_path, "void f(dart::A* a) {}"
    )
    evidence = _entries(before, "types")["dart::A"]["alignment_declarations"]
    assert any(
        d["class"] == owner and "16" in d["signature"].replace(" ", "")
        for d in evidence
    )
    header.write_text(header.read_text().replace(*change))
    after = extractor.scan_sources(extractor.index_api(headers), tmp_path / "source")
    assert after != before
    assert any(
        d["class"] == owner and "32" in d["signature"].replace(" ", "")
        for d in _entries(after, "types")["dart::A"]["alignment_declarations"]
    )


def test_uppercase_declarations_are_not_assumed_to_be_macros(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        """#define MARKER()
namespace dart { struct API { API(int); void RUN(); MARKER(); }; }
"""
    )
    before = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path / "before",
        "void use(dart::API* api) { dart::API value(1); api->RUN(); }",
    )
    assert _entries(before, "members")["RUN"]["count"] == 1
    assert _entries(before, "constructors")["dart::API"]["declarations"]
    assert "MARKER" not in extractor.index_api(headers).members
    header.write_text(
        header.read_text()
        .replace("RUN()", "RUN(int)")
        .replace("API(int)", "API(int, int)")
    )
    after = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path / "after",
        "void use(dart::API* api) { dart::API value(1); api->RUN(); }",
    )
    assert after != before


def test_data_member_types_resolve_chained_receivers(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        """namespace dart {
struct Target { void other(); };
struct B { void hit(); Target* operator->(); };
struct Foreign { void hit(double); };
struct A { B child; B* pointer{nullptr}; std::vector<B*> children; };
struct Unused { Foreign child; };
}
"""
    )
    source = "void use(dart::A& a) { a.child.hit(); a.pointer->hit(); a.child->other(); a.children.front()->hit(); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    members = _entries(before, "members")
    assert members["hit"]["count"] == 3
    assert members["hit"]["receiver_types"] == ["dart::B"]
    assert members["hit"]["unresolved_count"] == 0
    assert members["other"]["receiver_types"] == ["dart::Target"]
    assert members["operator->"]["count"] == 1
    header.write_text(header.read_text().replace("void hit();", "void hit(int);"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after != before


def test_referenced_class_final_status_detects_inheritance_breaks(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { struct A { A(); }; struct D : A {}; using Alias = A; struct Unused final {}; }"
    )
    source = "using SourceAlias = dart::A; struct Derived : SourceAlias {}; void use(dart::D* derived, dart::Alias* alias);"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert all("final_classes" not in row for row in before["types"])
    header.write_text(header.read_text().replace("struct A {", "struct A final {"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert _entries(after, "constructors") == _entries(before, "constructors")
    assert after != before
    types = _entries(after, "types")
    assert "dart::Unused" not in types
    assert all(
        types[name]["final_classes"] == ["dart::A"]
        for name in ("dart::A", "dart::D", "dart::Alias")
    )


@pytest.mark.parametrize(
    ("imported_namespace", "qualifier"), [("dart::math", ""), ("dart", "math::")]
)
def test_free_function_namespace_imports_stay_in_their_scopes(
    extractor, api, tmp_path, imported_namespace, qualifier
):
    result = _scan_files(
        extractor,
        api,
        tmp_path,
        {
            "namespaces.cc": "namespace other { double measure(double); "
            "namespace math { double measure(double); } }\n"
            f"""namespace first {{
using namespace {imported_namespace};
void f() {{ {qualifier}measure(1.0); }}
}}
namespace second {{
using namespace other;
void f() {{ {qualifier}measure(2.0); }}
}}
""",
            "blocks.cc": "namespace other { double measure(double); "
            "namespace math { double measure(double); } }\n"
            f"""void f() {{
  {{
    using namespace {imported_namespace};
    {qualifier}measure(1.0);
  }}
  {{
    using namespace other;
    {qualifier}measure(2.0);
  }}
}}
""",
        },
    )
    functions = _entries(result, "free_functions")
    assert set(functions) == {"dart::math::measure"}
    assert functions["dart::math::measure"]["count"] == 2
    assert functions["dart::math::measure"]["examples"] == [
        "dartsim/src/blocks.cc:5",
        "dartsim/src/namespaces.cc:4",
    ]


def test_subscripted_container_elements(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """void use(std::vector<dart::dynamics::Widget*> widgets) {
  widgets[0]->getValue();
  widgets.at(1)->isReady();
}
""",
    )
    members = _entries(result, "members")
    assert members["getValue"]["receiver_types"] == ["dart::dynamics::Widget"]
    assert members["isReady"]["receiver_types"] == ["dart::dynamics::Widget"]


def test_extractor_imports_are_used():
    tree = ast.parse(SCRIPT.read_text())
    imported = {
        (alias.asname or alias.name).split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert imported <= {
        node.id for node in ast.walk(tree) if isinstance(node, ast.Name)
    }


def test_check_without_checkout_reports_skip(extractor, capsys):
    assert extractor.main(["--check"]) == 0
    assert "skip" in capsys.readouterr().out.lower()


def test_check_detects_source_inventory_and_revision_drift(
    extractor, api, tmp_path, monkeypatch, capsys
):
    physics = tmp_path / "physics"
    simulation = tmp_path / "simulation"
    physics.mkdir()
    simulation.mkdir()
    snippet = physics / "use.cc"
    snippet.write_text("void use(dart::dynamics::Widget* w) { w->getValue(); }\n")
    output = tmp_path / "inventory.json"
    sha = "a" * 40
    monkeypatch.setattr(extractor, "index_api", lambda _: api)
    monkeypatch.setattr(extractor, "revision", lambda _: sha)
    arguments = [
        str(physics),
        "--gz-sim",
        str(simulation),
        "--output",
        str(output),
    ]
    assert extractor.main(arguments) == 0
    original = output.read_text()
    assert extractor.main(["--check", *arguments]) == 0
    assert extractor.main(["--check", str(physics), "--output", str(output)]) == 0
    assert "gz-sim" in capsys.readouterr().out

    snippet.write_text("void use(dart::dynamics::Widget* w) { w->setValue(1); }\n")
    assert extractor.main(["--check", *arguments]) != 0
    assert output.read_text() == original
    snippet.write_text("void use(dart::dynamics::Widget* w) { w->getValue(); }\n")

    damaged = json.loads(original)
    damaged["sources"]["gz-physics"]["members"][0]["count"] += 1
    output.write_text(json.dumps(damaged))
    assert extractor.main(["--check", *arguments]) != 0
    output.write_text(original)

    output.write_text(json.dumps(json.loads(original)))
    assert extractor.main(["--check", *arguments]) != 0
    output.write_text(original)

    sha = "b" * 40
    assert extractor.main(["--check", *arguments]) != 0
    assert output.read_text() == original


def test_checked_in_inventory_is_well_formed_and_sorted():
    text = INVENTORY.read_text()
    inventory = json.loads(text)
    assert text == json.dumps(inventory, indent=2, sort_keys=True) + "\n"
    assert inventory["schema_version"] == 1
    assert list(inventory["sources"]) == ["gz-physics", "gz-sim"]
    physics = inventory["sources"]["gz-physics"]
    assert {
        "getSkeleton",
        "getNumBodyNodes",
        "moveTo",
        "getBodyNode",
        "createShapeNodeWith",
        "getPositions",
        "setPositions",
        "resetPositions",
        "resetVelocities",
    } <= set(_entries(physics, "members"))
    assert {
        "dart::collision::OdeCollisionGroup",
        "dart::dynamics::BoxShape",
        "dart::simulation::World",
    } <= set(_entries(physics, "constructors"))
    assert {"FORCE", "collisionFilter", "mRayHits", "penetrationDepth"} <= set(
        _entries(physics, "data_members")
    )
    overrides = _entries(physics, "overrides")
    for suffix in (
        "GzOdeCollisionDetector::collide",
        "GzOdeCollisionDetector::raycast",
        "GzOdeCollisionDetector::createCollisionGroup",
        "GzBulletCollisionDetector::collide",
        "GzBulletCollisionDetector::createCollisionGroup",
        "GzContactSurfaceHandler::createParams",
        "GzContactSurfaceHandler::createConstraint",
    ):
        assert any(name.endswith(suffix) for name in overrides)
    for source in inventory["sources"].values():
        assert re.fullmatch(r"[0-9a-f]{40}", source["sha"])
        for section in SECTIONS:
            entries = source[section]
            names = [entry["name"] for entry in entries]
            assert names == sorted(set(names))
            for entry in entries:
                if section in {"types", "constructors"}:
                    assert entry["name"].startswith("dart::")
                assert isinstance(entry["count"], int) and entry["count"] > 0
                assert entry["examples"] == sorted(set(entry["examples"]))
                assert 1 <= len(entry["examples"]) <= 3
                assert all(
                    re.fullmatch(r"[^:]+:[1-9][0-9]*", example)
                    and not Path(example.rsplit(":", 1)[0]).is_absolute()
                    for example in entry["examples"]
                )
                if section == "members":
                    assert entry["classification"] in {"accessor", "mutation"}
                if section in {"members", "data_members", "variables"}:
                    assert entry["declarations"]
                if section == "macros":
                    assert isinstance(entry["declared_in_dart"], bool)
                for declaration in entry.get("declarations", []):
                    assert isinstance(declaration["const"], bool)
                    assert isinstance(declaration["virtual"], bool)
                    assert declaration["signature"]


@pytest.mark.parametrize(
    ("operator", "expression"),
    [
        ("operator->", "value->getValue()"),
        ("operator[]", "value[0]->getValue()"),
        ("operator()", "value(0)->getValue()"),
    ],
)
def test_operator_syntax_on_known_dart_receivers(
    extractor, api, tmp_path, operator, expression
):
    header = tmp_path / "dart" / "operators.hpp"
    header.write_text(
        """namespace dart::dynamics {
struct Operators {
  Operators(int value);
  Widget* operator->() const;
  Widget* operator[](int index) const;
  Widget* operator()(int index) const;
  Widget* get() const;
};
}
"""
    )
    api = extractor.index_api(header.parent)
    result = _scan(
        extractor,
        api,
        tmp_path,
        f"""void use(
    dart::dynamics::Operators value, dart::dynamics::Operators* raw) {{
  dart::dynamics::Operators constructed(1);
  {expression};
  raw->get();
}}
""",
    )
    members = _entries(result, "members")
    entry = members[operator]
    assert entry["count"] == 1
    assert entry["unresolved_count"] == 0
    assert entry["receiver_types"] == ["dart::dynamics::Operators"]
    assert entry["declarations"][0]["const"]
    assert members["getValue"]["receiver_types"] == ["dart::dynamics::Widget"]
    assert set(members) == {operator, "getValue", "get"}


def test_template_pointer_arrow_operator(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """void use(
    dart::dynamics::WidgetPtr custom,
    dart::dynamics::Pointer<dart::dynamics::Widget>* raw,
    dart::dynamics::WidgetSharedPtr standard) {
  custom->getValue();
  raw->get();
  standard->getValue();
}
""",
    )
    entry = _entries(result, "members")["operator->"]
    assert entry["count"] == 1
    assert entry["receiver_types"] == ["dart::dynamics::WidgetPtr"]
    assert {d["class"] for d in entry["declarations"]} == {"dart::dynamics::Pointer"}


@pytest.mark.parametrize(
    ("parameter", "expression"),
    [
        ("std::shared_ptr<dart::Handle> h", "h->touch()"),
        ("dart::HandleShared h", "h->touch()"),
        ("void* p", "static_cast<dart::Handle*>(p)->touch()"),
        ("dart::HandleShared h", "h.get()->touch()"),
    ],
)
def test_builtin_pointer_arrow_does_not_invoke_pointee_operator(
    extractor, api, tmp_path, parameter, expression
):
    header = tmp_path / "dart" / "handle.hpp"
    header.write_text(
        """namespace dart {
struct Handle {
  dynamics::Widget* operator->() const;
  void touch();
};
using HandleShared = std::shared_ptr<Handle>;
}
"""
    )
    result = _scan(
        extractor,
        extractor.index_api(header.parent),
        tmp_path,
        f"void use({parameter}) {{ {expression}; }}\n",
    )
    members = _entries(result, "members")
    assert "operator->" not in members
    assert members["touch"]["count"] == 1
    assert {d["class"] for d in members["touch"]["declarations"]} == {"dart::Handle"}


def test_scoped_enum_constants_are_indexed_and_recorded(extractor, tmp_path):
    headers = tmp_path / "headers"
    headers.mkdir()
    (headers / "enums.hpp").write_text(
        """namespace dart {
enum class Mode : int { Fast = (1, 2), Slow = 3 };
enum struct State { Ready, Waiting };
struct Holder { enum class Mode { Fast, Slow }; };
}
"""
    )
    api = extractor.index_api(headers)
    assert set(api.data_members) == {"Fast", "Slow", "Ready", "Waiting"}
    result = _scan(
        extractor,
        api,
        tmp_path,
        """void use() {
  auto mode = dart::Mode::Fast;
  auto other = dart::Holder::Mode::Fast;
  auto state = dart::State::Ready;
}
""",
    )
    constants = _entries(result, "data_members")
    assert set(constants) == {"Fast", "Ready"}
    assert constants["Fast"]["count"] == 2
    assert constants["Fast"]["receiver_types"] == [
        "dart::Holder::Mode",
        "dart::Mode",
    ]
    assert {d["class"] for d in constants["Fast"]["declarations"]} == {
        "dart::Holder::Mode",
        "dart::Mode",
    }
    assert all(
        declaration["const"]
        for entry in constants.values()
        for declaration in entry["declarations"]
    )
    assert constants["Ready"]["receiver_types"] == ["dart::State"]


def test_unscoped_enum_names_retain_enumerator_and_type_drift(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { namespace math { enum Axis : unsigned char { X = 1, Y }; "
        "enum class Scoped { Hidden }; } struct Holder { enum Flag { Ready = 2 }; }; }"
    )
    source = "namespace dm = dart::math; void use() { auto x = dm::X; auto y = dm::Axis::Y; auto flag = dart::Holder::Ready; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    variables = _entries(before, "variables")
    assert set(variables) == {"dart::math::X"}
    assert (
        variables["dart::math::X"]["declarations"][0]["enum_type"] == "dart::math::Axis"
    )
    members = _entries(before, "data_members")
    assert set(members) == {"Ready", "Y"}
    assert members["Ready"]["receiver_types"] == ["dart::Holder"]
    assert "dart::math::Hidden" not in extractor.index_api(headers).variables
    header.write_text(header.read_text().replace("X = 1", "X = 4"))
    changed = _scan(
        extractor, extractor.index_api(headers), tmp_path / "changed", source
    )
    assert _entries(changed, "variables") != variables
    header.write_text(header.read_text().replace("unsigned char", "unsigned int"))
    underlying = _scan(
        extractor, extractor.index_api(headers), tmp_path / "underlying", source
    )
    assert _entries(underlying, "variables") != _entries(changed, "variables")
    header.write_text(header.read_text().replace("X = 4, ", ""))
    removed = _scan(
        extractor, extractor.index_api(headers), tmp_path / "removed", source
    )
    assert "dart::math::X" not in _entries(removed, "variables")


def test_enum_type_headers_detect_underlying_and_scopedness_drift(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { enum class Mode : unsigned char { Fast }; "
        "enum class Forward : short; using Alias = Mode; enum Unused { Other }; }"
    )
    source = "void use(dart::Mode mode, dart::Alias alias, dart::Forward forward);"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    types = _entries(before, "types")
    assert set(types) == {"dart::Mode", "dart::Alias", "dart::Forward"}
    assert types["dart::Mode"]["enum_declarations"] == [
        "enum class Mode : unsigned char"
    ]
    assert (
        types["dart::Alias"]["enum_declarations"]
        == types["dart::Mode"]["enum_declarations"]
    )
    assert types["dart::Forward"]["enum_declarations"] == ["enum class Forward : short"]
    for old, new in (
        ("unsigned char", "unsigned int"),
        ("enum class Mode", "enum Mode"),
        ("Forward : short", "Forward : int"),
    ):
        header.write_text(header.read_text().replace(old, new))
        after = _scan(
            extractor,
            extractor.index_api(headers),
            tmp_path / new.replace(" ", "-"),
            source,
        )
        assert after["types"] != before["types"]
        before = after


def test_array_constructions_retain_element_constructor_drift(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text("namespace dart { struct A { A(); }; }")
    source = """dart::A global[3];
struct Holder { dart::A member[2][3]; static dart::A staticOnly[2]; };
extern dart::A external[3];
void parameter(dart::A values[3]);
void use() { dart::A local[2]; dart::A initialized[2]{}; dart::A* pointers[2]; }
"""
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    constructor = _entries(before, "constructors")["dart::A"]
    assert constructor["count"] == 4
    assert [d["signature"] for d in constructor["declarations"]] == ["A ( )"]
    header.write_text(header.read_text().replace("A();", "explicit A(int);"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (
        _entries(after, "constructors")["dart::A"]["declarations"]
        != constructor["declarations"]
    )


def test_array_receivers_preserve_dimensions_and_pointer_elements(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Target { void other(); }; struct A { A(); void hit(); Target* operator->(); }; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use() { dart::A values[2][3]; values[0][1].hit(); dart::A* pointers[2]; pointers[0]->hit(); values[0][1]->other(); }",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 2
    assert members["hit"]["receiver_types"] == ["dart::A"]
    assert members["hit"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 1
    assert members["other"]["receiver_types"] == ["dart::Target"]


def test_constructor_access_distinguishes_implicit_and_unavailable(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { struct A {}; struct Derived : A {}; struct Unused {}; }"
    )
    source = "void use() { dart::A direct; dart::Derived derived; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert all(row["declarations"] == [] for row in before["constructors"])
    for definition, access in (
        ("private: A();", "private"),
        ("protected: A();", "protected"),
        ("A() = delete;", "public"),
    ):
        header.write_text(
            "namespace dart { struct A { "
            + definition
            + " }; struct Derived : A {}; struct Unused {}; }"
        )
        after = _scan(
            extractor, extractor.index_api(headers), tmp_path / access, source
        )
        assert after != before
        constructors = _entries(after, "constructors")
        assert set(constructors) == {"dart::A", "dart::Derived"}
        for row in constructors.values():
            assert row["declarations"][0]["access"] == access
            assert row["declarations"][0]["class"] == "dart::A"
            if access == "public":
                assert "delete" in row["declarations"][0]["signature"]


@pytest.mark.parametrize(
    "body",
    [
        "item->mutate();",
        "{ item->mutate(); }",
        "if (true) item->mutate(); else item->mutate();",
    ],
)
def test_range_for_returns_bind_elements_only_inside_loop(extractor, tmp_path, body):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { struct Target { void other(); }; struct Item { void mutate(); Target* operator->(); }; struct Factory { std::vector<Item*> items(); }; }"
    )
    source = (
        "struct Foreign { void mutate(); }; void use(dart::Factory* factory, Foreign* item) { for (auto* item : factory->items()) "
        + body
        + " item->mutate(); }"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    members = _entries(before, "members")
    assert members["mutate"]["count"] == (2 if body.startswith("if") else 1)
    assert members["mutate"]["receiver_types"] == ["dart::Item"]
    assert members["mutate"]["unresolved_count"] == 0
    assert "operator->" not in members
    header.write_text(header.read_text().replace("void mutate();", "void mutate(int);"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after["members"] != before["members"]


def test_range_for_value_and_pointer_alias_elements_dispatch_correctly(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Target { void other(); }; struct Item { void mutate(); Target* operator->(); }; using Pointer = Item*; struct Factory { std::vector<Pointer> pointers(); std::vector<Item> values(); }; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::Factory* factory) { for (auto item : factory->pointers()) item->mutate(); for (auto& item : factory->values()) item->other(); }",
    )
    members = _entries(result, "members")
    assert members["mutate"]["count"] == 1
    assert members["mutate"]["receiver_types"] == ["dart::Pointer"]
    assert members["mutate"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 1
    assert members["other"]["receiver_types"] == ["dart::Target"]


def test_foreign_range_elements_do_not_gain_dart_members(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct Known { void get(); }; }")
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "struct Foreign { void get(); }; struct ForeignFactory { std::vector<Foreign*> items(); }; "
        "void use(dart::Known* known, ForeignFactory factory, std::vector<Foreign*> values) { "
        "for (auto* item : factory.items()) item->get(); for (auto* item : values) item->get(); "
        "} template<class Mystery> void unresolved(dart::Known* known, Mystery& unknown) { "
        "for (auto* item : unknown) item->get(); }",
    )
    members = _entries(result, "members")
    assert members["get"]["count"] == 1
    assert members["get"]["unresolved_count"] == 1
    assert members["get"]["receiver_types"] == []


def test_grouped_receivers_preserve_return_and_pointer_types(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { struct Target { void other(); }; struct A { void hit(); Target* operator->(); }; struct Callable { A operator()(); }; struct Factory { A make(); A* pointer(); std::vector<A*> items(); Callable callable(); }; }"
    )
    source = "void use(dart::Factory* factory) { (factory->make()).hit(); ((factory->pointer()))->hit(); (factory->items()).front()->hit(); (factory->callable())().hit(); (factory->make())->other(); return (factory->make()).hit(); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    members = _entries(before, "members")
    assert members["hit"]["count"] == 5
    assert members["hit"]["receiver_types"] == ["dart::A"]
    assert members["hit"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 1
    assert members["operator()"]["receiver_types"] == ["dart::Callable"]
    assert members["other"]["receiver_types"] == ["dart::Target"]
    header.write_text(header.read_text().replace("void hit();", "void hit(int);"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after["members"] != before["members"]


@pytest.mark.parametrize(
    "statement,inner_count",
    [
        ("if (dart::A* item = nullptr) item->hit();", 1),
        ("if (dart::A* item = nullptr) item->hit(); else item->hit();", 2),
        ("if (dart::A* item = nullptr) { item->hit(); } else { item->hit(); }", 2),
        ("while (dart::A* item = nullptr) item->hit();", 1),
        ("for (dart::A* item = nullptr; false;) item->hit();", 1),
        (
            "for (; dart::A* item = nullptr;) if (true) item->hit(); else item->hit();",
            2,
        ),
        ("switch (dart::A* item = nullptr; 0) { default: item->hit(); }", 1),
    ],
)
def test_control_declarations_end_with_complete_statement(
    extractor, tmp_path, statement, inner_count
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct A { void inner(); }; struct B { void outer(); }; }"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::B* item) { "
        + statement.replace("hit", "inner")
        + " item->outer(); }",
    )
    members = _entries(result, "members")
    assert members["inner"]["count"] == inner_count
    assert members["inner"]["receiver_types"] == ["dart::A"]
    assert members["outer"]["count"] == 1
    assert members["outer"]["receiver_types"] == ["dart::B"]
    assert all(member["unresolved_count"] == 0 for member in members.values())


def test_grouped_foreign_receiver_does_not_gain_dart_member(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct Known { void hit(); }; }")
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "struct Foreign { void hit(); }; struct Factory { Foreign make(); }; void use(dart::Known* known, Factory factory) { ((factory.make())).hit(); (known)->hit(); }",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 1
    assert hit["receiver_types"] == ["dart::Known"]
    assert hit["unresolved_count"] == 0


@pytest.mark.parametrize(
    "statement",
    [
        "if (Foreign* item = nullptr) { item->hit(); } else { item->hit(); }",
        "while (Foreign* item = nullptr) item->hit();",
    ],
)
def test_foreign_control_declaration_shadows_and_restores_dart_receiver(
    extractor, tmp_path, statement
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct B { void hit(); }; }")
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "struct Foreign { void hit(); }; void use(dart::B* item) { "
        + statement
        + " item->hit(); }",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 1
    assert hit["receiver_types"] == ["dart::B"]
    assert hit["unresolved_count"] == 0


def test_namespace_directives_expose_variables_without_local_leakage(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { constexpr int LIMIT = 3; namespace math { constexpr int AXIS = 1; } }"
    )
    source = "namespace first { using namespace dart; void use() { auto x = LIMIT; { int LIMIT = 4; auto local = LIMIT; } } } namespace second { void use() { int LIMIT = 5; auto local = LIMIT; } } namespace third { using namespace dart::math; void use() { auto axis = AXIS; } }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    variables = _entries(before, "variables")
    assert set(variables) == {"dart::LIMIT", "dart::math::AXIS"}
    assert all(row["count"] == 1 for row in variables.values())
    header.write_text(header.read_text().replace("LIMIT = 3", "LIMIT = 6"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after["variables"] != before["variables"]


@pytest.mark.parametrize(
    "return_type",
    [
        "decltype(makeValue())",
        "decltype((makeValue()))",
        "decltype(makeValue(choose(1)))",
    ],
)
def test_decltype_operands_are_not_method_declarators(extractor, tmp_path, return_type):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { int choose(int); int makeValue(int value = 0); struct A { "
        + return_type
        + " getValue() const; int ordinary(); }; }"
    )
    api = extractor.index_api(headers)
    assert set(api.members) == {"getValue", "ordinary"}
    source = "void use(dart::A* value) { value->getValue(); }"
    before = _scan(extractor, api, tmp_path / "before", source)
    value = _entries(before, "members")["getValue"]
    assert value["unresolved_count"] == 0
    assert (
        value["declarations"][0]["signature"]
        == " ".join(extractor.TOKEN.findall(return_type)) + " getValue ( ) const"
    )
    header.write_text(header.read_text().replace("getValue()", "getValue(int)"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after["members"] != before["members"]


def test_source_return_aliases_preserve_declaration_scope_and_shape(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct Target { void other(); }; struct A { void hit(); Target* operator->(); }; }"
    )
    source = """namespace app {
using APtr = dart::A*;
using Shared = std::shared_ptr<dart::A>;
using AVec = std::vector<APtr>;
struct Factory {
  using Local = APtr;
  Local make(); AVec items(); std::vector<APtr> also(); Shared shared();
  void hit();
};
Factory::Local Factory::make() { return nullptr; }
struct Derived : dart::A {};
struct Adapters { Derived* derived(); };
APtr free();
void use(Factory factory, Adapters adapter) {
  factory.make()->hit(); factory.items().front()->hit();
  factory.also().at(0)->hit(); factory.shared()->hit();
  adapter.derived()->hit(); auto value = free(); value->hit();
}
}
"""
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    members = _entries(result, "members")
    assert set(members) == {"hit"}
    assert members["hit"]["count"] == 6
    assert members["hit"]["receiver_types"] == ["dart::A"]
    assert members["hit"]["unresolved_count"] == 0


def test_source_return_aliases_use_included_header_context(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct A { void hit(); }; }")
    source = tmp_path / "source"
    source.mkdir()
    (source / "aliases.hh").write_text(
        "namespace app { using A = dart::A; using APtr = A*; using AVec = std::vector<APtr>; struct Factory { APtr make(); AVec items(); }; }"
    )
    (source / "use.cc").write_text(
        '#include "aliases.hh"\nvoid use(app::Factory factory) { factory.make()->hit(); factory.items().front()->hit(); }'
    )
    result = extractor.scan_sources(extractor.index_api(headers), source)
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 2
    assert hit["receiver_types"] == ["dart::A"]
    assert hit["unresolved_count"] == 0


@pytest.mark.parametrize("call", ["foreign()", "util::foreign()", "renamed::foreign()"])
def test_known_foreign_free_function_returns_suppress_dart_candidates(
    extractor, tmp_path, call
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct A { void hit(); }; }")
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "struct Foreign { void hit(); }; Foreign foreign(); namespace util { Foreign foreign(); } namespace renamed = util; void use(dart::A* known) { auto value = "
        + call
        + "; value.hit(); }",
    )
    assert result["members"] == []


def test_source_free_function_imports_and_callable_shadowing(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct A { void hit(); }; struct Callable { A* operator()(); }; }"
    )
    source = """struct Foreign { void hit(); };
namespace util { Foreign foreign(); }
using namespace util;
void use(dart::A* known) { auto value = foreign(); value.hit(); }
void shadow(dart::Callable foreign) { foreign()->hit(); }
namespace other {
using util::foreign;
void use(dart::A* known) { foreign().hit(); }
}
"""
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 1
    assert hit["receiver_types"] == ["dart::A"]
    assert hit["unresolved_count"] == 0


def test_unknown_callable_parameter_hides_known_foreign_function(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct A { void hit(); }; }")
    source = """struct Foreign { void hit(); }; Foreign factory();
template<class Callable> void use(dart::A* known, Callable factory) {
  auto value = factory(); value.hit();
}
template<class Callable> struct Wrapper {
  dart::A* known;
  Wrapper(dart::A* input, Callable factory) : known(input) {
    auto value = factory(); value.hit();
  }
  void after() { factory().hit(); }
};
void lambda(dart::A* known) {
  auto call = [](auto factory) mutable { auto value = factory(); value.hit(); };
  factory().hit();
}
void after(dart::A* known) { factory().hit(); }
"""
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == hit["unresolved_count"] == 3
    assert hit["receiver_types"] == []


@pytest.mark.parametrize("wrapper", ["shared_ptr", "unique_ptr", "weak_ptr"])
def test_known_foreign_standard_pointer_returns_suppress_dart_candidates(
    extractor, tmp_path, wrapper
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct A { void hit(); }; }")
    expression = "foreign().lock()" if wrapper == "weak_ptr" else "foreign()"
    source = (
        "struct Foreign { void hit(); }; std::"
        + wrapper
        + "<Foreign> foreign(); void use(dart::A* known) { "
        + expression
        + "->hit(); }"
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert result["members"] == []


def test_out_of_line_template_owner_is_not_a_source_return_type(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct A { void hit(); }; }")
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "struct Foreign { void hit(); }; template<class T> struct Factory : dart::A { Foreign make(); }; template<class T> Foreign Factory<T>::make() { return {}; } void use(Factory<int> factory) { factory.make().hit(); }",
    )
    assert result["members"] == []


@pytest.mark.parametrize(
    "return_type",
    ["T", "auto", "decltype(T{})", "std::vector<T>", "std::shared_ptr<T>"],
)
def test_unknown_source_free_function_returns_remain_conservative(
    extractor, tmp_path, return_type
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { struct A { void hit(); }; }")
    definition = "struct Foreign { void hit(); }; "
    definition += "template<class T> " + return_type + " foreign()"
    definition += " { return T{}; }" if return_type == "auto" else ";"
    expression = (
        "foreign<Foreign>().front()"
        if "vector" in return_type
        else "foreign<Foreign>()"
    )
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        definition
        + " void use(dart::A* known) { auto value = "
        + expression
        + ("; value->hit(); }" if "ptr" in return_type else "; value.hit(); }"),
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == hit["unresolved_count"] == 1
    assert hit["receiver_types"] == []


def test_template_qualified_static_calls_and_return_receivers(extractor, tmp_path):
    headers = tmp_path / "headers"
    headers.mkdir()
    (headers / "templates.hpp").write_text(
        """namespace dart {
struct Result { int value() const; };
template<class T> struct Box {
  static Result* create();
  static void mutate();
  static constexpr int limit = 1;
};
}
"""
    )
    api = extractor.index_api(headers)
    result = _scan(
        extractor,
        api,
        tmp_path,
        """void use() {
  dart::Box<int>::mutate();
  dart::Box<std::vector<int>>::mutate();
  auto result = dart::Box<std::pair<int, double>>::create();
  result->value();
  dart::Box<int>::create()->value();
  auto limit = dart::Box<std::vector<int>>::limit;
}
""",
    )
    members = _entries(result, "members")
    assert members["mutate"]["count"] == 2
    assert members["mutate"]["receiver_types"] == ["dart::Box"]
    assert members["create"]["count"] == 2
    assert members["create"]["receiver_types"] == ["dart::Box"]
    assert members["value"]["count"] == 2
    assert members["value"]["receiver_types"] == ["dart::Result"]
    assert _entries(result, "data_members")["limit"]["receiver_types"] == ["dart::Box"]


def test_local_includes_propagate_dart_relevance_transitively(extractor, api, tmp_path):
    source = tmp_path / "source"
    snippets = {
        "include/Owner.hh": """struct Owner {
  dart::dynamics::Widget* engine;
  void tick();
};
""",
        "include/Bridge.hh": '#include "Owner.hh"\n',
        "src/use.cc": """#include "Bridge.hh"
void Owner::tick() { engine->getValue(); }
""",
        "src/unrelated.cc": "void unrelated() { engine->getValue(); }\n",
    }
    for name, snippet in snippets.items():
        path = source / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(snippet)
    result = extractor.scan_sources(api, source)
    member = _entries(result, "members")["getValue"]
    assert member["count"] == 1
    assert member["receiver_types"] == ["dart::dynamics::Widget"]
    assert member["unresolved_count"] == 0
    assert member["examples"] == ["src/use.cc:2"]


@pytest.mark.parametrize(
    "snippet",
    [
        """namespace other { struct Widget { void getValue(); }; }
namespace first {
using namespace dart::dynamics;
void f(Widget* w) { w->getValue(); }
}
namespace second {
using namespace other;
void f(Widget* w) { w->getValue(); }
}
""",
        """namespace other { struct Widget { void getValue(); }; }
namespace second {
namespace dd = other;
void f(dd::Widget* w) { w->getValue(); }
}
namespace first {
namespace dd = dart::dynamics;
void f(dd::Widget* w) { w->getValue(); }
}
""",
        """namespace other { struct Widget { void getValue(); }; }
void f() {
 { using namespace dart::dynamics; Widget* w; w->getValue(); }
 { using namespace other; Widget* w; w->getValue(); }
}
""",
        """namespace other { struct Widget { void getValue(); }; }
void f() {
 { namespace dd = other; dd::Widget* w; w->getValue(); }
 { namespace dd = dart::dynamics; dd::Widget* w; w->getValue(); }
}
""",
    ],
)
def test_known_filewide_namespace_import_and_alias_limitation(
    extractor, api, tmp_path, snippet
):
    result = _scan(extractor, api, tmp_path, snippet)
    member = _entries(result, "members")["getValue"]
    # Namespace directives and aliases currently retain an extra non-DART use.
    assert member["count"] == 2
    assert member["receiver_types"] == ["dart::dynamics::Widget"]
    assert member["unresolved_count"] == 0


def test_known_same_named_source_field_limitation(extractor, api, tmp_path):
    result = _scan(
        extractor,
        api,
        tmp_path,
        """struct Other { void getValue(); };
struct DartHolder { dart::dynamics::Widget* item; };
struct OtherHolder { Other* item; void f(); };
void OtherHolder::f() { item->getValue(); }
void outside(OtherHolder* other) { other->item->getValue(); }
""",
    )
    member = _entries(result, "members")["getValue"]
    # Source fields share names across owners, conservatively retaining both uses.
    assert member["count"] == 2
    assert member["receiver_types"] == ["dart::dynamics::Widget"]
    assert member["unresolved_count"] == 0


@pytest.mark.parametrize(
    ("snippet", "expected"),
    [
        ("void f(dart::ArrowRaw h) { h->touch(); }", {"touch"}),
        ("void f(dart::ArrowRawNext h) { h->touch(); }", {"touch"}),
        (
            "using Raw = dart::Arrow<int>*; void f(Raw h) { h->touch(); }",
            {"touch"},
        ),
        (
            "using Raw = dart::ArrowRaw; using Next = Raw; "
            "void f(Next h) { h->touch(); }",
            {"touch"},
        ),
        ("void f(dart::ArrowSharedNext h) { h->touch(); }", {"touch"}),
        (
            "using Shared = std::shared_ptr<dart::Arrow<int>>; "
            "void f(Shared h) { h->touch(); }",
            {"touch"},
        ),
        (
            "using Shared = dart::ArrowShared; using Next = Shared; "
            "void f(Next h) { h->touch(); }",
            {"touch"},
        ),
        (
            "void f(dart::ArrowRef h) { h->getValue(); }",
            {"operator->", "getValue"},
        ),
        (
            "using Ref = dart::Arrow<int*>&; void f(Ref h) { h->getValue(); }",
            {"operator->", "getValue"},
        ),
    ],
)
def test_operator_arrow_preserves_pointer_and_reference_alias_identity(
    extractor, api, tmp_path, snippet, expected
):
    (tmp_path / "dart" / "alias_operators.hpp").write_text(
        """namespace dart {
template<class T> struct Arrow {
  dynamics::Widget* operator->() const;
  void touch();
};
using ArrowRaw = Arrow<int>*;
using ArrowRawNext = ArrowRaw;
using ArrowRef = Arrow<int>&;
using ArrowShared = std::shared_ptr<Arrow<int>>;
using ArrowSharedNext = ArrowShared;
}
"""
    )
    result = _scan(extractor, extractor.index_api(tmp_path / "dart"), tmp_path, snippet)
    members = _entries(result, "members")
    assert set(members) == expected
    assert all(entry["count"] == 1 for entry in members.values())


@pytest.mark.parametrize(
    "holder",
    [
        "struct A { B value; };",
        "class A { B value; };",
        "class A { protected: B value; };",
        "struct A { B values[2][3]; };",
        "struct A { using Value = B; Value value; };",
        "struct A { std::array<B, 2> values; };",
        "class A { B value; public: A() = default; };",
        "class A { B value; public: A() {} };",
        "struct C { B value; }; struct A { C value; };",
        "struct A : B { A() = default; };",
    ],
)
def test_construction_retains_owned_member_dependencies(extractor, tmp_path, holder):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "#include <array>\nnamespace dart { struct B { B(); }; " + holder + " }"
    header.write_text(text)
    source = "void use() { dart::A value; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    declarations = _entries(before, "constructors")["dart::A"]["declarations"]
    assert any(
        d["class"] == "dart::B" and d["signature"] == "B ( )" for d in declarations
    )
    header.write_text(text.replace("B();", "B(int);"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert _entries(after, "constructors")["dart::A"]["declarations"] != declarations


@pytest.mark.parametrize(
    "holder, source",
    [
        ("struct A { B* value = nullptr; };", "void use() { dart::A value; }"),
        ("struct A { B* values[2]{}; };", "void use() { dart::A value; }"),
        ("struct A { static B value; };", "void use() { dart::A value; }"),
        (
            "struct A { using Pointer = B*; Pointer value = nullptr; };",
            "void use() { dart::A value; }",
        ),
        ("struct A { std::array<B, 0> values; };", "void use() { dart::A value; }"),
        ("struct A { std::vector<B> values; };", "void use() { dart::A value; }"),
        (
            "struct A { B& value; A(B& input) : value(input) {} };",
            "void use(dart::B& input) { dart::A value(input); }",
        ),
    ],
)
def test_construction_excludes_unconstructed_member_dependencies(
    extractor, tmp_path, holder, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "#include <array>\n#include <vector>\nnamespace dart { struct B { B(); }; "
        + holder
        + " }"
    )
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert all(
        d["class"] != "dart::B"
        for d in _entries(before, "constructors")["dart::A"]["declarations"]
    )
    header.write_text(text.replace("B();", "B(int);"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after == before


@pytest.mark.parametrize(
    "definition",
    [
        "struct A { virtual void required() = 0; };",
        "class A { virtual void required() = 0; };",
        "struct Root { virtual void required(); }; struct A : Root { void required() = 0; };",
        "struct Root { virtual void required() = 0; }; struct A : Root {};",
    ],
)
@pytest.mark.parametrize(
    "source",
    [
        "struct Derived : dart::A {}; void use() { Derived value; }",
        "struct Middle : dart::A {}; struct Derived : Middle {}; void use() { Derived value; }",
        "void use() { dart::A value; }",
        "namespace dart { struct Holder { A member; }; } void use() { dart::Holder value; }",
    ],
)
def test_construction_retains_pure_virtual_dependencies(
    extractor, tmp_path, definition, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    holder = "struct Holder { A member; };" if "dart::Holder" in source else ""
    if holder:
        source = "void use() { dart::Holder value; }"
    header.write_text(
        "namespace dart { " + definition.replace(" = 0", "") + holder + " }"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert all("pure_virtual_declarations" not in row for row in before["constructors"])
    header.write_text("namespace dart { " + definition + holder + " }")
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    requirements = [
        d
        for row in after["constructors"]
        for d in row.get("pure_virtual_declarations", [])
    ]
    assert requirements
    assert all(d["signature"].endswith("= 0") for d in requirements)
    assert after != before


def test_pointer_only_use_does_not_claim_pure_virtual_construction(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    source = "void use(dart::A* value) {}"
    header.write_text("namespace dart { struct A { virtual void required(); }; }")
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    header.write_text("namespace dart { struct A { virtual void required() = 0; }; }")
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after == before


@pytest.mark.parametrize("initializer", ["= BASE", "{BASE}", "(BASE)", "((BASE + 1))"])
def test_default_constants_retain_each_initializer_form(
    extractor, tmp_path, initializer
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "namespace dart { constexpr int BASE = 1; constexpr int DEFAULT "
        + initializer
        + "; void run(int = DEFAULT); }"
    )
    header.write_text(text)
    before = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path / "before",
        "void use() { dart::run(); }",
    )
    dependencies = _entries(before, "free_functions")["dart::run"][
        "default_argument_declarations"
    ]
    assert len(dependencies) == 2
    assert any(d["signature"] == "constexpr int BASE = 1" for d in dependencies)
    header.write_text(text.replace("BASE = 1", "BASE = 2"))
    after = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path / "after",
        "void use() { dart::run(); }",
    )
    assert after != before


def test_direct_constant_indexing_preserves_function_declarations(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { constexpr int BASE(1); constexpr int DEFAULT(BASE); using Alias = int; constexpr int plain(int); constexpr int alias(Alias); constexpr int deduced(decltype(BASE)); constexpr int defaulted(int = BASE); const int variadic(...); void run(int = DEFAULT); }"
    )
    api = extractor.index_api(headers)
    assert {"dart::BASE", "dart::DEFAULT"} <= set(api.variables)
    assert {
        "dart::plain",
        "dart::alias",
        "dart::deduced",
        "dart::defaulted",
        "dart::variadic",
        "dart::run",
    } == set(api.free_functions)
    result = _scan(extractor, api, tmp_path, "void use() { dart::run(); }")
    assert (
        len(
            _entries(result, "free_functions")["dart::run"][
                "default_argument_declarations"
            ]
        )
        == 2
    )


@pytest.mark.parametrize(
    "kind, before_value, after_value",
    [
        ("bool", "true", "false"),
        ("int", "0x10", "0x20"),
        ("int", "1U", "2U"),
        ("double", "1e-3", "2e-3"),
    ],
)
def test_direct_literal_constants_retain_used_defaults(
    extractor, tmp_path, kind, before_value, after_value
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "namespace dart { constexpr "
        + kind
        + " DEFAULT("
        + before_value
        + "); void run("
        + kind
        + " = DEFAULT); }"
    )
    header.write_text(text)
    source = "void use() { dart::run(); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert (
        len(
            _entries(before, "free_functions")["dart::run"][
                "default_argument_declarations"
            ]
        )
        == 1
    )
    header.write_text(text.replace(before_value, after_value))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


def test_braced_class_constant_retains_initializer_dependencies(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { static constexpr int BASE = 1; static constexpr int DEFAULT{BASE}; void run(int = DEFAULT); }; }"
    header.write_text(text)
    source = "void use(dart::A& value) { value.run(); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    dependencies = _entries(before, "members")["run"]["declarations"][0][
        "default_argument_declarations"
    ]
    assert len(dependencies) == 2
    header.write_text(text.replace("BASE = 1", "BASE = 2"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


@pytest.mark.parametrize("specifier", ["alignof(B) * 2", "sizeof(B) * 2", "B"])
def test_alignment_expressions_retain_type_dependencies(extractor, tmp_path, specifier):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "namespace dart { struct alignas(16) B {}; struct alignas("
        + specifier
        + ") A {}; }"
    )
    header.write_text(text)
    source = "void use() { dart::A value; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    evidence = _entries(before, "types")["dart::A"]["alignment_declarations"]
    assert {d["class"] for d in evidence} == {"dart::A", "dart::B"}
    header.write_text(text.replace("alignas(16)", "alignas(32)"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


@pytest.mark.parametrize(
    "specifier", ["B*", "alignof(B*) * 2", "sizeof(B*) * 2", "Pointer"]
)
def test_alignment_pointer_types_do_not_depend_on_pointee_alignment(
    extractor, tmp_path, specifier
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "namespace dart { struct alignas(16) B {}; using Pointer = B*; struct alignas("
        + specifier
        + ") A {}; }"
    )
    header.write_text(text)
    source = "void use() { dart::A value; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert {
        d["class"]
        for d in _entries(before, "types")["dart::A"]["alignment_declarations"]
    } == {"dart::A"}
    header.write_text(text.replace("alignas(16)", "alignas(32)"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        == before
    )


def test_alignment_uses_enclosing_declaration_scope(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { constexpr int Alignment = 16; struct alignas(Alignment) A { static constexpr int Alignment = 64; }; }"
    header.write_text(text)
    source = "void use() { dart::A value; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    evidence = _entries(before, "types")["dart::A"]["alignment_declarations"]
    assert any(d["class"] == "dart" and "Alignment" in d["signature"] for d in evidence)
    assert not any("64" in d["signature"] for d in evidence)
    header.write_text(text.replace("Alignment = 16", "Alignment = 32"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )
    header.write_text(text.replace("Alignment = 64", "Alignment = 128"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "control", source)
        == before
    )


@pytest.mark.parametrize(
    "definition",
    [
        "struct B { B(); }; struct Base { using Owned = B; }; struct A : Base { Owned member; };",
        "struct Base { struct B { B(); }; }; struct A : Base { B member; };",
        "struct B { B(); }; struct Root { using Owned = B; }; struct Base : Root {}; struct A : Base { Owned member; };",
    ],
)
def test_owned_members_resolve_inherited_nested_types(extractor, tmp_path, definition):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { " + definition + " }"
    header.write_text(text)
    source = "void use() { dart::A value; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert any(
        d["class"].endswith("::B")
        for d in _entries(before, "constructors")["dart::A"]["declarations"]
    )
    header.write_text(text.replace("B();", "B(int);"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


def test_owned_inherited_type_hides_outer_namespace_type(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct B { B(); }; struct Base { struct B { B() = default; }; }; struct A : Base { B member; }; }"
    header.write_text(text)
    source = "void use() { dart::A value; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    declarations = _entries(before, "constructors")["dart::A"]["declarations"]
    assert {d["class"] for d in declarations} == {"dart::Base::B"}
    header.write_text(text.replace("B();", "B(int);"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        == before
    )


def test_enum_data_declarations_have_no_parameter_defaults(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text("namespace dart { enum class E { Value = 1 }; }")
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use() { auto value = dart::E::Value; }",
    )
    assert all(
        "default_argument_declarations" not in d
        for row in result["data_members"]
        for d in row["declarations"]
    )


@pytest.mark.parametrize(
    "prefix, base, imported",
    [
        ("", "dart::A", "dart::A::A"),
        ("", "dart::A", "A::A"),
        ("using Alias = dart::A;", "Alias", "Alias::A"),
        ("namespace d = dart;", "d::A", "d::A::A"),
    ],
)
def test_inherited_constructor_import_does_not_hide_base_type(
    extractor, tmp_path, prefix, base, imported
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { A(); void hit(); }; }"
    header.write_text(text)
    source = (
        prefix
        + " struct Derived : "
        + base
        + " { using "
        + imported
        + "; void use() { A value; value.hit(); } };"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "members")["hit"]
    assert row["receiver_types"] == ["dart::A"]
    assert row["unresolved_count"] == 0
    assert _entries(before, "constructors")["dart::A"]["count"] >= 2
    header.write_text(text.replace("hit()", "hit(int)"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


def test_block_type_import_with_repeated_namespace_name_still_resolves(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct A { A(); void hit(); }; }"
    )
    source = "namespace same { using same = dart::A; } struct Derived { void use() { using same::same; same value; value.hit(); } };"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(result, "members")["hit"]["receiver_types"] == ["dart::A"]


@pytest.mark.parametrize(
    "expression",
    [
        "std::allocate_shared<dart::A>(std::allocator<dart::A>{})",
        "::std::allocate_shared<dart::A>(std::allocator<dart::A>{})",
        "allocate_shared<dart::A>(std::allocator<dart::A>{})",
        "alloc::allocate_shared<dart::A>(std::allocator<dart::A>{})",
        "std::allocate_shared<Value>(std::allocator<Value>{})",
        "std::allocate_shared<dart::A, std::allocator<dart::A>>(std::allocator<dart::A>{})",
    ],
)
def test_allocate_shared_retains_construction_dependencies(
    extractor, tmp_path, expression
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "#include <memory>\nnamespace dart { struct A { A(); }; }"
    header.write_text(text)
    source = (
        "namespace alloc = std; using std::allocate_shared; using Value = dart::A; void use() { auto value = "
        + expression
        + "; }"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "constructors")["dart::A"]
    assert row["count"] == 1
    assert any(d["signature"] == "A ( )" for d in row["declarations"])
    header.write_text(text.replace("A();", "A(int);"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after != before


def test_shared_pointer_type_does_not_construct_pointee(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "#include <memory>\nnamespace dart { struct A { A(); }; }"
    header.write_text(text)
    source = "void use() { std::shared_ptr<dart::A> pointer; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert not before["constructors"]
    header.write_text(text.replace("A();", "A(int);"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        == before
    )


@pytest.mark.parametrize(
    "declarations, names, change",
    [
        ("unsigned flags : 3;", ["flags"], (" : 3", " : 4")),
        ("unsigned flags : 3, mode : 2;", ["flags", "mode"], (" : 3", " : 4")),
        (
            "unsigned : 0, flags : 3, : 0, mode : 2;",
            ["flags", "mode"],
            (" : 3", " : 4"),
        ),
        (
            "unsigned long : 0, flags : 3, ordinary, mode : 2;",
            ["flags", "ordinary", "mode"],
            (" : 3", " : 4"),
        ),
        (
            "std::uint32_t : 0, flags : 3, mode : 2;",
            ["flags", "mode"],
            (" : 3", " : 4"),
        ),
        (
            "unsigned flags : (WIDTH ? 3 : 2), mode : 2;",
            ["flags", "mode"],
            ("? 3", "? 4"),
        ),
        (
            "unsigned flags : WIDTH ? 3 : 2, mode : 2;",
            ["flags", "mode"],
            ("? 3", "? 4"),
        ),
        (
            "unsigned flags : WIDTH < 4 ? 3 : 2, mode : 2;",
            ["flags", "mode"],
            ("? 3", "? 4"),
        ),
        (
            "unsigned flags : sizeof(unsigned), mode : 2;",
            ["flags", "mode"],
            ("sizeof(unsigned)", "(sizeof(unsigned) + 1)"),
        ),
        ("bool enabled : 1, ready : 1;", ["enabled", "ready"], ("bool", "unsigned")),
    ],
)
def test_named_bit_fields_keep_grouped_declarations_and_aggregate_order(
    extractor, tmp_path, declarations, names, change
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "#include <cstdint>\nnamespace dart { constexpr int WIDTH = 3; struct A { "
        + declarations
        + " }; }"
    )
    header.write_text(text)
    source = (
        "unsigned use() { dart::A value{"
        + ", ".join("1" for _ in names)
        + "}; return "
        + " + ".join("value." + name for name in names)
        + "; }"
    )
    api = extractor.index_api(headers)
    assert set(api.data_members) == set(names)
    assert not api.members  # widths containing sizeof are not function declarations
    before = _scan(extractor, api, tmp_path / "before", source)
    assert set(_entries(before, "data_members")) == set(names)
    aggregate = _entries(before, "constructors")["dart::A"]["brace_member_declarations"]
    assert [d["name"] for d in aggregate] == names
    header.write_text(text.replace(*change))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after != before


@pytest.mark.parametrize("width", ["WIDTH", "::dart::WIDTH", "(WIDTH ? WIDTH : 2)"])
@pytest.mark.parametrize(
    "source",
    [
        "void use(dart::A& value) { (void)value.flags; }",
        "void use() { dart::A value{1}; }",
    ],
)
def test_named_bit_field_widths_retain_constant_dependencies(
    extractor, tmp_path, width, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "namespace dart { constexpr int BASE = 1; constexpr int WIDTH = BASE + 2; struct A { unsigned flags : "
        + width
        + "; }; }"
    )
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    if before["data_members"]:
        declaration = _entries(before, "data_members")["flags"]["declarations"][0]
    else:
        declaration = _entries(before, "constructors")["dart::A"][
            "brace_member_declarations"
        ][0]
    evidence = declaration["bit_width_declarations"]
    assert {d["signature"] for d in evidence} == {
        "constexpr int BASE = 1",
        "constexpr int WIDTH = BASE + 2",
    }
    header.write_text(text.replace("BASE = 1", "BASE = 2"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after != before
    assert not before["variables"]


def test_private_bit_width_constant_is_not_a_readable_member(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { class A { static constexpr int WIDTH = 3; public: unsigned flags : WIDTH; }; }"
    )
    api = extractor.index_api(headers)
    assert set(api.data_members) == {"flags"}
    result = _scan(
        extractor, api, tmp_path, "void use(dart::A& value) { (void)value.flags; }"
    )
    evidence = _entries(result, "data_members")["flags"]["declarations"][0][
        "bit_width_declarations"
    ]
    assert [(d["class"], d["signature"]) for d in evidence] == [
        ("dart::A", "static constexpr int WIDTH = 3")
    ]


def test_bit_field_colons_preserve_constructor_initializers_and_scoped_types(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "#include <cstdint>\nnamespace dart { constexpr int WIDTH = 3; struct A { std::uint32_t flags : WIDTH, mode : 2; A() : flags(1), mode(2) {} }; }"
    )
    api = extractor.index_api(headers)
    assert set(api.data_members) == {"flags", "mode"}
    result = _scan(
        extractor,
        api,
        tmp_path,
        "unsigned use() { dart::A value; return value.flags + value.mode; }",
    )
    assert set(_entries(result, "data_members")) == {"flags", "mode"}
    assert [
        d["signature"]
        for d in _entries(result, "constructors")["dart::A"]["declarations"]
    ] == ["A ( )"]


@pytest.mark.parametrize(
    "preamble, declarations, initializer",
    [
        ("using Index = unsigned;", "Index : 0, flags : 3, mode : 2;", "1"),
        ("typedef unsigned Index;", "Index : 0, flags : 3, mode : 2;", "1"),
        (
            "namespace Types { using Index = unsigned; }",
            "Types::Index : 0, flags : 3, mode : 2;",
            "1",
        ),
        (
            "enum class Index : unsigned { one = 1 };",
            "Index : 0, flags : 3, mode : 2;",
            "dart::Index::one",
        ),
        (
            "namespace Types { enum class Index : unsigned { one = 1 }; }",
            "Types::Index : 0, flags : 3, mode : 2;",
            "dart::Types::Index::one",
        ),
        (
            "struct Types { using Index = unsigned; };",
            "typename Types::Index : 0, flags : 3, mode : 2;",
            "1",
        ),
        ("using Index = unsigned;", "const Index flags : 3, mode : 2;", "1"),
        ("using Index = unsigned;", "volatile Index flags : 3, mode : 2;", "1"),
    ],
)
def test_bit_fields_keep_alias_enum_and_qualified_types(
    extractor, tmp_path, preamble, declarations, initializer
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { " + preamble + " struct A { " + declarations + " }; }"
    header.write_text(text)
    source = (
        "unsigned use() { dart::A value{"
        + initializer
        + ", "
        + initializer
        + "}; return static_cast<unsigned>(value.flags) + static_cast<unsigned>(value.mode); }"
    )
    api = extractor.index_api(headers)
    assert {
        name
        for name, items in api.data_members.items()
        if any(item["class"] == "dart::A" for item in items)
    } == {"flags", "mode"}
    before = _scan(extractor, api, tmp_path / "before", source)
    assert {
        name
        for name, row in _entries(before, "data_members").items()
        if any(item["class"] == "dart::A" for item in row["declarations"])
    } == {"flags", "mode"}
    fields = _entries(before, "constructors")["dart::A"]["brace_member_declarations"]
    assert [d["name"] for d in fields] == ["flags", "mode"]
    header.write_text(text.replace("flags : 3", "flags : 4"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


@pytest.mark.parametrize("qualifiers", ["const", "volatile", "const volatile"])
def test_unnamed_cv_bit_fields_do_not_invent_members(extractor, qualifiers):
    # GCC accepts these unnamed cv-qualified bit-fields; Clang rejects them.
    values = extractor.Tokens(qualifiers + " Index : 0, flags : 3, mode : 2").values
    fields = extractor.data_declarations(values, "dart::A")
    assert [field["name"] for field in fields] == ["flags", "mode"]
    assert all(
        field["return_tokens"] == qualifiers.split() + ["Index"] for field in fields
    )


def test_dependent_typename_unnamed_bit_field_preserves_shared_type(extractor):
    values = extractor.Tokens("typename T::Index : 0, flags : 3, mode : 2").values
    fields = extractor.data_declarations(values, "dart::A")
    assert [field["name"] for field in fields] == ["flags", "mode"]
    assert all(
        field["return_tokens"] == ["typename", "T", "::", "Index"] for field in fields
    )


WRAPPER_API_HEADER = """namespace dart {
struct Target { void other(); };
struct Item { void hit(); Target* operator->(); int value(); bool has_value(); };
using Pointer = Item*;
using MaybeShared = std::optional<std::shared_ptr<Item>>;
using Maybe = std::optional<Item>;
using MaybeAlias = Maybe;
using MaybePointer = std::optional<Pointer>;
struct Factory {
  Item make();
  MaybeShared shared();
  std::vector<std::optional<Item>> optionalValues();
  std::array<std::optional<Item>, 2> optionalArray();
  std::vector<std::optional<Item*>> optionalPointers();
  Item* pointer();
  std::vector<Item*> items();
  std::optional<Item> maybe();
  std::optional<Item*> maybePointer();
  std::optional<std::vector<Item*>> maybeItems();
  MaybeAlias alias();
  MaybePointer pointerAlias();
};
struct Holder { std::optional<Item> item; };
}
"""


@pytest.mark.parametrize(
    "wrapper", ["std::move", "std::forward<decltype(factory.make())>"]
)
def test_standard_forwarding_wrappers_keep_return_receivers(
    extractor, tmp_path, wrapper
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(WRAPPER_API_HEADER)
    source = (
        "void use(dart::Factory& factory) { " + wrapper + "(factory.make()).hit(); }"
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::Item"]
    assert hit["unresolved_count"] == 0
    header.write_text(WRAPPER_API_HEADER.replace("void hit();", "void hit(int);"))
    changed = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(changed, "members")["hit"]["declarations"] != hit["declarations"]


def test_standard_forwarding_wrappers_keep_pointer_container_and_array_shapes(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """namespace s = std;
void use(dart::Factory& factory) {
  std::move(factory.pointer())->hit();
  s::move(factory.items()).front()->hit();
  ::std::forward<decltype(factory.pointer())>(factory.pointer())->hit();
  dart::Item values[2]; std::move(values)->hit();
  std::forward<decltype(values)>(values)[0]->other();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 4
    assert members["hit"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 1
    assert members["other"]["receiver_types"] == ["dart::Target"]


def test_forwarding_wrappers_preserve_imports_foreign_receivers_and_callable_shadows(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Foreign { void hit(); };
namespace other { Foreign move(dart::Item); }
struct Callable { Foreign operator()(dart::Item); };
using std::move;
void use(dart::Factory& factory, Foreign foreign, Callable call) {
  move(factory.make()).hit();
  other::move(factory.make()).hit();
  std::move(foreign).hit();
  { auto move = call; move(factory.make()).hit(); }
  move(factory.make()).hit();
}
template<class Callable> void unknown(dart::Item* known, Callable move) { move(*known).hit(); }
""",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 4
    assert hit["unresolved_count"] == 2


@pytest.mark.parametrize(
    "expression",
    [
        "factory.maybe().value().hit()",
        "factory.maybe()->hit()",
        "(*factory.maybe()).hit()",
    ],
)
def test_optional_return_access_retains_member_dependencies(
    extractor, tmp_path, expression
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(WRAPPER_API_HEADER)
    source = "void use(dart::Factory& factory) { " + expression + "; }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::Item"]
    assert hit["unresolved_count"] == 0
    assert "value" not in _entries(result, "members")
    assert "operator->" not in _entries(result, "members")
    header.write_text(WRAPPER_API_HEADER.replace("void hit();", "void hit(int);"))
    changed = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(changed, "members")["hit"]["declarations"] != hit["declarations"]


def test_optional_shapes_keep_value_pointer_container_and_alias_distinctions(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """void use(dart::Factory& factory, dart::Holder& holder) {
  factory.maybePointer().value()->hit();
  factory.pointerAlias().value()->hit();
  factory.maybeItems().value().front()->hit();
  factory.alias().value().hit();
  holder.item.value().hit();
  auto item = factory.maybe(); item->hit();
  item.value()->other(); (*item)->other();
  item.has_value();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 6
    assert members["hit"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 2
    assert members["other"]["count"] == 2
    assert "value" not in members
    assert "has_value" not in members


def test_optional_source_bindings_and_aliases_keep_wrapper_access(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """using Maybe = std::optional<dart::Item>;
using Again = Maybe;
struct Holder { Again item; };
void use(Holder& holder) {
  std::optional<dart::Item> direct;
  direct.value().hit(); direct->hit(); (*direct).hit();
  Again alias; alias.value().hit(); alias->hit();
  dart::MaybeAlias apiAlias; apiAlias.value().hit();
  holder.item.value().hit();
  std::optional<dart::Item*> pointer; pointer.value()->hit();
  std::optional<std::vector<dart::Item*>> items; items.value().front()->hit();
  alias.has_value(); holder.item.has_value();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 9
    assert members["hit"]["unresolved_count"] == 0
    assert "operator->" not in members
    assert "value" not in members
    assert "has_value" not in members


def test_optional_foreign_and_unknown_results_preserve_shadowing(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Foreign { void hit(); };
struct Factory { std::optional<Foreign> maybe(); };
void use(dart::Item* known, Factory& factory) {
  auto item = known;
  { auto item = factory.maybe(); item.value().hit(); item->hit(); (*item).hit(); }
  item->hit();
}
template<class T> struct Unknown { std::optional<T> maybe(); };
template<class T> void unknown(dart::Item* known, Unknown<T>& factory) { factory.maybe().value().hit(); }
""",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 2
    assert hit["unresolved_count"] == 1


def test_forwarding_import_directives_and_qualified_foreign_names(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Foreign { void hit(); };
namespace other { Foreign move(dart::Item); }
using namespace std;
void use(dart::Factory& factory, Foreign foreign) {
  move(factory.make()).hit();
  forward<decltype(factory.pointer())>(factory.pointer())->hit();
  move(foreign).hit();
  { using other::move; move(factory.make()).hit(); }
  move(factory.make()).hit();
}
namespace own { namespace std { Foreign move(dart::Item); }
void use(dart::Factory& factory) { std::move(factory.make()).hit(); }
}
""",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 3
    assert hit["unresolved_count"] == 0


def test_explicit_forwarding_target_keeps_static_base_and_pointer_types(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Derived : dart::Item { void hit(); };
void use(Derived& derived, Derived* pointer) {
  std::forward<dart::Item&>(derived).hit();
  std::forward<dart::Item*>(pointer)->hit();
  std::move<dart::Item&>(derived).hit();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 3
    assert members["hit"]["receiver_types"] == ["dart::Item"]
    assert members["hit"]["unresolved_count"] == 0
    assert "operator->" not in members


def test_optional_namespace_aliases_keep_outer_type_and_source_fields(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """namespace s = std;
using Maybe = s::optional<dart::Item>;
struct Factory { s::optional<dart::Item> maybe(); s::optional<s::vector<dart::Item*>> items(); };
struct Holder { s::optional<dart::Item> maybe; dart::Item child; dart::Item* pointer; std::vector<dart::Item*> children; };
void use(Factory& factory, Holder& holder) {
  s::optional<s::vector<dart::Item*>> items; items.value().front()->hit();
  Maybe alias; alias->hit();
  factory.maybe().value().hit(); factory.items().value().front()->hit();
  holder.maybe.value().hit(); holder.child.hit(); holder.pointer->hit(); holder.children.front()->hit();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 8
    assert members["hit"]["unresolved_count"] == 0
    assert "value" not in members
    assert "operator->" not in members


def test_optional_pointer_array_and_auto_dereference_shapes(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """void use(dart::Factory& factory, std::optional<dart::Item>* pointer) {
  pointer->value().hit();
  std::optional<dart::Item> values[2]; values[0]->hit(); std::move(values)[0]->hit();
  auto item = *factory.maybePointer(); item->hit();
  auto value = *factory.maybe(); value->other();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 4
    assert members["hit"]["unresolved_count"] == 0
    assert members["operator->"]["count"] == 1
    assert members["other"]["receiver_types"] == ["dart::Target"]


@pytest.mark.parametrize(
    "shape, expression",
    [
        ("std::vector<std::optional<Item>>", "factory.values().front().value().hit()"),
        ("std::array<std::optional<Item>, 2>", "factory.values().at(0).value().hit()"),
        ("std::vector<std::optional<Item*>>", "factory.values().back().value()->hit()"),
    ],
)
def test_sequence_returns_preserve_optional_element_shapes(
    extractor, tmp_path, shape, expression
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    api_text = WRAPPER_API_HEADER.replace(
        "  Item make();", "  Item make();\n  " + shape + " values();"
    )
    header.write_text(api_text)
    source = "void use(dart::Factory& factory) { " + expression + "; }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    members = _entries(result, "members")
    assert members["hit"]["receiver_types"] == ["dart::Item"]
    assert members["hit"]["unresolved_count"] == 0
    assert "value" not in members
    assert "operator->" not in members
    header.write_text(api_text.replace("void hit();", "void hit(int);"))
    changed = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert (
        _entries(changed, "members")["hit"]["declarations"]
        != members["hit"]["declarations"]
    )


def test_nested_optional_sequences_preserve_explicit_bindings_and_foreign_elements(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Foreign { void hit(); };
std::vector<std::optional<dart::Item>> values();
std::vector<std::optional<Foreign>> foreign();
void use() {
  values().front().value().hit();
  std::array<std::optional<dart::Item*>, 2> pointers; pointers[0].value()->hit();
  std::vector<std::vector<std::optional<dart::Item>>> nested; nested.front().front()->hit();
  foreign().front()->hit();
  std::vector<std::optional<Foreign>> local; local.front().value().hit();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 3
    assert members["hit"]["unresolved_count"] == 0
    assert "value" not in members
    assert "operator->" not in members


def test_optional_standard_pointer_elements_keep_inner_pointer_identity(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = WRAPPER_API_HEADER
    (headers / "api.hpp").write_text(header)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """void use(dart::Factory& factory) {
  factory.shared().value()->hit(); (*factory.shared())->hit();
  std::optional<std::shared_ptr<dart::Item>> item; item.value()->hit(); (*item)->hit();
}
""",
    )
    members = _entries(result, "members")
    assert members["hit"]["count"] == 4
    assert members["hit"]["unresolved_count"] == 0
    assert "operator->" not in members
    assert "value" not in members


@pytest.mark.parametrize(
    "expression",
    [
        "factory.maybe().value_or(factory.make()).hit()",
        "factory.maybePointer().value_or(nullptr)->hit()",
        "auto item = factory.maybe(); item.value_or(factory.make()).hit()",
        "std::move(factory.maybe()).value_or(factory.make()).hit()",
    ],
)
def test_optional_value_or_keeps_optional_static_result_type(
    extractor, tmp_path, expression
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(WRAPPER_API_HEADER)
    source = "void use(dart::Factory& factory) { " + expression + "; }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    members = _entries(result, "members")
    assert members["hit"]["count"] == 1
    assert members["hit"]["receiver_types"] == ["dart::Item"]
    assert members["hit"]["unresolved_count"] == 0
    assert "operator->" not in members
    header.write_text(WRAPPER_API_HEADER.replace("void hit();", "void hit(int);"))
    changed = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert (
        _entries(changed, "members")["hit"]["declarations"]
        != members["hit"]["declarations"]
    )


def test_optional_value_or_foreign_target_does_not_inherit_default_type(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Foreign { Foreign(dart::Item); void hit(); };
struct Factory { std::optional<Foreign> maybe(); };
void use(Factory& factory, dart::Factory& dartFactory) { factory.maybe().value_or(dartFactory.make()).hit(); }
""",
    )
    assert "hit" not in _entries(result, "members")


@pytest.mark.parametrize("comparison", ["<", ">"])
def test_forwarding_decltype_target_balanced_groups_keep_argument_receiver(
    extractor, tmp_path, comparison
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(WRAPPER_API_HEADER)
    source = (
        "constexpr int N = 1; void use(dart::Factory& factory) { std::forward<decltype(N "
        + comparison
        + " 2 ? factory.make() : factory.make())>(factory.make()).hit(); }"
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::Item"]
    assert hit["unresolved_count"] == 0
    header.write_text(WRAPPER_API_HEADER.replace("void hit();", "void hit(int);"))
    changed = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(changed, "members")["hit"]["declarations"] != hit["declarations"]


@pytest.mark.parametrize(
    "imported",
    [
        "using namespace std;",
        "using std::optional;",
        "namespace s = std; using s::optional;",
    ],
)
def test_imported_optional_bindings_aliases_and_returns_keep_wrapper_shape(
    extractor, tmp_path, imported
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(WRAPPER_API_HEADER)
    source = (
        imported
        + """
using Maybe = optional<dart::Item>;
struct Factory { optional<dart::Item> maybe(); optional<dart::Item*> pointer(); };
struct Holder { optional<dart::Item> item; };
void use(Factory& factory, Holder& holder) {
  optional<dart::Item> direct; direct.value().hit(); direct->hit();
  Maybe alias; alias.value().hit();
  factory.maybe().value().hit(); factory.pointer().value()->hit();
  holder.item.value().hit();
}
"""
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    members = _entries(result, "members")
    assert members["hit"]["count"] == 6
    assert members["hit"]["unresolved_count"] == 0
    assert "value" not in members
    assert "operator->" not in members
    header.write_text(WRAPPER_API_HEADER.replace("void hit();", "void hit(int);"))
    changed = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert (
        _entries(changed, "members")["hit"]["declarations"]
        != members["hit"]["declarations"]
    )


def test_imported_optional_type_shadows_keep_foreign_and_directive_lifetimes(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(WRAPPER_API_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        """struct Foreign { void hit(); };
using namespace std;
namespace foreign {
template<class T> struct optional { Foreign value(); };
void use() { optional<dart::Item> item; item.value().hit(); }
}
namespace imported {
using foreign::optional;
void use() { optional<dart::Item> item; item.value().hit(); }
}
void use() {
  { using std::optional; optional<dart::Item> item; item.value().hit(); }
  optional<dart::Item> item; item->hit();
}
""",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["count"] == 2
    assert hit["unresolved_count"] == 0


@pytest.mark.parametrize(
    "source",
    [
        "void use() { auto a = std::make_optional<dart::A>(); }",
        "void use() { auto a = ::std::make_optional<dart::A>(); }",
        "using std::make_optional; void use() { auto a = make_optional<dart::A>(); }",
        "namespace make = std; void use() { auto a = make::make_optional<dart::A>(); }",
        "using Value = dart::A; void use() { auto a = std::make_optional<Value>(); }",
        "void use() { auto a = std::make_from_tuple<dart::A>(std::tuple<>{}); }",
        "void use() { auto a = std::make_from_tuple<dart::A, std::tuple<>>(std::tuple<>{}); }",
        "void use() { std::optional<dart::A> a(std::in_place); }",
        "void use() { std::optional<const dart::A> a{std::in_place}; }",
        "void use() { ::std::optional<dart::A> a{::std::in_place}; }",
        "void use() { auto a = std::optional<dart::A>(std::in_place); }",
        "void use() { auto a = std::optional<dart::A>{std::in_place}; }",
        "using Value = dart::A; void use() { std::optional<Value> a{std::in_place}; }",
        "void use() { alignas(dart::A) unsigned char bytes[sizeof(dart::A)]; auto a = new(bytes) dart::A; }",
        "void use() { alignas(dart::A) unsigned char bytes[sizeof(dart::A)]; auto a = ::new((bytes)) ::dart::A; }",
        "using Value = dart::A; void use() { alignas(Value) unsigned char bytes[sizeof(Value)]; auto a = new(static_cast<void*>(bytes)) Value; }",
    ],
)
def test_explicit_optional_tuple_and_placement_construction(
    extractor, tmp_path, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "#include <optional>\n#include <tuple>\n#include <new>\nnamespace dart { struct A { A(); explicit A(int); }; }"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "constructors")["dart::A"]
    assert row["count"] == 1
    assert any(item["signature"] == "A ( )" for item in row["declarations"])
    header.write_text(text.replace("A();", "A() = delete;"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after != before


@pytest.mark.parametrize(
    "source",
    [
        "void use() { std::optional<dart::A> a; }",
        "void use() { std::optional<dart::A> a{}; }",
        "void use() { std::optional<dart::A> a(std::nullopt); }",
        "void use() { std::optional<dart::A> a{std::nullopt}; }",
        "void use() { std::optional<dart::A>* a = nullptr; }",
        "void use() { std::optional<dart::A*> a{std::in_place}; }",
        "struct Holder { static std::optional<dart::A> value; };",
        "extern std::optional<dart::A> value;",
        "using Maybe = std::optional<dart::A>; void use() { Maybe a; }",
        "void use() { std::in_place_type_t<dart::A> tag; }",
        "void use() { auto tag = std::in_place_type<dart::A>; }",
        "void use() { std::shared_ptr<dart::A> a; }",
        "void use() { auto a = static_cast<dart::A*>(nullptr); }",
        "using Factory = std::optional<dart::A>(*)(); Factory factory = nullptr;",
        "void use() { alignas(dart::A*) unsigned char bytes[sizeof(dart::A*)]; auto a = new(bytes) dart::A*; }",
        "void use() { auto a = new dart::A*; }",
    ],
)
def test_type_only_optional_and_tag_uses_do_not_construct(extractor, tmp_path, source):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "#include <optional>\n#include <tuple>\n#include <new>\n#include <memory>\nnamespace dart { struct A { A(); explicit A(int); }; }"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert not before["constructors"]
    header.write_text(text.replace("A();", "A() = delete;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        == before
    )


def test_optional_in_place_type_tag_is_not_optional_engagement(extractor):
    # std::optional takes in_place_t, not the in_place_type_t tag used by std::any.
    tokens = extractor.Tokens("std::optional<dart::A> a(std::in_place_type<dart::A>);")
    start, end, _ = next(
        item for item in extractor.names_in(tokens.values) if item[2] == "dart::A"
    )
    assert not extractor.constructs(tokens, start, end, set(), [])


@pytest.mark.parametrize(
    "source",
    [
        "void use() { std::optional<dart::A> a{std::in_place, 1}; }",
        "void use() { auto a = std::make_optional<dart::A>(1); }",
        "void use() { auto a = std::make_from_tuple<dart::A>(std::tuple<int>{1}); }",
        "void use() { alignas(dart::A) unsigned char bytes[sizeof(dart::A)]; auto a = new(bytes) dart::A(1); }",
    ],
)
def test_explicit_construction_arguments_retain_constructor_candidates(
    extractor, tmp_path, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "#include <optional>\n#include <tuple>\n#include <new>\nnamespace dart { struct A { A(); explicit A(int); }; }"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "constructors")["dart::A"]
    assert row["count"] == 1
    assert any(
        item["signature"] == "explicit A ( int )" for item in row["declarations"]
    )
    header.write_text(text.replace("explicit A(int);", "explicit A(int) = delete;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


def test_factory_template_id_without_call_does_not_construct(extractor):
    tokens = extractor.Tokens("&std::make_optional<dart::A>;")
    start, end, _ = next(
        item for item in extractor.names_in(tokens.values) if item[2] == "dart::A"
    )
    assert not extractor.constructs(tokens, start, end, set(), [])


@pytest.mark.parametrize(
    "api_aliases, source, engaged",
    [
        (
            "",
            "using Maybe = std::optional<dart::A>; void use() { Maybe a{std::in_place}; }",
            True,
        ),
        (
            "",
            "using Maybe = std::optional<dart::A>; using Other = Maybe; void use() { Other a(std::in_place); }",
            True,
        ),
        (
            "using Maybe = std::optional<A>;",
            "void use() { dart::Maybe a{std::in_place}; }",
            True,
        ),
        (
            "using Maybe = std::optional<A>; using Other = Maybe;",
            "void use() { dart::Other a(std::in_place); }",
            True,
        ),
        (
            "using Maybe = std::optional<const A>;",
            "void use() { dart::Maybe a{std::in_place}; }",
            True,
        ),
        (
            "",
            "using Maybe = std::optional<const dart::A>; void use() { Maybe a{std::in_place}; }",
            True,
        ),
        (
            "",
            "using std::optional; using Maybe = optional<dart::A>; void use() { Maybe a{std::in_place}; }",
            True,
        ),
        (
            "",
            "namespace opt = std; using Maybe = opt::optional<dart::A>; void use() { Maybe a{std::in_place}; }",
            True,
        ),
        (
            "",
            "using Ptr = dart::A*; using Maybe = std::optional<Ptr>; void use() { Maybe a{std::in_place}; }",
            False,
        ),
        (
            "using Ptr = A*; using Maybe = std::optional<Ptr>;",
            "void use() { dart::Maybe a{std::in_place}; }",
            False,
        ),
        ("", "using Maybe = std::optional<dart::A>; void use() { Maybe a; }", False),
        (
            "using Maybe = std::optional<A>;",
            "void use() { dart::Maybe a{std::nullopt}; }",
            False,
        ),
    ],
)
def test_optional_alias_engagement_uses_declaration_context(
    extractor, tmp_path, api_aliases, source, engaged
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "#include <optional>\nnamespace dart { struct A { A(); explicit A(int); }; "
        + api_aliases
        + " }"
    )
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    if engaged:
        assert len(before["constructors"]) == 1
        row = before["constructors"][0]
        assert row["count"] == 1
        assert any(
            item["class"] == "dart::A" and item["signature"] == "A ( )"
            for item in row["declarations"]
        )
    else:
        assert not before["constructors"]
    header.write_text(text.replace("A();", "A() = delete;"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (after != before) is engaged


VALUE_FACTORIES = [
    "std::make_shared<{target}>()",
    "std::make_unique<{target}>()",
    "std::allocate_shared<{target}>(std::allocator<std::remove_cv_t<{target}>>{{}})",
    "std::make_optional<{target}>()",
    "std::make_from_tuple<{target}>(std::tuple<>{{}})",
]
POINTER_FACTORIES = VALUE_FACTORIES[:3] + [
    "std::make_optional<{target}>(nullptr)",
    "std::make_from_tuple<{target}>(std::tuple<decltype(nullptr)>{{nullptr}})",
]
FACTORY_API_HEADER = "#include <memory>\n#include <optional>\n#include <tuple>\n#include <array>\n#include <type_traits>\nnamespace dart { struct A { A(); }; template<class T> struct Box { Box(); }; }"


@pytest.mark.parametrize("factory", POINTER_FACTORIES)
@pytest.mark.parametrize("target", ["dart::A*", "dart::A(*)()"])
def test_factory_pointer_targets_do_not_construct_returned_class(
    extractor, tmp_path, factory, target
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(FACTORY_API_HEADER)
    source = "void use() { auto value = " + factory.format(target=target) + "; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert not before["constructors"]
    header.write_text(FACTORY_API_HEADER.replace("A();", "A() = delete;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        == before
    )


@pytest.mark.parametrize("factory", VALUE_FACTORIES)
@pytest.mark.parametrize(
    "target", ["dart::A", "const dart::A", "volatile dart::A", "const volatile dart::A"]
)
def test_factory_cv_value_targets_retain_constructor_dependencies(
    extractor, tmp_path, factory, target
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(FACTORY_API_HEADER)
    source = "void use() { auto value = " + factory.format(target=target) + "; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert _entries(before, "constructors")["dart::A"]["count"] == 1
    header.write_text(FACTORY_API_HEADER.replace("A();", "A() = delete;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


@pytest.mark.parametrize(
    "source",
    [
        "void use(dart::A& original) { auto& value = std::make_from_tuple<dart::A&>(std::tuple<dart::A&>{original}); }",
        "void use(dart::A& original) { auto&& value = std::make_from_tuple<dart::A&&>(std::tuple<dart::A&&>{std::move(original)}); }",
    ],
)
def test_tuple_factory_reference_target_does_not_construct(extractor, tmp_path, source):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(FACTORY_API_HEADER)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert not before["constructors"]
    header.write_text(FACTORY_API_HEADER.replace("A();", "A() = delete;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        == before
    )


@pytest.mark.parametrize("factory", VALUE_FACTORIES)
def test_factory_nested_pointer_argument_does_not_hide_value_constructor(
    extractor, tmp_path, factory
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(FACTORY_API_HEADER)
    source = (
        "void use() { auto value = "
        + factory.format(target="dart::Box<dart::A*>")
        + "; }"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert set(_entries(before, "constructors")) == {"dart::Box"}
    header.write_text(FACTORY_API_HEADER.replace("Box();", "Box() = delete;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


@pytest.mark.parametrize("factory", VALUE_FACTORIES)
def test_factory_owned_array_value_keeps_element_construction(
    extractor, tmp_path, factory
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(FACTORY_API_HEADER)
    source = (
        "void use() { auto value = "
        + factory.format(target="std::array<dart::A, sizeof(void*)>")
        + "; }"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert _entries(before, "constructors")["dart::A"]["count"] == 1
    header.write_text(FACTORY_API_HEADER.replace("A();", "A() = delete;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


def test_unique_factory_array_value_keeps_element_construction(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(FACTORY_API_HEADER)
    source = "void use() { auto value = std::make_unique<dart::A[]>(2); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert _entries(before, "constructors")["dart::A"]["count"] == 1
    header.write_text(FACTORY_API_HEADER.replace("A();", "A() = delete;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


def _scan_receiver_api(extractor, tmp_path, header, source):
    headers = tmp_path / "dart"
    headers.mkdir(exist_ok=True)
    (headers / "api.hpp").write_text(header)
    return _scan(extractor, extractor.index_api(headers), tmp_path, source)


CONDITIONAL_API = """namespace dart {
struct A { void hit(); };
struct B { operator A(); void hit(); };
A makeA(); B makeB();
}"""


@pytest.mark.parametrize(
    "expression",
    [
        "flag ? dart::makeA() : dart::makeB()",
        "(flag ? dart::makeA() : dart::makeB())",
        "flag ? (other ? dart::makeB() : dart::makeA()) : dart::makeB()",
        "flag ? other ? dart::makeB() : dart::makeA() : dart::makeB()",
        "flag ? dart::makeB() : other ? dart::makeA() : dart::makeB()",
        "std::move(flag ? dart::makeA() : dart::makeB())",
    ],
)
def test_conditional_receiver_mutation(extractor, tmp_path, expression):
    source = "void use(bool flag, bool other) { auto x = " + expression + "; x.hit(); }"
    changed = CONDITIONAL_API.replace(
        "struct A { void hit();", "struct A { void hit() = delete;"
    )
    before = _scan_receiver_api(extractor, tmp_path, CONDITIONAL_API, source)
    hit = _entries(before, "members")["hit"]
    assert "dart::A" in hit["receiver_types"]
    assert any(d["class"] == "dart::A" for d in hit["declarations"])
    assert _scan_receiver_api(extractor, tmp_path, changed, source) != before


ITERATOR_API = """namespace dart {
struct Target { void other(); };
struct A { void hit() const; Target* operator->(); };
struct Factory { std::vector<A> values(); std::vector<A*> pointers(); std::array<A, 2> array(); std::array<A*, 2> pointerArray(); };
}"""


@pytest.mark.parametrize(
    "method", ["begin", "cbegin", "rbegin", "crbegin", "end", "cend", "rend", "crend"]
)
@pytest.mark.parametrize(
    "container, access",
    [
        ("values", "it->hit()"),
        ("pointers", "(*it)->hit()"),
        ("array", "(*it).hit()"),
        ("pointerArray", "auto p = *it; p->hit()"),
    ],
)
def test_iterator_receiver_mutation(extractor, tmp_path, method, container, access):
    source = (
        "void use(dart::Factory& factory) { auto it = factory."
        + container
        + "()."
        + method
        + "(); "
        + ("--it; " if method.endswith("end") else "")
        + access
        + "; }"
    )
    changed = ITERATOR_API.replace("void hit() const;", "void hit() const = delete;")
    before = _scan_receiver_api(extractor, tmp_path, ITERATOR_API, source)
    hit = _entries(before, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"]
    assert hit["unresolved_count"] == 0
    assert "operator->" not in _entries(before, "members")
    assert _scan_receiver_api(extractor, tmp_path, changed, source) != before


def test_conditional_foreign_and_unknown_controls(extractor, tmp_path):
    header = CONDITIONAL_API
    source = """struct Foreign { void hit(); }; Foreign one(); Foreign two();
void use(dart::A& known, bool flag) { auto x = flag ? one() : two(); x.hit(); }
template<class T> void unknown(dart::A& known, bool flag, T& a, T& b) { auto x = flag ? a : b; x.hit(); }
"""
    hit = _entries(
        _scan_receiver_api(extractor, tmp_path, header, source), "members"
    ).get("hit")
    assert hit is None or hit["receiver_types"] == []
    assert hit is None or hit["count"] == hit["unresolved_count"]


def test_foreign_container_iterators_do_not_supply_dart_receiver(extractor, tmp_path):
    source = """struct Foreign { void hit() const; }; struct Factory { std::vector<Foreign> values(); std::array<Foreign, 2> array(); };
void use(dart::A& known, Factory& factory) { auto it = factory.values().begin(); it->hit(); auto a = factory.array().cbegin(); (*a).hit(); }
"""
    assert "hit" not in _entries(
        _scan_receiver_api(extractor, tmp_path, ITERATOR_API, source), "members"
    )


def test_dereferenced_iterator_uses_element_own_arrow_operator(extractor, tmp_path):
    source = "void use(dart::Factory& factory) { auto it = factory.values().begin(); (*it)->other(); }"
    changed = ITERATOR_API.replace("void other();", "void other() = delete;")
    before = _scan_receiver_api(extractor, tmp_path, ITERATOR_API, source)
    members = _entries(before, "members")
    assert members["operator->"]["receiver_types"] == ["dart::A"]
    assert members["operator->"]["count"] == 1
    assert members["other"]["receiver_types"] == ["dart::Target"]
    assert _scan_receiver_api(extractor, tmp_path, changed, source) != before


def test_pointer_element_iterator_arrow_does_not_reach_pointee(extractor, tmp_path):
    source = "void use(dart::Factory& factory) { auto it = factory.pointers().begin(); it->hit(); }"
    assert "hit" not in _entries(
        _scan_receiver_api(extractor, tmp_path, ITERATOR_API, source), "members"
    )


def test_unknown_iterator_does_not_acquire_outer_dart_binding(extractor, tmp_path):
    source = """template<class T> struct Factory { std::vector<T> values(); };
template<class T> void use(dart::A* known, Factory<T>& factory) { auto it = known; { auto it = factory.values().begin(); it->hit(); } }
"""
    hit = _entries(
        _scan_receiver_api(extractor, tmp_path, ITERATOR_API, source), "members"
    ).get("hit")
    assert hit is None or hit["receiver_types"] == []
    assert hit is None or hit["count"] == hit["unresolved_count"]


@pytest.mark.parametrize(
    ("case", "header", "changed", "source", "changes_inventory"),
    [
        (
            "std::as_const",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            " void use(dart::Factory& f) { auto value=f.make(); std::as_const(value).hit(); }",
            True,
        ),
        (
            "::std::as_const",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            " void use(dart::Factory& f) { auto value=f.make(); ::std::as_const(value).hit(); }",
            True,
        ),
        (
            "utility::as_const",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace utility = std; void use(dart::Factory& f) { auto value=f.make(); utility::as_const(value).hit(); }",
            True,
        ),
        (
            "as_const",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "using std::as_const; void use(dart::Factory& f) { auto value=f.make(); as_const(value).hit(); }",
            True,
        ),
        (
            "decltype(auto)f.make()",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f) { decltype(auto) value=f.make(); value.hit(); }",
            True,
        ),
        (
            "decltype(auto)f.ref()",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f) { decltype(auto) value=f.ref(); value.hit(); }",
            True,
        ),
        (
            "decltype(auto)f.pointer()",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f) { decltype(auto) value=f.pointer(); value->hit(); }",
            True,
        ),
        (
            "decltype(auto)std::move(f.make())",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f) { decltype(auto) value=std::move(f.make()); value.hit(); }",
            True,
        ),
        (
            "int values[kSize];",
            "namespace dart { constexpr int kSize=2; constexpr int kOther=kSize+1; struct A { int values[kSize]; }; }",
            "namespace dart { constexpr int kSize=3; constexpr int kOther=kSize+1; struct A { int values[kSize]; }; }",
            "static_assert(sizeof(dart::A::values)==sizeof(int)*2); void use(dart::A& a) { (void)a.values; }",
            True,
        ),
        (
            "int values[kSize][kOther];",
            "namespace dart { constexpr int kSize=2; constexpr int kOther=kSize+1; struct A { int values[kSize][kOther]; }; }",
            "namespace dart { constexpr int kSize=3; constexpr int kOther=kSize+1; struct A { int values[kSize][kOther]; }; }",
            "static_assert(sizeof(dart::A::values)==sizeof(int)*6); void use(dart::A& a) { (void)a.values; }",
            True,
        ),
        (
            "int first[2], values[kSize];",
            "namespace dart { constexpr int kSize=2; constexpr int kOther=kSize+1; struct A { int first[2], values[kSize]; }; }",
            "namespace dart { constexpr int kSize=3; constexpr int kOther=kSize+1; struct A { int first[2], values[kSize]; }; }",
            "static_assert(sizeof(dart::A::values)==sizeof(int)*2); void use(dart::A& a) { (void)a.values; }",
            True,
        ),
        (
            "static int values[kSize];",
            "namespace dart { constexpr int kSize=2; constexpr int kOther=kSize+1; struct A { static int values[kSize]; }; }",
            "namespace dart { constexpr int kSize=3; constexpr int kOther=kSize+1; struct A { static int values[kSize]; }; }",
            "static_assert(sizeof(dart::A::values)==sizeof(int)*2); void use(dart::A& a) { (void)a.values; }",
            True,
        ),
        (
            "void hit(int (&values)[kSize]);",
            "namespace dart { constexpr int kSize=2; constexpr int kOther=kSize+1; struct A { void hit(int (&values)[kSize]); }; }",
            "namespace dart { constexpr int kSize=3; constexpr int kOther=kSize+1; struct A { void hit(int (&values)[kSize]); }; }",
            "void use(dart::A& a) { int values[2]{}; a.hit(values); }",
            True,
        ),
        (
            "namespace array",
            "namespace dart { constexpr int kSize=2; int values[kSize]; }",
            "namespace dart { constexpr int kSize=3; int values[kSize]; }",
            "static_assert(sizeof(dart::values)==sizeof(int)*2); void use() { (void)dart::values; }",
            True,
        ),
        (
            "as_const foreign",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "struct Foreign { void hit() const; }; void use(dart::A* known, Foreign& value) { std::as_const(value).hit(); }",
            False,
        ),
        (
            "decltype shadow",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "struct Foreign { void hit(); }; Foreign foreign(); void use(dart::A* known) { { decltype(auto) known=foreign(); known.hit(); } }",
            False,
        ),
        (
            "attribute control",
            "namespace dart { constexpr int maybe_unused=2; struct A { [[maybe_unused]] int values[2]; }; }",
            "namespace dart { constexpr int maybe_unused=3; struct A { [[maybe_unused]] int values[2]; }; }",
            "void use(dart::A& a) { (void)a.values; }",
            False,
        ),
        (
            "unused array control",
            "namespace dart { constexpr int kSize=2; struct A { int used; int ignored[kSize]; }; }",
            "namespace dart { constexpr int kSize=3; struct A { int used; int ignored[kSize]; }; }",
            "void use(dart::A& a) { (void)a.used; }",
            False,
        ),
    ],
)
def test_cpp17_receiver_and_array_extent_mutations(
    extractor, tmp_path, case, header, changed, source, changes_inventory
):
    before = _scan_receiver_api(extractor, tmp_path, header, source)
    after = _scan_receiver_api(extractor, tmp_path, changed, source)
    assert (after != before) == changes_inventory, case


@pytest.mark.parametrize(
    ("case", "header", "changed", "source", "changes_inventory"),
    [
        (
            "decltype parens",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f) { decltype(auto) value(f.make()); value.hit(); }",
            True,
        ),
        (
            "decltype braces",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f) { decltype(auto) value{f.make()}; value.hit(); }",
            True,
        ),
        (
            "decltype grouped",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f) { decltype(auto) first=f.make(), second=f.make(); first.hit(); second.hit(); }",
            True,
        ),
        (
            "decltype if",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f, bool flag) { if (decltype(auto) value=f.make(); flag) value.hit(); }",
            True,
        ),
        (
            "decltype for",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f) { for (decltype(auto) value=f.make(); false;) value.hit(); }",
            True,
        ),
        (
            "as_const nested",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f) { auto value=f.make(); std::move(std::as_const(value)).hit(); }",
            True,
        ),
        (
            "as_const deref",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "void use(dart::Factory& f) { auto value=f.pointer(); std::as_const(*value).hit(); }",
            True,
        ),
        (
            "as_const imported directive",
            "namespace dart { struct A { void hit() const; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "namespace dart { struct A { void hit() const = delete; }; struct Factory { A make(); A& ref(); A* pointer(); }; }",
            "using namespace std; void use(dart::Factory& f) { auto value=f.make(); as_const(value).hit(); }",
            True,
        ),
    ],
)
def test_cpp17_automatic_declaration_and_wrapper_forms(
    extractor, tmp_path, case, header, changed, source, changes_inventory
):
    before = _scan_receiver_api(extractor, tmp_path, header, source)
    after = _scan_receiver_api(extractor, tmp_path, changed, source)
    assert (after != before) == changes_inventory, case


def test_free_function_array_parameter_keeps_namespace_extent_constant(
    extractor, tmp_path
):
    case, header, changed, source, changes_inventory = (
        "free array parameter",
        "namespace dart { constexpr int kSize=2; void hit(int (&values)[kSize]); }",
        "namespace dart { constexpr int kSize=3; void hit(int (&values)[kSize]); }",
        "void use() { int values[2]{}; dart::hit(values); }",
        True,
    )
    before = _scan_receiver_api(extractor, tmp_path, header, source)
    after = _scan_receiver_api(extractor, tmp_path, changed, source)
    assert after != before
    declaration = _entries(before, "free_functions")["dart::hit"]["declarations"][0]
    assert declaration["array_bound_declarations"][0]["class"] == "dart"


LAMBDA_CAPTURE_API = """namespace dart {
struct A { void hit() const {} };
struct B { void other() const {} };
struct Factory { A make(); A& ref(); A* pointer(); B other(); };
}
struct Foreign { void hit() const {} };
Foreign foreign();
template<class T> T unknown();
"""

LAMBDA_CAPTURE_CASES = [
    (
        "void use(dart::Factory& factory) { auto outer = factory.make(); auto fn = [item = outer] { auto child = item; child.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = factory.make()] { auto child = item; child.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory, Foreign item) { auto first = [item = factory.make()] { item.hit(); }; first(); auto second = [other = item] { other.hit(); }; second(); item.hit(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = factory.make()] { item.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [&item = factory.ref()] { item.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = factory.pointer()] { item->hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item{factory.make()}] { item.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item(factory.make())] { item.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = factory.make(), other = factory.other()] { item.hit(); other.other(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [&, item = factory.make()] { item.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [=, item = factory.make()] { item.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = factory.make()](int count) { item.hit(); }; fn(1); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = factory.make()](auto count) { item.hit(); }; fn(1); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = factory.make()]() mutable noexcept(true) -> void { item.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = (factory.make(), factory.make())] { item.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto item = factory.make(); auto fn = [item = item] { item.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = factory.make()] { auto inner = [child = item] { child.hit(); }; inner(); item.hit(); }; fn(); }",
        2,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = factory.make()] { auto inner = [item] { item.hit(); }; inner(); item.hit(); }; fn(); }",
        2,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [item = factory.make()] { auto inner = [item = foreign()] { item.hit(); }; inner(); item.hit(); }; fn(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory, Foreign item) { auto first = [item = factory.make()] { item.hit(); }; first(); item.hit(); auto second = [item = foreign()] { item.hit(); }; second(); item.hit(); }",
        1,
    ),
    (
        "void use(dart::Factory& factory) { auto fn = [callback = [item = factory.make()] { item.hit(); }] { callback(); }; fn(); }",
        1,
    ),
]


@pytest.mark.parametrize("source, count", LAMBDA_CAPTURE_CASES)
def test_lambda_init_captures_retain_body_receiver_dependencies(
    extractor, tmp_path, source, count
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(LAMBDA_CAPTURE_API)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "members")["hit"]
    assert row["count"] == count
    assert row["receiver_types"] == ["dart::A"]
    assert row["unresolved_count"] == 0
    header.write_text(
        LAMBDA_CAPTURE_API.replace(
            "void hit() const {}", "void hit() const = delete;", 1
        )
    )
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after != before
    if "other.other()" in source:
        assert _entries(before, "members")["other"]["receiver_types"] == [
            "dart::B",
            "dart::Factory",
        ]


LAMBDA_CAPTURE_NEGATIVES = [
    "void use(dart::Factory& factory) { auto fn = [item = foreign()] { item.hit(); }; fn(); }",
    "void use(dart::A* item) { auto fn = [item = foreign()] { item.hit(); }; fn(); }",
    "template<class T> void use(dart::A* item) { auto fn = [item = unknown<T>()] { item.hit(); }; }",
    "void use(dart::Factory& factory, Foreign item) { auto fn = [item = foreign()] { item.hit(); }; fn(); item.hit(); }",
]


@pytest.mark.parametrize("source", LAMBDA_CAPTURE_NEGATIVES)
def test_lambda_init_capture_foreign_and_unknown_shadows_do_not_resolve_outer_dart(
    extractor, tmp_path, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(LAMBDA_CAPTURE_API)
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    row = _entries(result, "members").get("hit")
    assert row is None or row["receiver_types"] == []
    assert row is None or row["count"] == row["unresolved_count"]


SMART_POINTER_HEADER = """
namespace dart {
struct Item { void hit() const; };
using Shared = std::shared_ptr<Item>;
using Unique = std::unique_ptr<Item>;
using Weak = std::weak_ptr<Item>;
using SharedAlias = Shared;
struct Factory {
 std::shared_ptr<Item> shared(); std::weak_ptr<Item> weak(); std::unique_ptr<Item> unique();
 Shared alias(); Weak weakAlias(); SharedAlias aliasChain();
 std::shared_ptr<Item>* sharedPointer(); std::weak_ptr<Item>* weakPointer();
 std::optional<std::shared_ptr<Item>> optional();
 std::vector<std::shared_ptr<Item>> sequence();
};
std::shared_ptr<Item> shared(); std::weak_ptr<Item> weak();
}
"""


@pytest.mark.parametrize(
    "expression",
    [
        "f.shared().get()->hit()",
        "f.shared()->hit()",
        "(*f.shared()).hit()",
        "f.weak().lock()->hit()",
        "f.weak().lock().get()->hit()",
        "f.unique().get()->hit()",
        "f.unique()->hit()",
        "(*f.unique()).hit()",
        "f.alias().get()->hit()",
        "f.weakAlias().lock()->hit()",
        "f.aliasChain().get()->hit()",
        "f.sharedPointer()->get()->hit()",
        "f.weakPointer()->lock()->hit()",
        "(*f.sharedPointer()).get()->hit()",
        "(*f.weakPointer()).lock().get()->hit()",
        "f.optional().value().get()->hit()",
        "f.optional()->get()->hit()",
        "f.sequence().front().get()->hit()",
        "f.sequence().begin()->get()->hit()",
        "(*f.sequence().begin())->hit()",
        "dart::shared().get()->hit()",
        "dart::weak().lock()->hit()",
    ],
)
def test_smart_pointer_returns_preserve_wrapper_kind(extractor, tmp_path, expression):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(SMART_POINTER_HEADER)
    before = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::Factory& f) { " + expression + "; }",
    )
    hit = _entries(before, "members")["hit"]
    assert hit["receiver_types"] == ["dart::Item"] and hit["unresolved_count"] == 0
    assert "get" not in _entries(before, "members") and "lock" not in _entries(
        before, "members"
    )
    header.write_text(
        SMART_POINTER_HEADER.replace("void hit() const;", "void hit(int) const;")
    )
    after = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::Factory& f) { " + expression + "; }",
    )
    assert _entries(after, "members")["hit"]["declarations"] != hit["declarations"]


@pytest.mark.parametrize(
    "imported",
    [
        "using namespace std;",
        "using std::shared_ptr; using std::weak_ptr;",
        "namespace s=std; using s::shared_ptr; using s::weak_ptr;",
    ],
)
def test_smart_pointer_source_aliases_bindings_fields_and_imports(
    extractor, tmp_path, imported
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(SMART_POINTER_HEADER)
    source = (
        imported
        + """
using Shared=shared_ptr<dart::Item>; using Alias=Shared;
struct Source { shared_ptr<dart::Item> shared(); weak_ptr<dart::Item> weak(); Shared alias();
 shared_ptr<dart::Item> field; weak_ptr<dart::Item> weakField; };
void use(Source& f) {
 f.shared().get()->hit(); f.weak().lock()->hit(); f.alias().get()->hit();
 f.field.get()->hit(); f.weakField.lock()->hit();
 shared_ptr<dart::Item> direct; direct.get()->hit(); Alias aliased; aliased.get()->hit();
 auto automatic=f.shared(); automatic.get()->hit();
}
"""
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert (
        hit["count"] == 8
        and hit["unresolved_count"] == 0
        and hit["receiver_types"] == ["dart::Item"]
    )


@pytest.mark.parametrize(
    "expression", ["f.weak()->hit()", "f.weak().get()->hit()", "(*f.weak()).hit()"]
)
def test_weak_pointer_does_not_act_as_pointee_receiver(extractor, tmp_path, expression):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(SMART_POINTER_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::Factory& f, dart::Item* known) { " + expression + "; }",
    )
    assert "hit" not in _entries(result, "members")


@pytest.mark.parametrize("kind", ["shared_ptr", "weak_ptr", "unique_ptr"])
def test_foreign_smart_pointer_returns_remain_non_dart(extractor, tmp_path, kind):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(SMART_POINTER_HEADER)
    chain = "lock().get()" if kind == "weak_ptr" else "get()"
    source = (
        "struct Foreign { void hit() const; }; std::"
        + kind
        + "<Foreign> make(); void use(dart::Item* known) { make()."
        + chain
        + "->hit(); }"
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "hit" not in _entries(result, "members")


SMART_ARRAY_HEADER = """
namespace dart {
struct Item { void hit() const; Item& operator[](int); Item* operator->(); };
using SharedArray=std::shared_ptr<Item[]>; using ArrayAlias=SharedArray;
struct Factory {
 std::shared_ptr<Item[]> shared(); std::unique_ptr<Item[]> unique(); std::weak_ptr<Item[]> weak();
 std::optional<std::shared_ptr<Item[]>> optional(); std::vector<std::shared_ptr<Item[]>> sequence();
 std::shared_ptr<Item[]>* sharedPointer(); std::shared_ptr<Item*[]> pointers(); ArrayAlias alias();
};
}
"""


@pytest.mark.parametrize(
    "expression",
    [
        "f.shared()[0].hit()",
        "f.unique()[0].hit()",
        "f.weak().lock()[0].hit()",
        "[&]{ auto p=f.shared(); p[0].hit(); }()",
        "[p=f.shared()] { p[0].hit(); }()",
        "f.optional().value()[0].hit()",
        "f.sequence()[0][0].hit()",
        "f.shared().get()[0].hit()",
        "f.unique().get()[0].hit()",
        "f.weak().lock().get()[0].hit()",
        "f.sharedPointer()[0][0].hit()",
        "f.sharedPointer()[0].get()[0].hit()",
        "f.pointers()[0]->hit()",
        "f.alias()[0].hit()",
        "f.sequence().begin()->get()[0].hit()",
        "(*f.sequence().begin())[0].hit()",
    ],
)
def test_smart_array_receivers_preserve_pointee_and_builtin_operators(
    extractor, tmp_path, expression
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(SMART_ARRAY_HEADER)
    source = "void use(dart::Factory& f) { " + expression + "; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    members = _entries(before, "members")
    hit = members["hit"]
    assert hit["receiver_types"] == ["dart::Item"] and hit["unresolved_count"] == 0
    assert "operator[]" not in members and "operator->" not in members
    header.write_text(
        SMART_ARRAY_HEADER.replace("void hit() const;", "void hit(int) const;")
    )
    after = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(after, "members")["hit"]["declarations"] != hit["declarations"]


@pytest.mark.parametrize(
    "expression", ["f.shared()[0].hit()", "f.unique()[0].hit()", "f.weak()[0].hit()"]
)
def test_nonarray_smart_subscript_does_not_become_pointee(
    extractor, tmp_path, expression
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(SMART_POINTER_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::Factory& f, dart::Item* known) { " + expression + "; }",
    )
    assert "hit" not in _entries(result, "members")


@pytest.mark.parametrize(
    "expression",
    [
        "f.nested().get()->get()->hit()",
        "f.optional().get()->value().hit()",
        "f.sequence().get()->front()->hit()",
    ],
)
def test_nested_smart_pointee_shapes_remain_wrapped(extractor, tmp_path, expression):
    header = """namespace dart { struct Item { void hit() const; }; struct Factory {
 std::shared_ptr<std::shared_ptr<Item>> nested(); std::shared_ptr<std::optional<Item>> optional();
 std::shared_ptr<std::vector<std::shared_ptr<Item>>> sequence(); }; }"""
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(header)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::Factory& f) { " + expression + "; }",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::Item"] and hit["unresolved_count"] == 0


@pytest.mark.parametrize(
    "source, dart_receiver",
    [
        (
            "struct Foreign { void hit() const; }; Foreign foreign(); void use(dart::A* known) { auto fn=[a=foreign()] { a.hit(); }; }",
            False,
        ),
        (
            "struct Foreign { void hit() const; }; Foreign foreign(); void use(dart::A* known) { auto fn=[a=foreign()] { auto x=a; auto inner=[b=x] { auto y=b; auto last=[c=y] { auto z=c; z.hit(); }; }; }; }",
            False,
        ),
        (
            "void use(dart::A& known) { auto fn=[a=known] { auto x=a; auto inner=[b=x] { auto y=b; auto last=[c=y] { auto z=c; z.hit(); }; }; }; }",
            True,
        ),
    ],
)
def test_nested_capture_local_auto_composition_preserves_known_receiver(
    extractor, tmp_path, source, dart_receiver
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { void hit() const; }; }"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "members").get("hit")
    if dart_receiver:
        assert row["receiver_types"] == ["dart::A"]
        assert row["unresolved_count"] == 0
    else:
        assert row is None
    header.write_text(text.replace("void hit() const;", "void hit(int) const;"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (after != before) is dart_receiver


@pytest.mark.parametrize(
    "comparison",
    [
        "flag < 2 ? f.make() : f.make()",
        "flag > 2 ? f.make() : f.make()",
        "(flag < 2 ? f.make() : f.make())",
    ],
)
def test_capture_comparison_does_not_merge_following_capture(
    extractor, tmp_path, comparison
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { void hit() const; }; struct B { void check() const; }; struct Factory { A make(); B other(); }; }"
    header.write_text(text)
    source = (
        "void use(dart::Factory& f, int flag) { auto fn=[item="
        + comparison
        + ", other=f.other()] { item.hit(); other.check(); }; }"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    rows = _entries(before, "members")
    assert rows["hit"]["receiver_types"] == ["dart::A"]
    assert rows["check"]["receiver_types"] == ["dart::B"]
    assert rows["check"]["unresolved_count"] == 0
    header.write_text(text.replace("void check() const;", "void check(int) const;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


@pytest.mark.parametrize(
    "kind, source",
    [
        ("shared_ptr", "void use(dart::Factory& factory) { inspect(factory.make()); }"),
        ("shared_ptr", "void use(std::shared_ptr<dart::A> value) { inspect(value); }"),
        ("weak_ptr", "void use(dart::Factory& factory) { inspect(factory.make()); }"),
        ("unique_ptr", "void use(dart::Factory& factory) { inspect(factory.make()); }"),
        (
            "shared_ptr",
            "void use(dart::Factory& factory) { auto lambda=[value=factory.make()] { inspect(value); }; }",
        ),
    ],
)
def test_smart_receiver_kinds_preserve_argument_associated_namespace(
    extractor, tmp_path, kind, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "namespace dart { struct A { friend void inspect(std::"
        + kind
        + "<A>) {} }; struct Factory { std::"
        + kind
        + "<A> make(); }; }"
    )
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert _entries(before, "free_functions")["dart::inspect"]["count"] == 1
    header.write_text(text.replace("<A>) {}", "<A>, int) {}"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after != before


@pytest.mark.parametrize(
    "definition",
    [
        "constexpr int N=2; inline constexpr int Names[N]{}; struct A { void run(const int (&values)[2]=Names); };",
        "struct A { static constexpr int BASE=1; static constexpr int N=BASE+1; inline static constexpr int Names[N]{}; public: void run(const int (&values)[2]=Names); };",
    ],
)
def test_used_default_array_constant_retains_bound_dependency(
    extractor, tmp_path, definition
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { " + definition + " }"
    header.write_text(text)
    source = "void use(dart::A& a) { a.run(); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "members")["run"]["declarations"][0]
    defaults = row["default_argument_declarations"]
    assert any(
        " N " in d["signature"]
        or d["signature"].startswith("constexpr int N ")
        or d["signature"].startswith("static constexpr int N ")
        for d in defaults
    )
    assert any(
        d["signature"].endswith("N = 2") or "N = BASE + 1" in d["signature"]
        for d in defaults
    )
    header.write_text(text.replace("N=2", "N=3").replace("BASE=1", "BASE=2"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after != before


def test_foreign_smart_arguments_do_not_acquire_unrelated_dart_namespace(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(
        "namespace dart { struct A { friend void inspect(std::shared_ptr<A>) {} }; }"
    )
    source = "namespace foreign { struct B { friend void inspect(std::shared_ptr<B>) {} }; } void use(dart::A* known, std::shared_ptr<foreign::B> value) { inspect(value); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert not before["free_functions"]
    header.write_text(header.read_text().replace("<A>) {}", "<A>, int) {}"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        == before
    )


SMART_ADL_CASES = [
    (
        "shared_auto",
        "std::shared_ptr<A>",
        "void use(dart::Factory& f) { auto value=f.make(); inspect(value); }",
    ),
    (
        "api_alias_variable",
        "std::shared_ptr<A>",
        "void use(dart::Alias value) { inspect(value); }",
    ),
    (
        "source_alias_variable",
        "std::shared_ptr<A>",
        "using Shared=std::shared_ptr<dart::A>; using Alias=Shared; void use(Alias value) { inspect(value); }",
    ),
    (
        "source_return",
        "std::shared_ptr<A>",
        "std::shared_ptr<dart::A> make(); void use() { inspect(make()); }",
    ),
    (
        "weak_variable",
        "std::weak_ptr<A>",
        "void use(std::weak_ptr<dart::A> value) { inspect(value); }",
    ),
    (
        "unique_variable",
        "std::unique_ptr<A>",
        "void use(std::unique_ptr<dart::A> value) { inspect(std::move(value)); }",
    ),
    (
        "array_return",
        "std::shared_ptr<A[]>",
        "void use(dart::Factory& f) { inspect(f.make()); }",
    ),
    (
        "nested_smart_return",
        "std::shared_ptr<std::shared_ptr<A>>",
        "void use(dart::Factory& f) { inspect(f.make()); }",
    ),
    (
        "optional_smart_return",
        "std::optional<std::shared_ptr<A>>",
        "void use(dart::Factory& f) { inspect(f.make()); }",
    ),
    (
        "sequence_smart_return",
        "std::vector<std::shared_ptr<A>>",
        "void use(dart::Factory& f) { inspect(f.make()); }",
    ),
    (
        "smart_optional_return",
        "std::shared_ptr<std::optional<A>>",
        "void use(dart::Factory& f) { inspect(f.make()); }",
    ),
    (
        "source_smart_field",
        "std::shared_ptr<A>",
        "struct Source { std::shared_ptr<dart::A> field; }; void use(Source& f) { inspect(f.field); }",
    ),
]


def _smart_adl_header(wrapped):
    return (
        "namespace dart { struct A; using Wrapped="
        + wrapped
        + "; using Alias=Wrapped; struct A { friend void inspect(Wrapped); }; struct Factory { Wrapped make(); }; }"
    )


@pytest.mark.parametrize("name,wrapped,source", SMART_ADL_CASES)
def test_smart_wrapped_arguments_keep_hidden_friend_adl(
    extractor, tmp_path, name, wrapped, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = _smart_adl_header(wrapped)
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    friend = _entries(before, "free_functions")["dart::inspect"]
    assert friend["declarations"]
    header.write_text(text.replace("inspect(Wrapped);", "inspect(Wrapped, int);"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert (
        _entries(after, "free_functions")["dart::inspect"]["declarations"]
        != friend["declarations"]
    )


@pytest.mark.parametrize("kind", ["shared_ptr", "weak_ptr", "unique_ptr"])
def test_foreign_smart_arguments_do_not_gain_dart_adl(extractor, tmp_path, kind):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = _smart_adl_header("std::shared_ptr<A>")
    header.write_text(text)
    source = (
        "struct Foreign { friend void inspect(std::"
        + kind
        + "<Foreign>); }; std::"
        + kind
        + "<Foreign> make(); void use(dart::A* known) { inspect(make()); }"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "dart::inspect" not in _entries(before, "free_functions")
    header.write_text(text.replace("inspect(Wrapped);", "inspect(Wrapped, int);"))
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) == before


def test_smart_argument_member_lookup_still_suppresses_adl(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = _smart_adl_header("std::shared_ptr<A>")
    header.write_text(text)
    source = "struct Source { void inspect(std::shared_ptr<dart::A>); void use(std::shared_ptr<dart::A> value) { inspect(value); } };"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "dart::inspect" not in _entries(result, "free_functions")


@pytest.mark.parametrize(
    "source",
    [
        "void use(dart::A* known) { std::unique_ptr<aiMesh> mesh; mesh->mFaces[0].mIndices[0]=1; }",
        "void use(dart::A* known) { std::shared_ptr<aiMesh> mesh; mesh->mFaces[0].mIndices[0]=1; }",
        "void use(dart::A* known) { std::weak_ptr<aiMesh> mesh; mesh.lock()->mFaces[0].mIndices[0]=1; }",
        "void use(dart::A* known) { std::unique_ptr<aiMesh> mesh=std::make_unique<aiMesh>(); mesh->mFaces[0].mIndices[0]=1; }",
        "void use(dart::A* known, aiMesh* mesh) { mesh->mFaces[0].mIndices[0]=1; }",
    ],
)
def test_external_assimp_array_fields_do_not_become_dart_fields(
    extractor, tmp_path, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    # External non-DART definitions are available to the compiler but are not
    # collected by index_api or present in the scanned Gazebo source files.
    text = "#include <memory>\nstruct aiFace { unsigned* mIndices; }; struct aiMesh { aiFace* mFaces; }; namespace dart { struct A { int mIndices; }; }"
    header.write_text(text)
    api = extractor.index_api(headers)
    assert api.types == {"dart::A"}
    before = _scan(extractor, api, tmp_path / "before", source)
    assert not before["data_members"]
    header.write_text(text.replace("int mIndices;", "double mIndices;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        == before
    )


@pytest.mark.parametrize(
    "name", ["dDerived", "aiDerived", "btDerived", "SourceDerived"]
)
def test_known_source_derived_returns_override_external_prefix_heuristic(
    extractor, tmp_path, name
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { void hit() const; }; }"
    source = (
        "struct "
        + name
        + ":dart::A {}; "
        + name
        + " make(); void use() { make().hit(); }"
    )
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    hit = _entries(before, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"]
    assert hit["unresolved_count"] == 0
    header.write_text(text.replace("void hit() const;", "void hit() const = delete;"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


def test_smart_field_union_retains_custom_pointer_candidates(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { void hit(); }; template<class T> struct Ptr { T* operator->(); }; using APtr=Ptr<A>; }"
    header.write_text(text)
    source = "struct Info { dart::APtr item; }; struct Foreign {}; struct Other { std::shared_ptr<Foreign> item; }; template<class T> T* factory(); void use() { factory<Info>()->item->hit(); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "members")["hit"]
    assert row["receiver_types"] == ["dart::APtr"]
    assert row["unresolved_count"] == 0
    header.write_text(text.replace("void hit();", "void hit(int);"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


@pytest.mark.parametrize(
    "source, count",
    [
        ("dart::A make() { return {1}; }", 1),
        ("dart::A make(bool flag) { if (flag) return {1}; else return {2}; }", 2),
        (
            "dart::A make(int flag) { switch(flag) { case 1: return {1}; default: return {2}; } }",
            2,
        ),
        ("dart::A make() { try { return {1}; } catch (...) { return {2}; } }", 2),
        (
            "namespace consumer { using Result=dart::A; Result make() { return {1}; } }",
            1,
        ),
        ("namespace physics=dart; physics::A make() { return {1}; }", 1),
        ("struct Maker { dart::A make() { return {1}; } };", 1),
        ("struct Maker { dart::A make(); }; dart::A Maker::make() { return {1}; }", 1),
        ("struct Maker { using Result=dart::A; Result make() { return {1}; } };", 1),
        (
            "namespace consumer { struct Maker { using Result=dart::A; Result make(); }; } consumer::Maker::Result consumer::Maker::make() { return {1}; }",
            1,
        ),
        ("const dart::A& make() { return {1}; }", 1),
        ("dart::A make() { auto lambda=[] { return 1; }; return {lambda()}; }", 1),
        (
            "dart::A make() { auto lambda=[]() noexcept -> int { return {1}; }; return {lambda()}; }",
            1,
        ),
        (
            "dart::A make() { struct Local { int get() { return {1}; } }; return {1}; }",
            1,
        ),
        ("dart::A make();", 0),
        ("dart::A* make() { return {}; }", 0),
        ("using Pointer=dart::A*; Pointer make() { return {}; }", 0),
        ("dart::A make() { return dart::A(1); }", 1),
        ("void use() { dart::A* known; auto lambda=[]() -> int { return {1}; }; }", 0),
    ],
)
def test_contextual_braced_returns_retain_only_their_construction(
    extractor, tmp_path, source, count
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { A(int); }; }"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert _entries(before, "constructors").get("dart::A", {}).get("count", 0) == count
    header.write_text(text.replace("A(int);", "A();"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (after != before) == bool(count)


def test_contextual_braced_aggregate_return_preserves_ordered_fields(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { int first; double second; }; }"
    source = "dart::A make() { return {1, 2.0}; }"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "constructors")["dart::A"]
    assert [d["name"] for d in row["brace_member_declarations"]] == ["first", "second"]
    header.write_text(
        text.replace("int first; double second;", "double second; int first;")
    )
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after != before


def test_contextual_braced_return_reuses_owned_constructor_dependencies(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct Base { Base(int); }; struct A:Base { A(int); }; }"
    source = "dart::A make() { return {1}; }"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "constructors")["dart::A"]
    assert {d["class"] for d in row["declarations"]} == {"dart::A", "dart::Base"}
    header.write_text(text.replace("Base(int);", "Base();"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert after != before


OWNED_CTOR_BEFORE = "namespace dart { struct A { A() = default; }; }"
OWNED_CTOR_AFTER = OWNED_CTOR_BEFORE.replace("A() = default", "A() = delete")
OWNED_TUPLE_CASES = [
    (
        "tuple",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<dart::A> value;",
        True,
        "dart::A",
    ),
    (
        "pair_first",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::pair<dart::A,int> value;",
        True,
        "dart::A",
    ),
    (
        "pair_second",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::pair<int,dart::A> value;",
        True,
        "dart::A",
    ),
    (
        "tuple_cv",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<const volatile dart::A> value;",
        True,
        "dart::A",
    ),
    (
        "tuple_brace",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<int,dart::A> value{};",
        True,
        "dart::A",
    ),
    (
        "pair_brace",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::pair<int,dart::A> value{};",
        True,
        "dart::A",
    ),
    (
        "tuple_pair",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<std::pair<int,dart::A>> value;",
        True,
        "dart::A",
    ),
    (
        "pair_tuple",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::pair<std::tuple<dart::A>,int> value;",
        True,
        "dart::A",
    ),
    (
        "tuple_array",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<std::array<dart::A,1>> value;",
        True,
        "dart::A",
    ),
    (
        "array_tuple",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::array<std::tuple<dart::A>,1> value;",
        True,
        "dart::A",
    ),
    (
        "tuple_multiple",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<int,dart::A*,const dart::A> value;",
        True,
        "dart::A",
    ),
    (
        "tuple_pointer",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<dart::A*> value;",
        False,
        "dart::A",
    ),
    (
        "pair_pointer",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::pair<int,dart::A*> value;",
        False,
        "dart::A",
    ),
    (
        "tuple_reference",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "void use(dart::A& existing) { std::tuple<dart::A&> value(existing); }",
        False,
        "dart::A",
    ),
    (
        "pair_reference",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "void use(dart::A& existing) { std::pair<int,dart::A&> value(1,existing); }",
        False,
        "dart::A",
    ),
    (
        "tuple_rvalue_reference",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "void use(dart::A& existing) { std::tuple<dart::A&&> value(std::move(existing)); }",
        False,
        "dart::A",
    ),
    (
        "tuple_function_pointer",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<dart::A(*)()> value;",
        False,
        "dart::A",
    ),
    (
        "tuple_function_parameter",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<void(*)(int,dart::A)> value;",
        False,
        "dart::A",
    ),
    (
        "pair_function_pointer",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::pair<int,dart::A(*)()> value;",
        False,
        "dart::A",
    ),
    (
        "tuple_vector",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<std::vector<dart::A>> value;",
        False,
        "dart::A",
    ),
    (
        "tuple_shared",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<std::shared_ptr<dart::A>> value;",
        False,
        "dart::A",
    ),
    (
        "tuple_optional",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<std::optional<dart::A>> value;",
        False,
        "dart::A",
    ),
    (
        "nested_tuple_pointer",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "std::tuple<std::tuple<dart::A*>> value;",
        False,
        "dart::A",
    ),
    (
        "tuple_parameter",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "void use(std::tuple<dart::A> value);",
        False,
        "dart::A",
    ),
    (
        "tuple_type_only",
        OWNED_CTOR_BEFORE,
        OWNED_CTOR_AFTER,
        "using Type=std::tuple<dart::A>;",
        False,
        "dart::A",
    ),
]
for kind in ["std::tuple<A>", "std::pair<int,A>", "std::tuple<std::array<A,1>>"]:
    before = (
        OWNED_CTOR_BEFORE.removesuffix("}")
        + " struct Holder { "
        + kind
        + " value; }; }"
    )
    OWNED_TUPLE_CASES.append(
        (
            "field_" + kind,
            before,
            before.replace("A() = default", "A() = delete"),
            "dart::Holder value;",
            True,
            "dart::Holder",
        )
    )


OWNED_DEFAULT_CASES = [
    (
        "field_constant",
        "namespace dart { inline constexpr int K=1; struct A { int x=K; }; }",
        "namespace dart { inline constexpr int K=2; struct A { int x=K; }; }",
        "constexpr dart::A value; static_assert(value.x==1);",
        True,
        "dart::A",
    ),
    (
        "field_brace_constant",
        "namespace dart { inline constexpr int K=1; struct A { int x{K}; }; }",
        "namespace dart { inline constexpr int K=2; struct A { int x{K}; }; }",
        "constexpr dart::A value; static_assert(value.x==1);",
        True,
        "dart::A",
    ),
    (
        "field_literal",
        "namespace dart { struct A { int x=1; }; }",
        "namespace dart { struct A { int x=2; }; }",
        "constexpr dart::A value; static_assert(value.x==1);",
        True,
        "dart::A",
    ),
    (
        "field_local_constant",
        "namespace dart { struct A { static constexpr int K=1; int x=K; }; }",
        "namespace dart { struct A { static constexpr int K=2; int x=K; }; }",
        "constexpr dart::A value; static_assert(value.x==1);",
        True,
        "dart::A",
    ),
    (
        "field_transitive_constant",
        "namespace dart { inline constexpr int BASE=0; inline constexpr int K=BASE+1; struct A { int x=K; }; }",
        "namespace dart { inline constexpr int BASE=1; inline constexpr int K=BASE+1; struct A { int x=K; }; }",
        "constexpr dart::A value; static_assert(value.x==1);",
        True,
        "dart::A",
    ),
    (
        "field_inherited_constant",
        "namespace dart { struct Base { static constexpr int K=1; }; struct A : Base { int x=K; }; }",
        "namespace dart { struct Base { static constexpr int K=2; }; struct A : Base { int x=K; }; }",
        "constexpr dart::A value; static_assert(value.x==1);",
        True,
        "dart::A",
    ),
    (
        "field_owned_constant",
        "namespace dart { inline constexpr int K=1; struct B { int x=K; }; struct A { B value; }; }",
        "namespace dart { inline constexpr int K=2; struct B { int x=K; }; struct A { B value; }; }",
        "constexpr dart::A value; static_assert(value.value.x==1);",
        True,
        "dart::A",
    ),
    (
        "field_array_constant",
        "namespace dart { inline constexpr int K=1; struct A { int x[1]={K}; }; }",
        "namespace dart { inline constexpr int K=2; struct A { int x[1]={K}; }; }",
        "constexpr dart::A value; static_assert(value.x[0]==1);",
        True,
        "dart::A",
    ),
    (
        "field_array_bound",
        "namespace dart { inline constexpr int N=1; struct A { int x[N]{}; }; }",
        "namespace dart { inline constexpr int N=2; struct A { int x[N]{}; }; }",
        "constexpr dart::A value; static_assert(sizeof(value.x)==sizeof(int));",
        True,
        "dart::A",
    ),
    (
        "field_tuple_constant",
        "namespace dart { inline constexpr int K=1; struct B { int x=K; }; struct A { std::tuple<B> value; }; }",
        "namespace dart { inline constexpr int K=2; struct B { int x=K; }; struct A { std::tuple<B> value; }; }",
        "constexpr dart::A value; static_assert(std::get<0>(value.value).x==1);",
        True,
        "dart::A",
    ),
    (
        "field_unused",
        "namespace dart { inline constexpr int K=1; struct A { int x=K; }; }",
        "namespace dart { inline constexpr int K=2; struct A { int x=K; }; }",
        "void use(dart::A* value);",
        False,
        "dart::A",
    ),
    (
        "field_pointer",
        "namespace dart { inline constexpr int K=1; struct A { int x=K; }; }",
        "namespace dart { inline constexpr int K=2; struct A { int x=K; }; }",
        "dart::A* value=nullptr;",
        False,
        "dart::A",
    ),
    (
        "field_static",
        "namespace dart { inline constexpr int K=1; struct A { inline static int x=K; }; }",
        "namespace dart { inline constexpr int K=2; struct A { inline static int x=K; }; }",
        "dart::A value;",
        False,
        "dart::A",
    ),
    (
        "field_shadowed_namespace",
        "namespace dart { inline constexpr int K=3; struct A { static constexpr int K=1; int x=K; }; }",
        "namespace dart { inline constexpr int K=4; struct A { static constexpr int K=1; int x=K; }; }",
        "constexpr dart::A value; static_assert(value.x==1);",
        False,
        "dart::A",
    ),
]


@pytest.mark.parametrize(
    "name,before_text,after_text,source,changed,owner", OWNED_TUPLE_CASES
)
def test_tuple_pair_owned_construction_evidence(
    extractor, tmp_path, name, before_text, after_text, source, changed, owner
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(before_text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    constructors = _entries(before, "constructors")
    if changed:
        assert owner in constructors
        assert any(d["class"] == "dart::A" for d in constructors[owner]["declarations"])
    else:
        assert not constructors
    header.write_text(after_text)
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (before != after) == changed


@pytest.mark.parametrize(
    "name,before_text,after_text,source,changed,owner", OWNED_DEFAULT_CASES
)
def test_constructed_default_field_initializer_evidence(
    extractor, tmp_path, name, before_text, after_text, source, changed, owner
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(before_text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    if changed:
        declarations = _entries(before, "constructors")[owner][
            "default_member_initializer_declarations"
        ]
        assert any(
            d["class"] in {"dart::A", "dart::B"} and "x" in d["signature"].split()
            for d in declarations
        )
        if "K=" in before_text:
            assert any("K" in d["signature"].split() for d in declarations)
        if "BASE=" in before_text:
            assert any("BASE" in d["signature"].split() for d in declarations)
    header.write_text(after_text)
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (before != after) == changed


@pytest.mark.parametrize(
    "source, count",
    [
        ("std::tuple<dart::A> make() { return {}; }", 1),
        ("std::pair<dart::A,int> make() { return {}; }", 1),
        ("std::pair<int,dart::A> make() { return {}; }", 1),
        ("std::tuple<std::pair<int,dart::A>> make() { return {}; }", 1),
        ("std::array<dart::A,1> make() { return {}; }", 1),
        ("std::array<std::tuple<dart::A>,1> make() { return {}; }", 1),
        ("std::tuple<std::array<dart::A,1>> make() { return {}; }", 1),
        ("std::tuple<dart::A*> make() { return {}; }", 0),
        ("std::pair<dart::A*,int> make() { return {}; }", 0),
        ("std::tuple<dart::A&> make(dart::A& a) { return {a}; }", 0),
        ("std::array<dart::A*,1> make() { return {}; }", 0),
        ("std::array<dart::A,0> make() { return {}; }", 0),
        ("std::vector<dart::A> make() { return {}; }", 0),
        ("std::optional<dart::A> make() { return {}; }", 0),
        ("std::shared_ptr<dart::A> make() { return {}; }", 0),
        ("std::tuple<dart::A*> make();", 0),
    ],
)
def test_contextual_braced_container_returns_preserve_element_ownership(
    extractor, tmp_path, source, count
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { A() = default; }; }"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert _entries(before, "constructors").get("dart::A", {}).get("count", 0) == count
    header.write_text(text.replace("A() = default;", "A(int);"))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (after != before) == bool(count)


DEDUCED_HELPER_HEADER = "namespace dart { struct A { void hit() const; }; A makeA(); struct Factory { A make(); std::shared_ptr<A> smart(); }; }"
DEDUCED_HELPER_CASES = [
    (
        "concrete_parameter",
        True,
        {
            "sample.cc": "auto helper(dart::A& value) { return value; } void use(dart::A& value) { helper(value).hit(); }"
        },
    ),
    (
        "smart_parameter",
        True,
        {
            "sample.cc": "decltype(auto) helper(const std::shared_ptr<dart::A>& value) { return (value); } void use(const std::shared_ptr<dart::A>& value) { helper(value).get()->hit(); }"
        },
    ),
    (
        "source_member",
        True,
        {
            "sample.cc": "struct Maker { auto helper(dart::Factory& f) { return f.make(); } }; void use(Maker& m, dart::Factory& f) { m.helper(f).hit(); }"
        },
    ),
    (
        "out_of_line_member",
        True,
        {
            "sample.cc": "struct Maker { auto helper(dart::Factory& f); }; auto Maker::helper(dart::Factory& f) { return f.make(); } void use(Maker& m, dart::Factory& f) { m.helper(f).hit(); }"
        },
    ),
    (
        "recursive_known_first_return",
        True,
        {
            "sample.cc": "auto helper(int n) { if(n==0) return dart::makeA(); return helper(n-1); } void use() { helper(0).hit(); }"
        },
    ),
    (
        "conditional_same_type",
        True,
        {
            "sample.cc": "auto helper(bool flag,dart::Factory& f) { return flag ? f.make() : f.make(); } void use(dart::Factory& f) { helper(true,f).hit(); }"
        },
    ),
    (
        "branch_returns",
        True,
        {
            "sample.cc": "auto helper(bool flag,dart::Factory& f) { if(flag) { return f.make(); } else { return f.make(); } } void use(dart::Factory& f) { helper(true,f).hit(); }"
        },
    ),
    (
        "local_auto_chain",
        True,
        {
            "sample.cc": "auto helper(dart::Factory& f) { auto first=f.make(); auto second=first; return second; } void use(dart::Factory& f) { helper(f).hit(); }"
        },
    ),
    (
        "included_header_parameter_alias",
        True,
        {
            "helper.hh": "namespace provider { using Value=dart::A; inline auto helper(Value& value) { return value; } }",
            "caller.cc": '#include "helper.hh"\nvoid use(dart::A& value) { provider::helper(value).hit(); }',
        },
    ),
    (
        "included_header_local_chain",
        True,
        {
            "helper.hh": "namespace provider { inline auto helper(dart::Factory& f) { auto value=f.make(); return value; } }",
            "caller.cc": '#include "helper.hh"\nvoid use(dart::Factory& f) { auto value=provider::helper(f); value.hit(); }',
        },
    ),
    (
        "lambda_return_excluded",
        False,
        {
            "sample.cc": "struct Foreign { void hit() const; }; Foreign foreign(); auto helper() { auto ignored=[] { return dart::makeA(); }; return foreign(); } void use(dart::A* known) { helper().hit(); }"
        },
    ),
    (
        "local_class_return_excluded",
        False,
        {
            "sample.cc": "struct Foreign { void hit() const; }; Foreign foreign(); auto helper() { struct Local { auto make() { return dart::makeA(); } }; return foreign(); } void use(dart::A* known) { helper().hit(); }"
        },
    ),
    (
        "lambda_ignored_actual_dart",
        True,
        {
            "sample.cc": "auto helper() { auto ignored=[] { return 0; }; return dart::makeA(); } void use() { helper().hit(); }"
        },
    ),
    (
        "inner_namespace_helper_shadow",
        False,
        {
            "helper.hh": "namespace provider { inline auto helper() { return dart::makeA(); } }",
            "caller.cc": '#include "helper.hh"\nstruct Foreign { void hit() const; }; using provider::helper; namespace consumer { Foreign helper(); void use(dart::A* known) { helper().hit(); } }',
        },
    ),
    (
        "unrelated_header_not_visible",
        False,
        {
            "unused.hh": "inline auto helper() { return dart::makeA(); }",
            "caller.cc": "struct Foreign { void hit() const; }; Foreign helper(); void use(dart::A* known) { helper().hit(); }",
        },
    ),
]


@pytest.mark.parametrize("name,positive,files", DEDUCED_HELPER_CASES)
def test_concrete_deduced_helpers_preserve_scope_and_receiver_shapes(
    extractor, tmp_path, name, positive, files
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(DEDUCED_HELPER_HEADER)
    before = _scan_files(
        extractor, extractor.index_api(headers), tmp_path / "before", files
    )
    row = _entries(before, "members").get("hit")
    if positive:
        assert row and row["receiver_types"] == ["dart::A"]
        assert row["unresolved_count"] == 0
    else:
        assert row is None
    header.write_text(
        DEDUCED_HELPER_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    after = _scan_files(
        extractor, extractor.index_api(headers), tmp_path / "after", files
    )
    assert (before != after) == positive


BRACED_ALIAS_CASES = [
    (
        "reference_alias_source",
        "namespace dart {struct A{A(int);};}",
        "using Ref=const dart::A&; Ref make(){return{1};}",
        "A(int);",
        "A();",
        True,
    ),
    (
        "reference_alias_api",
        "namespace dart {struct A{A(int);};using Ref=const A&;}",
        "dart::Ref make(){return{1};}",
        "A(int);",
        "A();",
        True,
    ),
    (
        "tuple_reference_alias_source",
        "namespace dart {struct A{A();};}",
        "using Ref=dart::A&; std::tuple<Ref> make(dart::A&a){return{a};}",
        "A();",
        "A()=delete;",
        False,
    ),
    (
        "tuple_reference_alias_api",
        "namespace dart {struct A{A();};using Ref=A&;}",
        "std::tuple<dart::Ref> make(dart::A&a){return{a};}",
        "A();",
        "A()=delete;",
        False,
    ),
    (
        "tuple_value_alias",
        "namespace dart {struct A{A();};}",
        "using Value=dart::A; std::tuple<Value> make(){return{};}",
        "A();",
        "A()=delete;",
        True,
    ),
    (
        "reference_alias_chain_return",
        "namespace dart {struct A{A(int);};}",
        "using Ref=const dart::A&; using Other=Ref; Other make(){return{1};}",
        "A(int);",
        "A();",
        True,
    ),
    (
        "reference_alias_chain_element",
        "namespace dart {struct A{A();};}",
        "using Ref=dart::A&; using Other=Ref; std::pair<int,Other> make(dart::A&a){return{1,a};}",
        "A();",
        "A()=delete;",
        False,
    ),
    (
        "container_alias_element",
        "namespace dart {struct A{A();};}",
        "using Vector=std::vector<dart::A>; std::tuple<Vector> make(){return{};}",
        "A();",
        "A()=delete;",
        False,
    ),
]


@pytest.mark.parametrize("name,text,source,old,new,changed", BRACED_ALIAS_CASES)
def test_contextual_braced_alias_preserves_element_ownership(
    extractor, tmp_path, name, text, source, old, new, changed
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    constructors = _entries(before, "constructors")
    if changed:
        assert any(
            d["class"] == "dart::A"
            for row in constructors.values()
            for d in row["declarations"]
        )
    else:
        assert not constructors
    header.write_text(text.replace(old, new))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (before != after) == changed


AUTO_HELPER_HEADER = """namespace dart {
struct A { void hit() const; }; struct B { void other() const; };
A makeA(); A& reference(); A* pointer(); std::shared_ptr<A> shared(); std::weak_ptr<A> weak();
std::optional<A> optional(); std::vector<A> sequence(); std::array<A,2> array();
std::optional<std::shared_ptr<A>> nested();
}"""
AUTO_HELPER_CASES = [
    ("auto_free", "auto helper() { return dart::makeA(); }", "helper().hit()"),
    (
        "decltype_auto_free",
        "decltype(auto) helper() { return dart::makeA(); }",
        "helper().hit()",
    ),
    ("auto_pointer", "auto helper() { return dart::pointer(); }", "helper()->hit()"),
    (
        "decltype_reference",
        "decltype(auto) helper() { return dart::reference(); }",
        "helper().hit()",
    ),
    (
        "auto_method",
        "struct Source { auto make() { return dart::makeA(); } };",
        "Source value; value.make().hit()",
    ),
    (
        "decltype_method",
        "struct Source { decltype(auto) make() { return dart::reference(); } };",
        "Source value; value.make().hit()",
    ),
    (
        "out_of_line",
        "struct Source { auto make(); }; auto Source::make() { return dart::makeA(); }",
        "Source value; value.make().hit()",
    ),
    (
        "source_namespace",
        "namespace one { auto make() { return dart::makeA(); } }",
        "one::make().hit()",
    ),
    (
        "source_using",
        "namespace one { auto make() { return dart::makeA(); } } using one::make;",
        "make().hit()",
    ),
    (
        "local_auto",
        "auto helper() { auto value=dart::makeA(); return value; }",
        "helper().hit()",
    ),
    (
        "parameter",
        "auto helper(dart::A& value) { return value; }",
        "dart::A value; helper(value).hit()",
    ),
    ("shared", "auto helper() { return dart::shared(); }", "helper().get()->hit()"),
    ("weak", "auto helper() { return dart::weak(); }", "helper().lock()->hit()"),
    (
        "optional",
        "auto helper() { return dart::optional(); }",
        "helper().value().hit()",
    ),
    (
        "sequence",
        "auto helper() { return dart::sequence(); }",
        "helper().front().hit()",
    ),
    ("array", "auto helper() { return dart::array(); }", "helper()[0].hit()"),
    (
        "nested_wrapper",
        "decltype(auto) helper() { return dart::nested(); }",
        "helper().value().get()->hit()",
    ),
    (
        "forwarding",
        "auto helper() { return std::move(dart::shared()); }",
        "helper().get()->hit()",
    ),
    (
        "helper_chain",
        "auto inner() { return dart::makeA(); } auto helper() { return inner(); }",
        "helper().hit()",
    ),
    (
        "nested_method",
        "struct Source { auto inner() { return dart::makeA(); } auto helper() { return inner(); } };",
        "Source value; value.helper().hit()",
    ),
    (
        "ordinary_block",
        "auto helper(bool choose) { if(choose) { return dart::makeA(); } else { return dart::makeA(); } }",
        "helper(true).hit()",
    ),
    (
        "switch_block",
        "auto helper(int choose) { switch(choose) { case 0:return dart::makeA(); default:return dart::makeA(); } }",
        "helper(0).hit()",
    ),
    (
        "try_block",
        "auto helper() { try { return dart::makeA(); } catch(...) { return dart::makeA(); } }",
        "helper().hit()",
    ),
    (
        "lambda_exclusion",
        "auto helper() { auto nested=[]() noexcept -> dart::B { return {}; }; return dart::makeA(); }",
        "helper().hit()",
    ),
    (
        "local_class_exclusion",
        "auto helper() { struct Nested { dart::B make() { return {}; } }; return dart::makeA(); }",
        "helper().hit()",
    ),
]


@pytest.mark.parametrize("name,prefix,expression", AUTO_HELPER_CASES)
def test_source_auto_helper_return_inference(
    extractor, tmp_path, name, prefix, expression
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(AUTO_HELPER_HEADER)
    source = prefix + " void use() { " + expression + "; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(before, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0
    header.write_text(
        AUTO_HELPER_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    after = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(after, "members")["hit"]["declarations"] != hit["declarations"]


@pytest.mark.parametrize(
    "prefix,expression",
    [
        (
            "struct Foreign { void hit() const; }; Foreign makeForeign(); auto helper() { return makeForeign(); }",
            "helper().hit()",
        ),
        (
            "struct Foreign { void hit() const; }; auto helper(Foreign& value) { return value; }",
            "Foreign value; helper(value).hit()",
        ),
        (
            "struct Foreign { void hit() const; }; Foreign makeForeign(); auto helper() { auto nested=[] { return dart::makeA(); }; return makeForeign(); }",
            "helper().hit()",
        ),
        (
            "struct Foreign { void hit() const; }; Foreign makeForeign(); auto helper() { struct Nested { auto make() { return dart::makeA(); } }; return makeForeign(); }",
            "helper().hit()",
        ),
    ],
)
def test_known_foreign_auto_helper_returns_remain_non_dart(
    extractor, tmp_path, prefix, expression
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(AUTO_HELPER_HEADER)
    source = prefix + " void use(dart::A* known) { " + expression + "; }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "hit" not in _entries(before, "members")
    header.write_text(
        AUTO_HELPER_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) == before


@pytest.mark.parametrize(
    "prefix",
    [
        "auto helper();",
        "auto helper() -> dart::A;",
        "auto helper() { return helper(); }",
        "auto first(); auto helper() { return first(); } auto first() { return helper(); }",
    ],
)
def test_unresolved_auto_helpers_remain_bounded_and_conservative(
    extractor, tmp_path, prefix
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(AUTO_HELPER_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        prefix + " void use(dart::A* known) { helper().hit(); }",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == [] and hit["unresolved_count"] == 1


@pytest.mark.parametrize(
    "prefix,expression",
    [
        (
            "auto helper() noexcept(noexcept(dart::pointer()->hit())) { return dart::makeA(); }",
            "helper().hit()",
        ),
        (
            "auto helper() { auto nested=[]() noexcept(noexcept(dart::makeA())) -> dart::B { return {}; }; return dart::makeA(); }",
            "helper().hit()",
        ),
        (
            "auto helper() { auto nested=[value=dart::makeA()]() mutable noexcept -> dart::B { return {}; }; return dart::makeA(); }",
            "helper().hit()",
        ),
    ],
)
def test_auto_helper_suffix_and_capture_bodies_keep_own_returns(
    extractor, tmp_path, prefix, expression
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(AUTO_HELPER_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        prefix + " void use() { " + expression + "; }",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0


def test_auto_helper_member_return_uses_own_scope_before_namespace(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(AUTO_HELPER_HEADER)
    source = """struct Foreign { void hit() const; }; dart::A inner();
 struct Source { Foreign inner(); auto helper() { return inner(); } };
 void use(Source& source,dart::A* known) { source.helper().hit(); }"""
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "hit" not in _entries(before, "members")
    header.write_text(
        AUTO_HELPER_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) == before


def test_primitive_auto_helper_return_is_not_a_dart_receiver(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(AUTO_HELPER_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "auto helper(int value) { return value; } void use(dart::A* known) { helper(1).hit(); }",
    )
    assert "hit" not in _entries(result, "members")


def test_auto_helper_return_uses_included_header_context(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(AUTO_HELPER_HEADER)
    sources = {
        "helper.hh": "using dart::makeA; auto helper() { return makeA(); }",
        "client.cc": '#include "helper.hh"\nvoid use() { helper().hit(); }',
    }
    result = _scan_files(extractor, extractor.index_api(headers), tmp_path, sources)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0


def test_auto_helper_alternative_definitions_keep_return_union(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct A { void hit(); }; struct B { void hit(); }; A makeA(); B makeB(); }"
    )
    source = """#if PICK_FIRST
 auto helper() { return dart::makeA(); }
 #else
 auto helper() { return dart::makeB(); }
 #endif
 void use() { helper().hit(); }"""
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert (
        hit["receiver_types"] == ["dart::A", "dart::B"] and hit["unresolved_count"] == 0
    )


def test_auto_helper_conditional_return_keeps_operand_candidates(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct A { void hit(); }; struct B:A { void hit(); }; A makeA(); B makeB(); }"
    )
    source = "auto helper(bool pick) { return pick ? dart::makeA() : dart::makeB(); } void use() { helper(true).hit(); }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert (
        hit["receiver_types"] == ["dart::A", "dart::B"] and hit["unresolved_count"] == 0
    )


CALLBACK_SHADOWS = [
    ("local_callback", "void use() { Foreign (*helper)()=&foreign; helper().hit(); }"),
    ("parameter_callback", "void use(Foreign (*helper)()) { helper().hit(); }"),
    (
        "namespace_callback",
        "namespace consumer { Foreign (*helper)()=&foreign; void use() { helper().hit(); } }",
    ),
    (
        "member_callback",
        "struct Source { Foreign (*helper)()=&foreign; void use() { helper().hit(); } };",
    ),
]


@pytest.mark.parametrize("name,body", CALLBACK_SHADOWS)
def test_callbacks_shadow_deduced_source_helper_functions(
    extractor, tmp_path, name, body
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(AUTO_HELPER_HEADER)
    source = (
        "struct Foreign { void hit() const; }; Foreign foreign(); auto helper() { return dart::makeA(); } "
        + body
    )
    files = {"sample.cc": source}
    before = _scan_files(
        extractor, extractor.index_api(headers), tmp_path / "before", files
    )
    assert "hit" not in _entries(before, "members")
    header.write_text(
        AUTO_HELPER_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    assert (
        _scan_files(extractor, extractor.index_api(headers), tmp_path / "after", files)
        == before
    )


@pytest.mark.parametrize(
    "body",
    [
        "struct Source { Foreign (*helper)()=&foreign; void use() { helper().hit(); } };",
        "namespace consumer { Foreign (*helper)()=&foreign; void use() { helper().hit(); } }",
        "void local() { Foreign (*helper)()=&foreign; helper().hit(); }",
    ],
)
def test_callback_shadows_stay_within_their_own_scope(extractor, tmp_path, body):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(AUTO_HELPER_HEADER)
    source = (
        "struct Foreign { void hit() const; }; Foreign foreign(); auto helper() { return dart::makeA(); } "
        + body
        + " void outside() { helper().hit(); }"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(before, "members")["hit"]
    assert (
        hit["count"] == 1
        and hit["receiver_types"] == ["dart::A"]
        and hit["unresolved_count"] == 0
    )
    header.write_text(
        AUTO_HELPER_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) != before


@pytest.mark.parametrize(
    "declaration",
    [
        "Foreign (*helper)()=&foreign;",
        "Foreign (* const helper)()=&foreign;",
        "Foreign (&helper)()=foreign;",
    ],
)
def test_known_foreign_callback_shadow_stays_non_dart_with_known_receivers(
    extractor, tmp_path, declaration
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(AUTO_HELPER_HEADER)
    source = (
        "struct Foreign { void hit() const; }; Foreign foreign(); auto helper() { return dart::makeA(); } void use(dart::A* known) { "
        + declaration
        + " helper().hit(); }"
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "hit" not in _entries(result, "members")


def test_dart_callback_shadow_retains_conservative_candidates(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(AUTO_HELPER_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "auto helper() { return dart::makeA(); } void use(dart::A (*helper)()) { helper().hit(); }",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == [] and hit["unresolved_count"] == 1


DEDUCED_CONDITIONAL_HEADER = "namespace dart { struct Base { void hit() const; }; struct Derived:Base {}; Base* base(); Derived* derived(); std::shared_ptr<Base> sharedBase(); std::shared_ptr<Derived> sharedDerived(); }"
DEDUCED_CONDITIONAL_CASES = [
    (
        "raw_base_derived",
        "auto helper(bool flag) { return flag ? dart::base() : dart::derived(); } void use() { helper(true)->hit(); }",
    ),
    (
        "smart_base_derived",
        "auto helper(bool flag) { return flag ? dart::sharedBase() : dart::sharedDerived(); } void use() { helper(true)->hit(); }",
    ),
]


@pytest.mark.parametrize("name,source", DEDUCED_CONDITIONAL_CASES)
def test_deduced_conditional_helpers_keep_both_real_dart_owners(
    extractor, tmp_path, name, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(DEDUCED_CONDITIONAL_HEADER)
    before = _scan_files(
        extractor,
        extractor.index_api(headers),
        tmp_path / "before",
        {"sample.cc": source},
    )
    row = _entries(before, "members")["hit"]
    assert row["receiver_types"] == ["dart::Base", "dart::Derived"]
    assert row["unresolved_count"] == 0
    header.write_text(
        DEDUCED_CONDITIONAL_HEADER.replace(
            "void hit() const;", "void hit() const = delete;"
        )
    )
    assert (
        _scan_files(
            extractor,
            extractor.index_api(headers),
            tmp_path / "after",
            {"sample.cc": source},
        )
        != before
    )


def test_callback_parameter_shadow_stays_within_its_function(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(AUTO_HELPER_HEADER)
    source = "struct Foreign { void hit() const; }; Foreign foreign(); auto helper() { return dart::makeA(); } void use(Foreign (*helper)()) { helper().hit(); } void outside() { helper().hit(); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    row = _entries(before, "members")["hit"]
    assert row["count"] == 1
    assert row["receiver_types"] == ["dart::A"]
    assert row["unresolved_count"] == 0
    header.write_text(
        AUTO_HELPER_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
        != before
    )


QUOTED_INITIALIZER_CASES = []


def quoted_initializer_case(name, header, old, new, source, changed=True):
    QUOTED_INITIALIZER_CASES.append(
        (name, header, header.replace(old, new), source, changed, old)
    )


quoted_initializer_case(
    "char_equals",
    "namespace dart { struct A { char x='a'; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "char_brace",
    "namespace dart { struct A { char x{'a'}; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "string_array",
    'namespace dart { struct A { char x[3]="aa"; }; }',
    '"aa"',
    '"bb"',
    "constexpr dart::A value; static_assert(value.x[0]=='a');",
)
quoted_initializer_case(
    "string_brace",
    'namespace dart { struct A { char x[3]{"aa"}; }; }',
    '"aa"',
    '"bb"',
    "constexpr dart::A value; static_assert(value.x[0]=='a');",
)
quoted_initializer_case(
    "char_array_brace",
    "namespace dart { struct A { char x[2]={'a','x'}; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x[0]=='a');",
)
quoted_initializer_case(
    "escaped_char",
    r"namespace dart { struct A { char x='\x61'; }; }",
    r"'\x61'",
    r"'\x62'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "wide_char",
    "namespace dart { struct A { wchar_t x=L'a'; }; }",
    "L'a'",
    "L'b'",
    "constexpr dart::A value; static_assert(value.x==L'a');",
)
quoted_initializer_case(
    "unicode_char",
    "namespace dart { struct A { char32_t x=U'a'; }; }",
    "U'a'",
    "U'b'",
    "constexpr dart::A value; static_assert(value.x==U'a');",
)
quoted_initializer_case(
    "utf8_string",
    'namespace dart { struct A { const char* x=u8"aa"; }; }',
    'u8"aa"',
    'u8"bb"',
    "constexpr dart::A value; static_assert(value.x[0]=='a');",
)
quoted_initializer_case(
    "escaped_string",
    r'namespace dart { struct A { const char* x="\x61,\"{x}"; }; }',
    r'"\x61,\"{x}"',
    r'"\x62,\"{x}"',
    "constexpr dart::A value; static_assert(value.x[0]=='a');",
)
quoted_initializer_case(
    "raw_string",
    'namespace dart { struct A { const char* x=R"tag(a, {dart::A{}.hit();})tag"; }; }',
    'R"tag(a, {dart::A{}.hit();})tag"',
    'R"tag(b, {dart::A{}.hit();})tag"',
    "constexpr dart::A value; static_assert(value.x[0]=='a');",
)
quoted_initializer_case(
    "adjacent_string",
    'namespace dart { struct A { const char* x="a" "a"; }; }',
    '"a" "a"',
    '"b" "a"',
    "constexpr dart::A value; static_assert(value.x[0]=='a');",
)
quoted_initializer_case(
    "grouped_first",
    "namespace dart { struct A { char x='a', y='q'; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "grouped_second",
    "namespace dart { struct A { char x='q', y='a'; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.y=='a');",
)
quoted_initializer_case(
    "namespace_char",
    "namespace dart { inline constexpr char K='a'; struct A { char x=K; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "namespace_string",
    'namespace dart { inline constexpr char K[3]="aa"; struct A { char x=K[0]; }; }',
    '"aa"',
    '"bb"',
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "namespace_brace",
    "namespace dart { inline constexpr char K{'a'}; struct A { char x=K; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "namespace_grouped",
    "namespace dart { inline constexpr char K='q', L='a'; struct A { char x=L; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "class_char",
    "namespace dart { struct A { static constexpr char K='a'; char x=K; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "class_string",
    'namespace dart { struct A { inline static constexpr char K[3]="aa"; char x=K[0]; }; }',
    '"aa"',
    '"bb"',
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "transitive_char",
    "namespace dart { inline constexpr char BASE='a'; inline constexpr char K=BASE; struct A { char x=K; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "comment_control",
    "namespace dart { struct A { char x='a' /* 'q' \"fake()\" */; }; }",
    "'q'",
    "'z'",
    "constexpr dart::A value; static_assert(value.x=='a');",
    False,
)
quoted_initializer_case(
    "unused_type",
    "namespace dart { struct A { char x='a'; }; }",
    "'a'",
    "'b'",
    "void use(dart::A* value);",
    False,
)
quoted_initializer_case(
    "unused_macro",
    '#define DART_TEXT "dart::A{}, fake(), {x}"\nnamespace dart { struct A { int x=1; }; }',
    '"dart::A{}, fake(), {x}"',
    '"dart::A{}.hit(), {y}"',
    "constexpr dart::A value; static_assert(value.x==1);",
    False,
)
quoted_initializer_case(
    "source_string",
    "namespace dart { struct A { char x='a'; void hit(); }; }",
    "'a'",
    "'b'",
    'const char* value="dart::A{}.hit(), {x}";',
    False,
)
quoted_initializer_case(
    "parenthesized_char",
    "namespace dart { inline constexpr char K('a'); struct A { char x=K; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "parenthesized_nested_char",
    "namespace dart { inline constexpr char K(('a')); struct A { char x=K; }; }",
    "'a'",
    "'b'",
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "parenthesized_string_pointer",
    'namespace dart { inline constexpr const char* const K("aa"); struct A { char x=K[0]; }; }',
    '"aa"',
    '"bb"',
    "constexpr dart::A value; static_assert(value.x=='a');",
)
quoted_initializer_case(
    "function_zero_argument",
    "namespace dart { constexpr char f(); /* 'a' */ }",
    "'a'",
    "'b'",
    "void use() { dart::f(); }",
    False,
)
quoted_initializer_case(
    "function_typed_parameter",
    "namespace dart { constexpr char f(char value); /* 'a' */ }",
    "'a'",
    "'b'",
    "void use() { dart::f('q'); }",
    False,
)
quoted_initializer_case(
    "function_defaulted_parameter",
    "namespace dart { constexpr char f(char value='a'); }",
    "'a'",
    "'b'",
    "void use() { dart::f('q'); }",
    False,
)
quoted_initializer_case(
    "function_alias_defaulted_parameter",
    "namespace dart { using Alias=char; constexpr char f(Alias value='a'); }",
    "'a'",
    "'b'",
    "void use() { dart::f('q'); }",
    False,
)
quoted_initializer_case(
    "function_defined",
    "namespace dart { constexpr char f() { return 'a'; } }",
    "'a'",
    "'b'",
    "void use() { dart::f(); }",
    False,
)


@pytest.mark.parametrize(
    "name,before_text,after_text,source,changed,literal", QUOTED_INITIALIZER_CASES
)
def test_quoted_default_field_initializer_evidence(
    extractor, tmp_path, name, before_text, after_text, source, changed, literal
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(before_text)
    api = extractor.index_api(headers)
    if name.startswith("function_"):
        assert "dart::f" in api.free_functions
        assert "dart::f" not in api.variables
    before = _scan(extractor, api, tmp_path / "before", source)
    assert not before["members"]
    if changed:
        declarations = _entries(before, "constructors")["dart::A"][
            "default_member_initializer_declarations"
        ]
        values = [
            value
            for declaration in declarations
            for value in declaration.get("initializer_literals", [])
        ]
        if name == "adjacent_string":
            assert values == ['"a"', '"a"']
        else:
            assert literal in values
        if name.startswith("grouped_"):
            by_signature = {
                d["signature"]: d.get("initializer_literals", []) for d in declarations
            }
            assert by_signature["char x ="] == (
                ["'a'"] if name == "grouped_first" else ["'q'"]
            )
            assert by_signature["char y ="] == (
                ["'q'"] if name == "grouped_first" else ["'a'"]
            )
    header.write_text(after_text)
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    if name in {"function_defaulted_parameter", "function_alias_defaulted_parameter"}:
        declarations = _entries(before, "free_functions")["dart::f"]["declarations"]
        assert any(
            literal in declaration.get("declaration_literals", [])
            for declaration in declarations
        )
        assert before != after
    else:
        assert (before != after) == changed


ENUM_LITERAL_CASES = [
    (
        "enum_scoped",
        "namespace dart{enum class Flag:char{K='a'};struct A{Flag x=Flag::K;};}",
        "namespace dart{enum class Flag:char{K='b'};struct A{Flag x=Flag::K;};}",
        "constexpr dart::A value;static_assert(value.x==static_cast<dart::Flag>('a'));",
        True,
        "'a'",
    ),
    (
        "enum_unscoped",
        "namespace dart{enum Flag{K='a'};struct A{char x=K;};}",
        "namespace dart{enum Flag{K='b'};struct A{char x=K;};}",
        "constexpr dart::A value;static_assert(value.x=='a');",
        True,
        "'a'",
    ),
    (
        "enum_anonymous",
        "namespace dart{enum:unsigned{K='a'};struct A{char x=K;};}",
        "namespace dart{enum:unsigned{K='b'};struct A{char x=K;};}",
        "constexpr dart::A value;static_assert(value.x=='a');",
        True,
        "'a'",
    ),
    (
        "enum_private",
        "namespace dart{struct A{private:enum:unsigned{K='a'};public:char x=K;};}",
        "namespace dart{struct A{private:enum:unsigned{K='b'};public:char x=K;};}",
        "constexpr dart::A value;static_assert(value.x=='a');",
        True,
        "'a'",
    ),
    (
        "enum_implicit_dependency",
        "namespace dart{enum class Flag:char{K='a',Next};struct A{Flag x=Flag::Next;};}",
        "namespace dart{enum class Flag:char{K='b',Next};struct A{Flag x=Flag::Next;};}",
        "constexpr dart::A value;static_assert(value.x==static_cast<dart::Flag>('b'));",
        True,
        "'a'",
    ),
    (
        "enum_nested_class",
        "namespace dart { struct A { enum class Flag:char { K='a' }; Flag x=Flag::K; }; }",
        "namespace dart { struct A { enum class Flag:char { K='b' }; Flag x=Flag::K; }; }",
        "constexpr dart::A value; static_assert(value.x==static_cast<dart::A::Flag>('a'));",
        True,
        "'a'",
    ),
    (
        "enum_explicit_dependency",
        "namespace dart { enum class Flag:char { K='a', Next=K }; struct A { Flag x=Flag::Next; }; }",
        "namespace dart { enum class Flag:char { K='b', Next=K }; struct A { Flag x=Flag::Next; }; }",
        "constexpr dart::A value; static_assert(value.x==static_cast<dart::Flag>('a'));",
        True,
        "'a'",
    ),
    (
        "enum_comma_character",
        "namespace dart { enum class Flag:char { K=',',Other='q' }; struct A { Flag x=Flag::K; }; }",
        "namespace dart { enum class Flag:char { K='.',Other='q' }; struct A { Flag x=Flag::K; }; }",
        "constexpr dart::A value; static_assert(value.x==static_cast<dart::Flag>(','));",
        True,
        "','",
    ),
    (
        "enum_brace_character",
        "namespace dart { enum class Flag:char { K='{' }; struct A { Flag x=Flag::K; }; }",
        "namespace dart { enum class Flag:char { K='}' }; struct A { Flag x=Flag::K; }; }",
        "constexpr dart::A value; static_assert(value.x==static_cast<dart::Flag>('{'));",
        True,
        "'{'",
    ),
    (
        "enum_escaped_character",
        "namespace dart { enum class Flag:char { K='\\x61' }; struct A { Flag x=Flag::K; }; }",
        "namespace dart { enum class Flag:char { K='\\x62' }; struct A { Flag x=Flag::K; }; }",
        "constexpr dart::A value; static_assert(value.x==static_cast<dart::Flag>('a'));",
        True,
        "'\\x61'",
    ),
    (
        "enum_wide_character",
        "namespace dart { enum class Flag:wchar_t { K=L'a' }; struct A { Flag x=Flag::K; }; }",
        "namespace dart { enum class Flag:wchar_t { K=L'b' }; struct A { Flag x=Flag::K; }; }",
        "constexpr dart::A value; static_assert(value.x==static_cast<dart::Flag>(L'a'));",
        True,
        "L'a'",
    ),
    (
        "enum_unused_type",
        "namespace dart { enum class Flag:char { K='a' }; struct A { Flag x=Flag::K; }; }",
        "namespace dart { enum class Flag:char { K='b' }; struct A { Flag x=Flag::K; }; }",
        "void use(dart::A* value);",
        False,
        "'a'",
    ),
    (
        "enum_comment_control",
        "namespace dart { enum class Flag:char { K='a' /* 'q' */ }; struct A { Flag x=Flag::K; }; }",
        "namespace dart { enum class Flag:char { K='a' /* 'z' */ }; struct A { Flag x=Flag::K; }; }",
        "constexpr dart::A value; static_assert(value.x==static_cast<dart::Flag>('a'));",
        False,
        "'q'",
    ),
]


@pytest.mark.parametrize(
    "name,before_text,after_text,source,changed,literal", ENUM_LITERAL_CASES
)
def test_enum_quoted_default_initializer_closure(
    extractor, tmp_path, name, before_text, after_text, source, changed, literal
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(before_text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    assert not before["members"]
    if changed:
        declarations = _entries(before, "constructors")["dart::A"][
            "default_member_initializer_declarations"
        ]
        assert any(literal in d.get("initializer_literals", []) for d in declarations)
        if "dependency" in name:
            assert any("Next" in d["signature"].split() for d in declarations)
            assert any("K" in d["signature"].split() for d in declarations)
    header.write_text(after_text)
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (before != after) == changed


QUOTED_DEFAULT_CASES = [
    (
        "free",
        "namespace dart { constexpr char f(char value='a') { return value; } }",
        "'a'",
        "'b'",
        "static_assert(dart::f()=='a');",
        True,
        "free_functions",
        "dart::f",
    ),
    (
        "member",
        "namespace dart { struct A { constexpr char f(char value='a') const { return value; } }; }",
        "'a'",
        "'b'",
        "constexpr dart::A value; static_assert(value.f()=='a');",
        True,
        "members",
        "f",
    ),
    (
        "constructor",
        "namespace dart { struct A { char x; constexpr A(char value='a'):x(value) {} }; }",
        "'a'",
        "'b'",
        "constexpr dart::A value; static_assert(value.x=='a');",
        True,
        "constructors",
        "dart::A",
    ),
    (
        "string",
        'namespace dart { constexpr char f(const char* value="aa") { return value[0]; } }',
        '"aa"',
        '"bb"',
        "static_assert(dart::f()=='a');",
        True,
        "free_functions",
        "dart::f",
    ),
    (
        "second_parameter",
        "namespace dart { constexpr char f(int n=1,char value='a') { return value; } }",
        "'a'",
        "'b'",
        "static_assert(dart::f()=='a');",
        True,
        "free_functions",
        "dart::f",
    ),
    (
        "raw_string",
        'namespace dart { constexpr char f(const char* value=R"tag(a,{dart::A{}.hit();})tag") { return value[0]; } }',
        'R"tag(a,{dart::A{}.hit();})tag"',
        'R"tag(b,{dart::A{}.hit();})tag"',
        "static_assert(dart::f()=='a');",
        True,
        "free_functions",
        "dart::f",
    ),
    (
        "escaped",
        r"namespace dart { constexpr char f(char value='\x61') { return value; } }",
        r"'\x61'",
        r"'\x62'",
        "static_assert(dart::f()=='a');",
        True,
        "free_functions",
        "dart::f",
    ),
    (
        "comment",
        "namespace dart { constexpr char f(char value='a' /* 'q' */) { return value; } }",
        "'q'",
        "'z'",
        "static_assert(dart::f()=='a');",
        False,
        "free_functions",
        "dart::f",
    ),
    (
        "unused",
        "namespace dart { constexpr char f(char value='a') { return value; } }",
        "'a'",
        "'b'",
        "void use() {}",
        False,
        "free_functions",
        "dart::f",
    ),
    (
        "body",
        "namespace dart { constexpr char f() { return 'a'; } }",
        "'a'",
        "'b'",
        "void use() { dart::f(); }",
        False,
        "free_functions",
        "dart::f",
    ),
]


QUOTED_DEFAULT_CASES.extend(
    [
        (
            "grouped_first",
            "namespace dart { constexpr char f(char='a'), g(char='x'); constexpr char f(char v) {return v;} constexpr char g(char v) {return v;} }",
            "'a'",
            "'b'",
            "static_assert(dart::f()=='a');",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "grouped_second",
            "namespace dart { constexpr char f(char='x'), g(char='a'); constexpr char f(char v) {return v;} constexpr char g(char v) {return v;} }",
            "'a'",
            "'b'",
            "static_assert(dart::g()=='a');",
            True,
            "free_functions",
            "dart::g",
        ),
        (
            "unnamed",
            "namespace dart { constexpr char f(char='a'); constexpr char f(char v) {return v;} }",
            "'a'",
            "'b'",
            "static_assert(dart::f()=='a');",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "alias",
            "namespace dart { using Char=char; constexpr Char f(Char v='a') {return v;} }",
            "'a'",
            "'b'",
            "static_assert(dart::f()=='a');",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "member_static",
            "namespace dart { struct A { static constexpr char f(char v='a') {return v;} }; }",
            "'a'",
            "'b'",
            "static_assert(dart::A::f()=='a');",
            True,
            "members",
            "f",
        ),
        (
            "out_of_line",
            "namespace dart { struct A { constexpr char f(char v='a') const; }; constexpr char A::f(char v) const {return v;} }",
            "'a'",
            "'b'",
            "constexpr dart::A value; static_assert(value.f()=='a');",
            True,
            "members",
            "f",
        ),
        (
            "macro_name",
            "#define DART_FN f\nnamespace dart { constexpr char DART_FN(char v='a') {return v;} }",
            "'a'",
            "'b'",
            "static_assert(dart::f()=='a');",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "macro_owner",
            "#define DART_OWNER A\nnamespace dart { struct DART_OWNER { constexpr char f(char v='a') const {return v;} }; }",
            "'a'",
            "'b'",
            "constexpr dart::A value; static_assert(value.f()=='a');",
            True,
            "members",
            "f",
        ),
        (
            "wide",
            "namespace dart { constexpr wchar_t f(wchar_t v=L'a') {return v;} }",
            "L'a'",
            "L'b'",
            "static_assert(dart::f()==L'a');",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "lambda_default",
            "namespace dart { constexpr char f(char v=[] {return 'a';}()) {return v;} }",
            "'a'",
            "'b'",
            "static_assert(dart::f()=='a');",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "braced_default",
            "namespace dart { constexpr char f(char v={'a'}) {return v;} }",
            "'a'",
            "'b'",
            "static_assert(dart::f()=='a');",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "nested_calls",
            "namespace dart { constexpr char choose(char x,char) {return x;} constexpr char f(char v=choose('a','x')) {return v;} }",
            "'a'",
            "'b'",
            "static_assert(dart::f()=='a');",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "foreign",
            "namespace foreign { constexpr char f(char v='a') {return v;} }",
            "'a'",
            "'b'",
            "void use() {foreign::f();}",
            False,
            "free_functions",
            "foreign::f",
        ),
    ]
)


QUOTED_DEFAULT_CASES.extend(
    [
        (
            "noexcept_literal",
            "namespace dart { constexpr bool f() noexcept('a'==97) {return true;} }",
            "'a'",
            "'b'",
            "static_assert(noexcept(dart::f()));",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "return_template_literal",
            "#include <type_traits>\nnamespace dart { std::integral_constant<char,'a'> f(); }",
            "'a'",
            "'b'",
            "static_assert(decltype(dart::f())::value=='a');",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "parameter_template_literal",
            "#include <type_traits>\nnamespace dart { constexpr char f(std::integral_constant<char,'a'> v={}) {return v;} }",
            "'a'",
            "'b'",
            "static_assert(dart::f()=='a');",
            True,
            "free_functions",
            "dart::f",
        ),
        (
            "template_comma_default",
            "#include <array>\nnamespace dart { constexpr char f(std::array<char,2> v={{'a','x'}},char z='q') {return v[0];} }",
            "'a'",
            "'b'",
            "static_assert(dart::f()=='a');",
            True,
            "free_functions",
            "dart::f",
        ),
    ]
)


QUOTED_DEFAULT_CASES.extend(
    [
        (
            "grouped_unused",
            "namespace dart { constexpr char f(char='q'), g(char='a'); constexpr char f(char v) {return v;} constexpr char g(char v) {return v;} }",
            "'a'",
            "'b'",
            "static_assert(dart::f()=='q');",
            False,
            "free_functions",
            "dart::f",
        ),
        (
            "unused_member",
            "namespace dart { struct A { constexpr char f() const {return 0;} constexpr char g(char v='a') const {return v;} }; }",
            "'a'",
            "'b'",
            "constexpr dart::A value; static_assert(value.f()==0);",
            False,
            "members",
            "f",
        ),
        (
            "call_operator",
            "namespace dart { struct A { constexpr char operator()(char v='a') const {return v;} }; }",
            "'a'",
            "'b'",
            "constexpr dart::A value; static_assert(value()=='a');",
            True,
            "members",
            "operator()",
        ),
    ]
)


@pytest.mark.parametrize(
    "name,header_text,old,new,source,changed,category,key", QUOTED_DEFAULT_CASES
)
def test_quoted_parameter_defaults_retain_literal_evidence(
    extractor, tmp_path, name, header_text, old, new, source, changed, category, key
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(header_text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    if changed:
        declarations = _entries(before, category)[key]["declarations"]
        assert any(old in d.get("declaration_literals", []) for d in declarations)
    header.write_text(header_text.replace(old, new))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (before != after) == changed


ROUND16_SPEC_CASES = [
    (
        "parenthesized_field_reference",
        "namespace dart { class A { int field; public: void hit() const; }; struct Holder { A item; }; A make(); A& reference(); Holder& holder(); }",
        "int field",
        "double field",
        "void use(dart::Holder& source) { decltype((source.item)) value=source.item; value.hit(); }",
        False,
        False,
    ),
    (
        "parenthesized_arrow_field_reference",
        "namespace dart { class A { int field; public: void hit() const; }; struct Holder { A item; }; A make(); A& reference(); Holder& holder(); }",
        "int field",
        "double field",
        "void use(dart::Holder* source) { decltype((source->item)) value=source->item; value.hit(); }",
        False,
        False,
    ),
    (
        "parenthesized_call_field_reference",
        "namespace dart { class A { int field; public: void hit() const; }; struct Holder { A item; }; A make(); A& reference(); Holder& holder(); }",
        "int field",
        "double field",
        "void use() { decltype((dart::holder().item)) value=dart::holder().item; value.hit(); }",
        False,
        False,
    ),
    (
        "unparenthesized_field_value",
        "namespace dart { class A { int field; public: void hit() const; }; struct Holder { A item; }; A make(); A& reference(); Holder& holder(); }",
        "int field",
        "double field",
        "void use(dart::Holder& source) { decltype(source.item) value=source.item; static_assert(sizeof(value)==sizeof(int)); value.hit(); }",
        True,
        True,
    ),
    (
        "decltype_auto_field_reference",
        "namespace dart { class A { int field; public: void hit() const; }; struct Holder { A item; }; A make(); A& reference(); Holder& holder(); }",
        "int field",
        "double field",
        "void use(dart::Holder& source) { decltype(auto) value=(source.item); value.hit(); }",
        False,
        False,
    ),
    (
        "parenthesized_value_call",
        "namespace dart { class A { int field; public: void hit() const; }; struct Holder { A item; }; A make(); A& reference(); Holder& holder(); }",
        "int field",
        "double field",
        "void use() { decltype((dart::make())) value=dart::make(); static_assert(sizeof(value)==sizeof(int)); value.hit(); }",
        True,
        True,
    ),
    (
        "parenthesized_reference_call",
        "namespace dart { class A { int field; public: void hit() const; }; struct Holder { A item; }; A make(); A& reference(); Holder& holder(); }",
        "int field",
        "double field",
        "void use() { decltype((dart::reference())) value=dart::reference(); value.hit(); }",
        False,
        False,
    ),
    (
        "inferred_callable_value",
        "namespace dart { class A { int field; public: void hit() const; }; struct Holder { A item; }; A make(); A& reference(); Holder& holder(); }",
        "int field",
        "double field",
        "void use() { auto factory=&dart::make; auto value=factory(); static_assert(sizeof(value)==sizeof(int)); value.hit(); }",
        True,
        True,
    ),
    (
        "inferred_callable_reference",
        "namespace dart { class A { int field; public: void hit() const; }; struct Holder { A item; }; A make(); A& reference(); Holder& holder(); }",
        "int field",
        "double field",
        "void use() { auto factory=&dart::reference; decltype(auto) value=factory(); value.hit(); }",
        False,
        False,
    ),
    (
        "unnamed_bitfield_layout",
        "namespace dart { class A { unsigned int : 8; }; }",
        "int : 8",
        "int : 16",
        "static_assert(sizeof(dart::A)==1);",
        True,
        True,
    ),
    (
        "padding_bitfield_layout",
        "namespace dart { class A { unsigned int : 8; char field; }; }",
        "int : 8",
        "int : 16",
        "static_assert(sizeof(dart::A)==2);",
        True,
        True,
    ),
]

ROUND16_VISIBILITY_HEADER = (
    "namespace dart { struct A { void hit() const; }; A make(); }"
)

ROUND16_VISIBILITY_CASES = [
    (
        "same_file_global_callable",
        True,
        {
            "caller.cc": "namespace provider { inline auto factory=&dart::make; } void use() { provider::factory().hit(); }"
        },
    ),
    (
        "included_global_callable",
        True,
        {
            "helper.hh": "#include <dart/api.hpp>\nnamespace provider { inline auto factory=&dart::make; }",
            "caller.cc": '#include "helper.hh"\nvoid use() { provider::factory().hit(); }',
        },
    ),
    (
        "imported_global_callable",
        True,
        {
            "helper.hh": "#include <dart/api.hpp>\nnamespace provider { inline auto factory=&dart::make; }",
            "caller.cc": '#include "helper.hh"\nusing provider::factory; void use() { factory().hit(); }',
        },
    ),
    (
        "unrelated_header_callable",
        False,
        {
            "unused.hh": "#include <dart/api.hpp>\nnamespace provider { inline auto factory=&dart::make; }",
            "caller.cc": "struct Foreign { void hit() const; }; namespace provider { Foreign factory(); } void use() { provider::factory().hit(); }",
        },
    ),
]


def _round16_spec_strings(value):
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from _round16_spec_strings(item)
    elif isinstance(value, list):
        for item in value:
            yield from _round16_spec_strings(item)


@pytest.mark.parametrize(
    "name,text,old,new,source,changed,breaking", ROUND16_SPEC_CASES
)
def test_round16_spec_connected_decltype_and_layout(
    extractor, tmp_path, name, text, old, new, source, changed, breaking
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path / "before", source)
    if "bitfield" not in name:
        hit = _entries(before, "members")["hit"]
        assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0
        assert {d["class"] for d in hit["declarations"]} == {"dart::A"}
    assert not any(
        value.startswith(
            (
                "__callable__:",
                "__associated__:",
                "__inline_layout__:",
                "__reference__",
                "__no_inline_layout__",
                "__declared_type__",
            )
        )
        for value in _round16_spec_strings(before)
    )
    header.write_text(text.replace(old, new))
    after = _scan(extractor, extractor.index_api(headers), tmp_path / "after", source)
    assert (before != after) == changed


@pytest.mark.parametrize("name,positive,files", ROUND16_VISIBILITY_CASES)
def test_round16_spec_namespace_callable_storage_visibility(
    extractor, tmp_path, name, positive, files
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(ROUND16_VISIBILITY_HEADER)
    before = _scan_files(
        extractor, extractor.index_api(headers), tmp_path / "before", files
    )
    hit = _entries(before, "members").get("hit")
    assert bool(hit) == positive
    if positive:
        assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0
    header.write_text(
        ROUND16_VISIBILITY_HEADER.replace(
            "void hit() const;", "void hit() const = delete;"
        )
    )
    after = _scan_files(
        extractor, extractor.index_api(headers), tmp_path / "after", files
    )
    assert (before != after) == positive


RECEIVER16_HEADER = """namespace dart {
struct A { void hit() const; A& operator()(); A* operator->(); A& operator[](int); };
using Alias=A;
A makeA(); A& reference(); A* pointer(); Alias alias();
std::shared_ptr<A> shared(); std::weak_ptr<A> weak(); std::optional<A> optional();
std::vector<A> sequence(); std::optional<std::shared_ptr<A>> nested();
}"""
DECLTYPE16_CASES = [
    (
        "direct",
        "void use() { decltype(dart::makeA()) value=dart::makeA(); value.hit(); }",
    ),
    ("uninitialized", "void use() { decltype(dart::makeA()) value; value.hit(); }"),
    ("braced_init", "void use() { decltype(dart::makeA()) value{}; value.hit(); }"),
    ("parameter", "void use(decltype(dart::makeA()) value) { value.hit(); }"),
    (
        "const_parameter",
        "void use(const decltype(dart::makeA())& value) { value.hit(); }",
    ),
    (
        "pointer",
        "void use() { decltype(dart::pointer()) value=dart::pointer(); value->hit(); }",
    ),
    ("pointer_suffix", "void use(decltype(dart::makeA())* value) { value->hit(); }"),
    (
        "reference",
        "void use() { decltype(dart::reference()) value=dart::reference(); value.hit(); }",
    ),
    (
        "parenthesized_id",
        "void use() { auto original=dart::makeA(); decltype((original)) value=original; value.hit(); }",
    ),
    (
        "unparenthesized_id",
        "void use() { auto original=dart::makeA(); decltype(original) value=original; value.hit(); }",
    ),
    (
        "grouped",
        "void use() { decltype(dart::makeA()) first=dart::makeA(), second=dart::makeA(); first.hit(); second.hit(); }",
    ),
    (
        "grouped_pointer",
        "void use() { decltype(dart::makeA()) value, *pointer=&value; pointer->hit(); value.hit(); }",
    ),
    ("raw_array", "void use() { decltype(dart::makeA()) values[2]; values[0].hit(); }"),
    (
        "shared",
        "void use() { decltype(dart::shared()) value=dart::shared(); value.get()->hit(); }",
    ),
    (
        "weak",
        "void use() { decltype(dart::weak()) value=dart::weak(); value.lock()->hit(); }",
    ),
    (
        "optional",
        "void use() { decltype(dart::optional()) value=dart::optional(); value.value().hit(); }",
    ),
    (
        "sequence",
        "void use() { decltype(dart::sequence()) value=dart::sequence(); value.front().hit(); }",
    ),
    (
        "nested",
        "void use() { decltype(dart::nested()) value=dart::nested(); value.value().get()->hit(); }",
    ),
    (
        "field",
        "struct Source { decltype(dart::makeA()) value; }; void use(Source& source) { source.value.hit(); }",
    ),
    (
        "namespace_alias",
        "namespace api=dart; void use() { decltype(api::makeA()) value=api::makeA(); value.hit(); }",
    ),
    (
        "imported",
        "using dart::makeA; void use() { decltype(makeA()) value=makeA(); value.hit(); }",
    ),
    (
        "api_alias_return",
        "void use() { decltype(dart::alias()) value=dart::alias(); value.hit(); }",
    ),
    (
        "source_alias",
        "using Value=decltype(dart::makeA()); void use() { Value value=dart::makeA(); value.hit(); }",
    ),
    (
        "source_alias_chain",
        "using Value=decltype(dart::makeA()); using Alias=Value; void use() { Alias value=dart::makeA(); value.hit(); }",
    ),
    (
        "source_decltype_return",
        "decltype(dart::makeA()) helper(); void use() { helper().hit(); }",
    ),
]
CALLABLE16_CASES = [
    ("address", "void use() { auto factory=&dart::makeA; factory().hit(); }"),
    ("no_address", "void use() { auto factory=dart::makeA; factory().hit(); }"),
    ("ref_qualifier", "void use() { auto& factory=dart::makeA; factory().hit(); }"),
    ("forward_ref", "void use() { auto&& factory=dart::makeA; factory().hit(); }"),
    (
        "const_pointer",
        "void use() { const auto factory=&dart::makeA; factory().hit(); }",
    ),
    (
        "alias",
        "void use() { auto factory=&dart::makeA; auto copy=factory; copy().hit(); }",
    ),
    (
        "grouped",
        "void use() { auto first=&dart::makeA, second=&dart::makeA; first().hit(); second().hit(); }",
    ),
    ("deref", "void use() { auto factory=&dart::makeA; (*factory)().hit(); }"),
    ("pointer_return", "void use() { auto factory=&dart::pointer; factory()->hit(); }"),
    (
        "reference_return",
        "void use() { auto factory=&dart::reference; factory().hit(); }",
    ),
    (
        "smart_return",
        "void use() { auto factory=&dart::shared; factory().get()->hit(); }",
    ),
    (
        "weak_return",
        "void use() { auto factory=&dart::weak; factory().lock()->hit(); }",
    ),
    (
        "optional_return",
        "void use() { auto factory=&dart::optional; factory().value().hit(); }",
    ),
    (
        "sequence_return",
        "void use() { auto factory=&dart::sequence; factory().front().hit(); }",
    ),
    (
        "namespace_alias",
        "namespace api=dart; void use() { auto factory=&api::makeA; factory().hit(); }",
    ),
    (
        "using_import",
        "using dart::makeA; void use() { auto factory=&makeA; factory().hit(); }",
    ),
    (
        "using_namespace",
        "using namespace dart; void use() { auto factory=makeA; factory().hit(); }",
    ),
    (
        "lambda_capture",
        "void use() { auto f=[factory=&dart::makeA] { factory().hit(); }; }",
    ),
    (
        "source_function",
        "namespace source { auto make() { return dart::makeA(); } } void use() { auto factory=&source::make; factory().hit(); }",
    ),
    (
        "nested_factory",
        "auto make() { return &dart::makeA; } void use() { auto factory=make(); factory().hit(); }",
    ),
    (
        "decltype_function_pointer",
        "void use() { decltype(&dart::makeA) factory=&dart::makeA; factory().hit(); }",
    ),
    (
        "decltype_function_ref",
        "void use() { decltype(dart::makeA)& factory=dart::makeA; factory().hit(); }",
    ),
    (
        "decltype_function_alias",
        "using Factory=decltype(&dart::makeA); void use() { Factory factory=&dart::makeA; factory().hit(); }",
    ),
]


@pytest.mark.parametrize("name,source", DECLTYPE16_CASES + CALLABLE16_CASES)
def test_decltype_and_function_reference_receivers(extractor, tmp_path, name, source):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(RECEIVER16_HEADER)
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(before, "members")["hit"]
    assert hit["unresolved_count"] == 0
    assert {d["class"] for d in hit["declarations"]} == {"dart::A"}
    assert "operator()" not in _entries(before, "members")
    if "pointer" in name:
        assert "operator->" not in _entries(before, "members")
    header.write_text(
        RECEIVER16_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    after = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(after, "members")["hit"]["declarations"] != hit["declarations"]


@pytest.mark.parametrize(
    "source",
    [
        "struct Foreign { void hit() const; Foreign(dart::A); }; Foreign foreign(); void use() { decltype(foreign()) value=dart::makeA(); value.hit(); }",
        "struct Foreign { void hit() const; }; Foreign foreign(); void use(dart::A* known) { decltype(foreign()) value=foreign(); value.hit(); }",
        "struct Foreign { void hit() const; }; namespace provider { Foreign makeA(); } void use(dart::A* known) { auto factory=&provider::makeA; factory().hit(); }",
        "struct Foreign { void hit() const; }; using namespace dart; namespace consumer { Foreign makeA(); void use(dart::A* known) { auto factory=&makeA; factory().hit(); } }",
        "struct Foreign { void hit() const; }; Foreign foreign(); void use(dart::A* known) { auto factory=&dart::makeA; { auto factory=&foreign; factory().hit(); } }",
    ],
)
def test_decltype_and_function_reference_foreign_controls(extractor, tmp_path, source):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(RECEIVER16_HEADER)
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "hit" not in _entries(before, "members")
    header.write_text(
        RECEIVER16_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) == before


def test_explicit_callback_stays_conservative_with_new_function_markers(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(RECEIVER16_HEADER)
    result = _scan(
        extractor,
        extractor.index_api(headers),
        tmp_path,
        "void use(dart::A (*factory)()) { factory().hit(); }",
    )
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == [] and hit["unresolved_count"] == 1


def test_decltype_uses_declared_dart_type_before_convertible_initializer(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        RECEIVER16_HEADER
        + " namespace dart { struct B:A { void hit(int) const; }; B makeB(); }"
    )
    header.write_text(text)
    source = "void use() { decltype(dart::makeA()) value=dart::makeB(); value.hit(); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(before, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and {
        d["class"] for d in hit["declarations"]
    } == {"dart::A"}
    header.write_text(text.replace("void hit() const;", "void hit() const = delete;"))
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) != before


def test_known_foreign_decltype_return_does_not_remain_unknown(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(RECEIVER16_HEADER)
    source = "struct Foreign { void hit() const; }; Foreign unknown(); template<class T> decltype(unknown()) foreign(); void use(dart::A* known) { auto value=foreign<Foreign>(); value.hit(); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "hit" not in _entries(before, "members")
    header.write_text(
        RECEIVER16_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) == before


def test_constexpr_source_factory_reference_uses_included_context(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = RECEIVER16_HEADER.replace("A makeA();", "constexpr A makeA() { return {}; }")
    header.write_text(text)
    sources = {
        "factory.hh": "namespace provider { constexpr auto make() { return dart::makeA(); } }",
        "client.cc": '#include "factory.hh"\nvoid use() { auto factory=&provider::make; factory().hit(); }',
    }
    before = _scan_files(
        extractor, extractor.index_api(headers), tmp_path / "before", sources
    )
    hit = _entries(before, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0
    header.write_text(text.replace("void hit() const;", "void hit() const = delete;"))
    assert (
        _scan_files(
            extractor, extractor.index_api(headers), tmp_path / "after", sources
        )
        != before
    )


@pytest.mark.parametrize(
    "source",
    [
        "namespace provider { using Value=decltype(dart::shared()); } void use() { provider::Value value=dart::shared(); value.get()->hit(); }",
        "using Value=decltype(dart::makeA()); struct Source { Value value; }; void use(Source& source) { source.value.hit(); }",
        "struct Foreign { void hit() const; }; void use() { decltype(dart::makeA()) value; { Foreign value; value.hit(); } value.hit(); }",
        "struct Foreign { void hit() const; }; Foreign foreign(); void use() { auto factory=&dart::makeA; { auto factory=&foreign; factory().hit(); } factory().hit(); }",
    ],
)
def test_decltype_and_callable_connected_scopes(extractor, tmp_path, source):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(RECEIVER16_HEADER)
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert (
        hit["count"] == 1
        and {d["class"] for d in hit["declarations"]} == {"dart::A"}
        and hit["unresolved_count"] == 0
    )


@pytest.mark.parametrize(
    "name,source",
    [
        (
            "array",
            "void use() { std::array<decltype(&dart::makeA),1> factories{{&dart::makeA}}; factories[0]().hit(); }",
        ),
        (
            "vector",
            "void use() { std::vector<decltype(&dart::makeA)> factories{&dart::makeA}; factories.front()().hit(); }",
        ),
        (
            "optional",
            "void use() { std::optional<decltype(&dart::makeA)> factory{&dart::makeA}; factory.value()().hit(); }",
        ),
        (
            "optional_array",
            "void use() { std::optional<std::array<decltype(&dart::makeA),1>> factories{std::in_place}; factories.value()[0]().hit(); }",
        ),
        (
            "array_optional",
            "void use() { std::array<std::optional<decltype(&dart::makeA)>,1> factories{}; factories[0].value()().hit(); }",
        ),
    ],
)
def test_function_reference_container_shapes(extractor, tmp_path, name, source):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(RECEIVER16_HEADER)
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0


@pytest.mark.parametrize(
    "extra,source",
    [
        (
            "using Value=decltype(makeA());",
            "void use() { dart::Value value=dart::makeA(); value.hit(); }",
        ),
        (
            "using Value=decltype(makeA()); using Other=Value;",
            "void use() { dart::Other value=dart::makeA(); value.hit(); }",
        ),
        (
            "namespace types { using Value=decltype(makeA()); }",
            "void use() { dart::types::Value value=dart::makeA(); value.hit(); }",
        ),
        (
            "using Value=decltype(shared());",
            "void use() { dart::Value value=dart::shared(); value.get()->hit(); }",
        ),
        (
            "using Factory=decltype(&makeA);",
            "void use() { dart::Factory factory=&dart::makeA; factory().hit(); }",
        ),
        (
            "using Value=decltype(makeA()); Value aliased();",
            "void use() { dart::aliased().hit(); }",
        ),
    ],
)
def test_api_decltype_aliases(extractor, tmp_path, extra, source):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(RECEIVER16_HEADER + "namespace dart {" + extra + "}")
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0


@pytest.mark.parametrize(
    "header,source,name",
    [
        (
            "namespace dart { struct A { void hit() const; }; A make(); A invoke(A(*)()); }",
            "void use() { auto factory=&dart::make; invoke(factory).hit(); }",
            "dart::invoke",
        ),
        (
            "namespace dart { struct A {}; int invoke(int(*)(A)); }",
            "int factory(dart::A); void use() { auto callback=&factory; invoke(callback); }",
            "dart::invoke",
        ),
        (
            "namespace dart { struct A {}; using Alias=A; int invoke(int(*)(Alias)); }",
            "int factory(dart::Alias); void use() { auto callback=&factory; invoke(callback); }",
            "dart::invoke",
        ),
    ],
)
def test_callable_associated_namespaces(extractor, tmp_path, header, source, name):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(header)
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert name in _entries(result, "free_functions")


@pytest.mark.parametrize(
    "source,reference",
    [
        ("void use() { auto value=dart::reference(); }", False),
        ("void use() { auto& value=dart::reference(); }", True),
        ("void use() { decltype(dart::reference()) value=dart::reference(); }", True),
        ("void use(dart::A& original) { decltype((original)) value=original; }", True),
        ("void use(dart::A& original) { decltype(auto) value=(original); }", True),
        ("void use() { auto factory=&dart::reference; auto value=factory(); }", False),
        (
            "void use() { auto factory=&dart::reference; decltype(auto) value=factory(); }",
            True,
        ),
        (
            "auto helper() { return dart::reference(); } void use() { auto value=helper(); }",
            False,
        ),
        (
            "decltype(auto) helper() { return dart::reference(); } void use() { decltype(helper()) value=helper(); }",
            True,
        ),
        (
            "using Ref=dart::A&; Ref helper(); void use() { decltype(helper()) value=helper(); }",
            True,
        ),
    ],
)
def test_reference_evidence_preserves_only_reference_storage(
    extractor, tmp_path, source, reference
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        RECEIVER16_HEADER.replace("struct A {", "struct A { int field;")
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    owners = {row["name"] for row in result["types"] if row.get("layout_declarations")}
    assert ("dart::A" in owners) == (not reference)


@pytest.mark.parametrize(
    "extra,source",
    [
        (
            "using Value=decltype(makeA());",
            "struct Foreign { void hit() const; }; Foreign makeA(); void use() { dart::Value value=dart::makeA(); value.hit(); }",
        ),
        (
            "using Value=decltype(pointer());",
            "void use() { dart::Value value=dart::pointer(); value->hit(); }",
        ),
        (
            "using Value=decltype(reference());",
            "void use() { dart::Value value=dart::reference(); value.hit(); }",
        ),
        (
            "using Factory=decltype(&makeA); namespace types { using Other=Factory; }",
            "void use() { dart::types::Other factory=&dart::makeA; factory().hit(); }",
        ),
    ],
)
def test_api_expression_alias_context_and_return_shape(
    extractor, tmp_path, extra, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        RECEIVER16_HEADER + "namespace dart {" + extra + "}"
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0


@pytest.mark.parametrize(
    "source",
    [
        "struct Foreign { void hit() const; }; Foreign foreign(); void use(dart::A* known) { std::array<decltype(&foreign),1> factories{{&foreign}}; factories[0]().hit(); }",
        "struct Foreign { void hit() const; }; Foreign foreign(); void use(dart::A* known) { std::optional<decltype(&foreign)> factory{&foreign}; factory.value()().hit(); }",
        "namespace foreign { struct A {}; int invoke(int(*)(A)); } int factory(foreign::A); void use(dart::A* known) { auto callback=&factory; invoke(callback); }",
    ],
)
def test_callable_composition_foreign_controls(extractor, tmp_path, source):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        RECEIVER16_HEADER + "namespace dart { int invoke(int(*)(A)); }"
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "hit" not in _entries(result, "members") and "dart::invoke" not in _entries(
        result, "free_functions"
    )


@pytest.mark.parametrize(
    "source",
    [
        "void use(dart::A& value) { static_assert(sizeof(&value)==sizeof(void*)); }",
        "void use(dart::A& value) { auto pointer=&value; static_assert(sizeof(pointer)==sizeof(void*)); }",
    ],
)
def test_builtin_address_only_preserves_pointer_layout(extractor, tmp_path, source):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { private: int field; }; }"
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert not any(row.get("layout_declarations") for row in before["types"])
    header.write_text(text.replace("int field", "double field"))
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) == before


def test_known_overloaded_address_preserves_declared_return_shape(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { private: int field; public: A operator&() const; }; }"
    header.write_text(text)
    source = "void use(dart::A& value) { static_assert(sizeof(&value)==sizeof(int)); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert any(row.get("layout_declarations") for row in before["types"])
    header.write_text(text.replace("int field", "double field"))
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) != before


def test_api_primitive_decltype_alias_suppresses_unrelated_receivers(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        RECEIVER16_HEADER
        + "namespace dart { int primitive(); using Value=decltype(primitive()); }"
    )
    source = "void use(dart::A* known) { dart::Value value=dart::primitive(); }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "hit" not in _entries(result, "members") and not any(
        row.get("layout_declarations") for row in result["types"]
    )


@pytest.mark.parametrize(
    "kind,owners",
    [
        ("std::tuple<A*>", set()),
        ("std::tuple<A&>", set()),
        ("std::pair<A*,A*>", set()),
        ("std::pair<A*,B>", {"dart::B"}),
        ("std::tuple<A*,B>", {"dart::B"}),
        ("std::array<A,2>", {"dart::A"}),
        ("std::array<A,0>", set()),
        ("std::optional<std::tuple<A*>>", set()),
        ("std::variant<A*,B>", {"dart::B"}),
    ],
)
def test_inferred_inline_wrapper_uses_exact_declared_storage_owners(
    extractor, tmp_path, kind, owners
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = (
        "namespace dart { struct A { private: int field; }; struct B { private: int field; }; "
        + kind
        + " make(); }"
    )
    header.write_text(text)
    source = "void use() { auto value=dart::make(); }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert {
        row["name"] for row in result["types"] if row.get("layout_declarations")
    } == owners


def test_source_inline_alias_returns_keep_defining_storage_context(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        "namespace dart { struct A { private: int field; }; struct B { private: int field; }; }"
    )
    source = "namespace provider { using Value=std::pair<dart::A*,dart::B>; Value make(); } void use() { auto value=provider::make(); }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert {
        row["name"] for row in result["types"] if row.get("layout_declarations")
    } == {"dart::B"}


@pytest.mark.parametrize(
    "replacement",
    [
        "A operator&() const noexcept = delete;",
        "private: A operator&() const noexcept;",
        "A operator&() noexcept;",
        "A operator&() const noexcept(false);",
    ],
)
def test_unary_overloaded_address_retains_used_declarations(
    extractor, tmp_path, replacement
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { private: int field; public: A operator&() const noexcept; }; }"
    header.write_text(text)
    source = "void use(const dart::A& value) { static_assert(sizeof(&value)==sizeof(int)); static_assert(noexcept(&value)); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(before, "members")["operator&"]["receiver_types"] == ["dart::A"]
    header.write_text(text.replace("A operator&() const noexcept;", replacement))
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) != before


@pytest.mark.parametrize(
    "source,reference",
    [
        (
            "void use(dart::Holder& source) { decltype((source.item)) value=source.item; value.hit(); }",
            True,
        ),
        (
            "void use(dart::Holder* source) { decltype((source->item)) value=source->item; value.hit(); }",
            True,
        ),
        (
            "void use() { decltype((dart::holder().item)) value=dart::holder().item; value.hit(); }",
            True,
        ),
        (
            "void use(dart::Holder& source) { decltype(auto) value=(source.item); value.hit(); }",
            True,
        ),
        (
            "void use(dart::Holder& source) { decltype(source.item) value=source.item; value.hit(); }",
            False,
        ),
    ],
)
def test_decltype_member_lvalue_reference_and_value_controls(
    extractor, tmp_path, source, reference
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { private: int field; public: void hit() const; }; struct Holder { A item; }; Holder& holder(); }"
    header.write_text(text)
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert _entries(result, "members")["hit"]["receiver_types"] == ["dart::A"]
    assert any(row.get("layout_declarations") for row in result["types"]) == (
        not reference
    )
    header.write_text(text.replace("int field", "double field"))
    assert (
        _scan(extractor, extractor.index_api(headers), tmp_path, source) != result
    ) == (not reference)


@pytest.mark.parametrize(
    "files,hit_count",
    [
        (
            {
                "client.cc": "namespace provider { inline auto factory=&dart::makeA; } void use() { provider::factory().hit(); }"
            },
            1,
        ),
        (
            {
                "helper.hh": "namespace provider { inline auto factory=&dart::makeA; }",
                "client.cc": '#include "helper.hh"\nvoid use() { provider::factory().hit(); }',
            },
            1,
        ),
        (
            {
                "helper.hh": "namespace provider { inline auto factory=&dart::makeA; }",
                "client.cc": '#include "helper.hh"\nusing provider::factory; void use() { factory().hit(); }',
            },
            1,
        ),
        (
            {
                "helper.hh": "namespace provider { inline auto factory=&dart::makeA; }",
                "client.cc": '#include "helper.hh"\nnamespace api=provider; void use() { api::factory().hit(); }',
            },
            1,
        ),
        (
            {
                "helper.hh": "namespace provider { inline auto factory=&dart::makeA; }",
                "client.cc": '#include "helper.hh"\nusing namespace provider; void use() { factory().hit(); }',
            },
            1,
        ),
        (
            {
                "helper.hh": "namespace provider { inline auto factory=&dart::makeA; }",
                "client.cc": '#include "helper.hh"\nstruct Foreign { void hit() const; }; Foreign foreign(); using provider::factory; void use() { auto factory=&foreign; factory().hit(); provider::factory().hit(); }',
            },
            1,
        ),
        (
            {
                "unused.hh": "namespace provider { inline auto factory=&dart::makeA; }",
                "client.cc": "struct Foreign { void hit() const; }; namespace provider { Foreign factory(); } void use() { provider::factory().hit(); }",
            },
            0,
        ),
        (
            {
                "helper.hh": "namespace provider { inline auto factory=&dart::makeA; }",
                "client.cc": '#include "helper.hh"\nstruct Foreign { void hit() const; }; Foreign foreign(); using provider::factory; void use() { auto factory=&foreign; factory().hit(); }',
            },
            0,
        ),
    ],
)
def test_namespace_inferred_callable_visibility(extractor, tmp_path, files, hit_count):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(RECEIVER16_HEADER)
    before = _scan_files(
        extractor, extractor.index_api(headers), tmp_path / "before", files
    )
    hit = _entries(before, "members").get("hit")
    assert (hit["count"] if hit else 0) == hit_count
    if hit:
        assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0
    header.write_text(
        RECEIVER16_HEADER.replace("void hit() const;", "void hit() const = delete;")
    )
    assert (
        _scan_files(extractor, extractor.index_api(headers), tmp_path / "after", files)
        != before
    ) == bool(hit_count)


def test_binary_member_address_operator_does_not_replace_builtin_unary_address(
    extractor, tmp_path
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = "namespace dart { struct A { private: int field; public: A operator&(const A&) const; }; }"
    header.write_text(text)
    source = "void use(dart::A& value) { auto pointer=&value; static_assert(sizeof(pointer)==sizeof(void*)); }"
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "operator&" not in _entries(before, "members") and not any(
        row.get("layout_declarations") for row in before["types"]
    )
    header.write_text(text.replace("int field", "double field"))
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) == before


LAYOUT_CASES = [
    (
        "sizeof_type",
        "namespace dart { class A { int value; }; }",
        "namespace dart { class A { double value; }; }",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "by_value_local",
        "namespace dart { class A { int value; }; }",
        "namespace dart { class A { double value; }; }",
        "void use(){dart::A value{}; static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "by_value_parameter",
        "namespace dart { class A { int value; }; }",
        "namespace dart { class A { double value; }; }",
        "void use(dart::A value){static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "by_value_return",
        "namespace dart { class A { int value; }; }",
        "namespace dart { class A { double value; }; }",
        "dart::A make(); static_assert(sizeof(make())==sizeof(int));",
        True,
    ),
    (
        "sizeof_dereference",
        "namespace dart { class A { int value; }; }",
        "namespace dart { class A { double value; }; }",
        "void use(dart::A* p){static_assert(sizeof(*p)==sizeof(int));}",
        True,
    ),
    (
        "pointer_only",
        "namespace dart { class A { int value; }; }",
        "namespace dart { class A { double value; }; }",
        "void use(dart::A* p);",
        False,
    ),
    (
        "reference_only",
        "namespace dart { class A { int value; }; }",
        "namespace dart { class A { double value; }; }",
        "void use(const dart::A& a);",
        False,
    ),
    (
        "sizeof_pointer",
        "namespace dart { class A { int value; }; }",
        "namespace dart { class A { double value; }; }",
        "static_assert(sizeof(dart::A*)==sizeof(void*));",
        False,
    ),
    (
        "api_pointer_alias",
        "namespace dart { class A { int value; }; using Pointer=A*; }",
        "namespace dart { class A { double value; }; using Pointer=A*; }",
        "static_assert(sizeof(dart::Pointer)==sizeof(void*));",
        False,
    ),
    (
        "source_reference_alias",
        "namespace dart { class A { int value; }; }",
        "namespace dart { class A { double value; }; }",
        "using Ref=const dart::A&; void use(Ref a);",
        False,
    ),
    (
        "base_private_field",
        "namespace dart { class B { int value; }; class A: public B {}; }",
        "namespace dart { class B { double value; }; class A: public B {}; }",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "owned_private_field",
        "namespace dart { class B { int value; }; class A { B owned; }; }",
        "namespace dart { class B { double value; }; class A { B owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "raw_array_field",
        "namespace dart { class B { int value; }; class A { B owned[2]; }; }",
        "namespace dart { class B { double value; }; class A { B owned[2]; }; }",
        "static_assert(sizeof(dart::A)==2*sizeof(int));",
        True,
    ),
    (
        "array_field",
        "namespace dart { class B { int value; }; class A { std::array<B,2> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::array<B,2> owned; }; }",
        "static_assert(sizeof(dart::A)==2*sizeof(int));",
        True,
    ),
    (
        "array_zero_field",
        "namespace dart { class B { int value; }; class A { std::array<B,0> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::array<B,0> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(std::array<int,0>));",
        False,
    ),
    (
        "array_zero_paren_field",
        "namespace dart { class B { int value; }; class A { std::array<B,(0)> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::array<B,(0)> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(std::array<int,0>));",
        False,
    ),
    (
        "tuple_field",
        "namespace dart { class B { int value; }; class A { std::tuple<B> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::tuple<B> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "pair_field",
        "namespace dart { class B { int value; }; class A { std::pair<B,B> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::pair<B,B> owned; }; }",
        "static_assert(sizeof(dart::A)==2*sizeof(int));",
        True,
    ),
    (
        "optional_field",
        "namespace dart { class B { int value; }; class A { std::optional<B> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::optional<B> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(std::optional<int>));",
        True,
    ),
    (
        "variant_field",
        "namespace dart { class B { int value; }; class A { std::variant<B> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::variant<B> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(std::variant<int>));",
        True,
    ),
    (
        "pointer_field",
        "namespace dart { class B { int value; }; class A { B* owned; }; }",
        "namespace dart { class B { double value; }; class A { B* owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "ref_field",
        "namespace dart { class B { int value; }; class A { B& owned; }; }",
        "namespace dart { class B { double value; }; class A { B& owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "tuple_ref_field",
        "namespace dart { class B { int value; }; class A { std::tuple<B&> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::tuple<B&> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(std::tuple<int&>));",
        False,
    ),
    (
        "optional_pointer_field",
        "namespace dart { class B { int value; }; class A { std::optional<B*> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::optional<B*> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(std::optional<void*>));",
        False,
    ),
    (
        "variant_pointer_field",
        "namespace dart { class B { int value; }; class A { std::variant<B*> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::variant<B*> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(std::variant<void*>));",
        False,
    ),
    (
        "vector_field",
        "namespace dart { class B { int value; }; class A { std::vector<B> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::vector<B> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(std::vector<int>));",
        False,
    ),
    (
        "smart_field",
        "namespace dart { class B { int value; }; class A { std::shared_ptr<B> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::shared_ptr<B> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(std::shared_ptr<int>));",
        False,
    ),
    (
        "unique_field",
        "namespace dart { class B { int value; }; class A { std::unique_ptr<B> owned; }; }",
        "namespace dart { class B { double value; }; class A { std::unique_ptr<B> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(std::unique_ptr<int>));",
        False,
    ),
    (
        "static_field",
        "namespace dart { class B { int value; }; class A { inline static B owned; }; }",
        "namespace dart { class B { double value; }; class A { inline static B owned; }; }",
        "static_assert(sizeof(dart::A)==1);",
        False,
    ),
    (
        "foreign_container_field",
        "namespace dart { class B { int value; }; template<class T> class Foreign { T* pointer; }; "
        "class A { Foreign<B> owned; }; }",
        "namespace dart { class B { double value; }; template<class T> class Foreign { T* pointer; "
        "}; class A { Foreign<B> owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "api_value_alias",
        "namespace dart { class B { int value; }; using Value=B; class A { Value owned; }; }",
        "namespace dart { class B { double value; }; using Value=B; class A { Value owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "api_pointer_alias_field",
        "namespace dart { class B { int value; }; using Pointer=B*; class A { Pointer owned; }; }",
        "namespace dart { class B { double value; }; using Pointer=B*; class A { Pointer owned; }; "
        "}",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "api_ref_alias_field",
        "namespace dart { class B { int value; }; using Ref=B&; class A { Ref owned; }; }",
        "namespace dart { class B { double value; }; using Ref=B&; class A { Ref owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "api_alias_chain",
        "namespace dart { class B { int value; }; using Value=B;using Next=Value; class A { Next "
        "owned; }; }",
        "namespace dart { class B { double value; }; using Value=B;using Next=Value; class A { Next "
        "owned; }; }",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "source_value_alias",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "using Value=dart::B;static_assert(sizeof(Value)==sizeof(int));",
        True,
    ),
    (
        "source_array_alias",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "using Value=std::array<dart::B,2>;static_assert(sizeof(Value)==2*sizeof(int));",
        True,
    ),
    (
        "source_zero_array_alias",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "using Value=std::array<dart::B,0>;static_assert(sizeof(Value)==sizeof(std::array<int,0>));",
        False,
    ),
    (
        "source_tuple_ref_alias",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "using Ref=dart::B&;using "
        "Value=std::tuple<Ref>;static_assert(sizeof(Value)==sizeof(std::tuple<int&>));",
        False,
    ),
    (
        "source_alias_chain",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "using Value=dart::B;using Next=Value;static_assert(sizeof(Next)==sizeof(int));",
        True,
    ),
    (
        "api_ref_sizeof",
        "namespace dart { class B { int value; }; using Ref=B&;}",
        "namespace dart { class B { double value; }; using Ref=B&;}",
        "static_assert(sizeof(dart::Ref)==sizeof(int));",
        True,
    ),
    (
        "source_ref_sizeof",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "using Ref=dart::B&;static_assert(sizeof(Ref)==sizeof(int));",
        True,
    ),
    (
        "source_pointer_alias",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "using Pointer=dart::B*;static_assert(sizeof(Pointer)==sizeof(void*));",
        False,
    ),
    (
        "function_pointer_alias",
        "namespace dart { class B { int value; }; using Callback=B(*)();}",
        "namespace dart { class B { double value; }; using Callback=B(*)();}",
        "static_assert(sizeof(dart::Callback)==sizeof(void*));",
        False,
    ),
    (
        "smart_sizeof_pointee",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(std::unique_ptr<dart::B>& p){static_assert(sizeof(*p)==sizeof(int));}",
        True,
    ),
    (
        "smart_sizeof_wrapper",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(std::unique_ptr<dart::B>& "
        "p){static_assert(sizeof(p)==sizeof(std::unique_ptr<int>));}",
        False,
    ),
    (
        "vector_local",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(){std::vector<dart::B> "
        "values;static_assert(sizeof(values)==sizeof(std::vector<int>));}",
        False,
    ),
    (
        "zero_array_local",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(){std::array<dart::B,0> "
        "values;static_assert(sizeof(values)==sizeof(std::array<int,0>));}",
        False,
    ),
    (
        "field_order",
        "namespace dart { class A { int x;double y;char z; }; }",
        "namespace dart { class A { double y;int x;char z; }; }",
        "static_assert(sizeof(dart::A)==24);",
        True,
    ),
    (
        "base_order",
        "namespace dart { class X {int x;};class Y {double y;};class A: X,Y {char z;}; }",
        "namespace dart { class X {int x;};class Y {double y;};class A: Y,X {char z;}; }",
        "static_assert(sizeof(dart::A)==24);",
        True,
    ),
    (
        "bit_width_constant",
        "namespace dart { constexpr unsigned W=63; class A { unsigned long long x:W; unsigned long "
        "long y:1; }; }",
        "namespace dart { constexpr unsigned W=64; class A { unsigned long long x:W; unsigned long "
        "long y:1; }; }",
        "static_assert(sizeof(dart::A)==8);",
        True,
    ),
    (
        "group_pointer_then_value",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(){dart::B *pointer=nullptr, value;static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "group_reference_then_value",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(dart::B& base){dart::B &reference=base, "
        "value;static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "group_all_pointers",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(){dart::B *pointer=nullptr, "
        "*other=nullptr;static_assert(sizeof(other)==sizeof(void*));}",
        False,
    ),
    (
        "api_value_alias_outer_reference",
        "namespace dart { class B { int value; }; using Value=B;}",
        "namespace dart { class B { double value; }; using Value=B;}",
        "void use(dart::Value& value);",
        False,
    ),
    (
        "source_alias_outer_reference",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "using Value=dart::B;void use(Value& value);",
        False,
    ),
    (
        "source_helper_value_alias",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "using Value=dart::B;Value make();void use(){auto "
        "value=make();static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "auto_factory_value",
        "namespace dart { class B { int value; }; B make();}",
        "namespace dart { class B { double value; }; B make();}",
        "void use(){auto value=dart::make();static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "decltype_factory_value",
        "namespace dart { class B { int value; }; B make();}",
        "namespace dart { class B { double value; }; B make();}",
        "void use(){decltype(dart::make()) "
        "value=dart::make();static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "callable_factory_value",
        "namespace dart { class B { int value; }; struct Factory { B operator()(); };}",
        "namespace dart { class B { double value; }; struct Factory { B operator()(); };}",
        "void use(){dart::Factory factory;auto "
        "value=factory();static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "source_helper_ref_alias",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "using Ref=dart::B&;Ref make();void use(){decltype(make()) reference=make();}",
        False,
    ),
    (
        "auto_reference_copy",
        "namespace dart { class B { int value; }; B& make();}",
        "namespace dart { class B { double value; }; B& make();}",
        "void use(){auto value=dart::make();static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "auto_reference_binding",
        "namespace dart { class B { int value; }; B& make();}",
        "namespace dart { class B { double value; }; B& make();}",
        "void use(){auto& reference=dart::make();}",
        False,
    ),
    (
        "auto_const_reference_binding",
        "namespace dart { class B { int value; }; B& make();}",
        "namespace dart { class B { double value; }; B& make();}",
        "void use(){const auto& reference=dart::make();}",
        False,
    ),
    (
        "auto_rvalue_reference_binding",
        "namespace dart { class B { int value; }; B make();}",
        "namespace dart { class B { double value; }; B make();}",
        "void use(){auto&& reference=dart::make();}",
        False,
    ),
    (
        "decltype_reference_return",
        "namespace dart { class B { int value; }; B& make();}",
        "namespace dart { class B { double value; }; B& make();}",
        "void use(){decltype(dart::make()) reference=dart::make();}",
        False,
    ),
    (
        "decltype_grouped_local",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(dart::B& existing){decltype((existing)) reference=existing;}",
        False,
    ),
    (
        "decltype_auto_reference_return",
        "namespace dart { class B { int value; }; B& make();}",
        "namespace dart { class B { double value; }; B& make();}",
        "void use(){decltype(auto) reference=dart::make();}",
        False,
    ),
    (
        "auto_pointer_factory",
        "namespace dart { class B { int value; }; B* make();}",
        "namespace dart { class B { double value; }; B* make();}",
        "void use(){auto pointer=dart::make();static_assert(sizeof(pointer)==sizeof(void*));}",
        False,
    ),
    (
        "decltype_pointer_factory",
        "namespace dart { class B { int value; }; B* make();}",
        "namespace dart { class B { double value; }; B* make();}",
        "void use(){decltype(dart::make()) "
        "pointer=dart::make();static_assert(sizeof(pointer)==sizeof(void*));}",
        False,
    ),
    (
        "sizeof_reference_factory",
        "namespace dart { class B { int value; }; B& make();}",
        "namespace dart { class B { double value; }; B& make();}",
        "static_assert(sizeof(dart::make())==sizeof(int));",
        True,
    ),
    (
        "sizeof_temporary_pointer_field",
        "namespace dart { class A { int value;public:A* pointer; };}",
        "namespace dart { class A { double value;public:A* pointer; };}",
        "static_assert(sizeof(dart::A().pointer)==sizeof(void*));",
        False,
    ),
    (
        "primitive_alias_field",
        "namespace dart {using Scalar=int;class A{Scalar value;};}",
        "namespace dart {using Scalar=double;class A{Scalar value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "primitive_alias_chain_field",
        "namespace dart {using Scalar=int;using Next=Scalar;class A{Next value;};}",
        "namespace dart {using Scalar=double;using Next=Scalar;class A{Next value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "enum_field",
        "namespace dart {enum class E:int{};class A{E value;};}",
        "namespace dart {enum class E:long long{};class A{E value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "enum_alias_field",
        "namespace dart {enum class E:int{};using Value=E;class A{Value value;};}",
        "namespace dart {enum class E:long long{};using Value=E;class A{Value value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "enum_alias_chain_field",
        "namespace dart {enum class E:int{};using Value=E;using Next=Value;class A{Next value;};}",
        "namespace dart {enum class E:long long{};using Value=E;using Next=Value;class A{Next "
        "value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "enum_pointer_field",
        "namespace dart {enum class E:int{};class A{E* value;};}",
        "namespace dart {enum class E:long long{};class A{E* value;};}",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "enum_ref_field",
        "namespace dart {enum class E:int{};class A{E& value;};}",
        "namespace dart {enum class E:long long{};class A{E& value;};}",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "primitive_pointer_alias_field",
        "namespace dart {using Scalar=int*;class A{Scalar value;};}",
        "namespace dart {using Scalar=double*;class A{Scalar value;};}",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "primitive_pointer_alias_chain",
        "namespace dart {using Scalar=int*;using Next=Scalar;class A{Next value;};}",
        "namespace dart {using Scalar=double*;using Next=Scalar;class A{Next value;};}",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "bare_sizeof_dereference",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(dart::B* pointer){static_assert(sizeof *pointer==sizeof(int));}",
        True,
    ),
    (
        "bare_sizeof_reference",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(dart::B& value){static_assert(sizeof value==sizeof(int));}",
        True,
    ),
    (
        "bare_sizeof_pointer_factory",
        "namespace dart { class B { int value; }; B* pointer();}",
        "namespace dart { class B { double value; }; B* pointer();}",
        "static_assert(sizeof *dart::pointer()==sizeof(int));",
        True,
    ),
    (
        "bare_sizeof_pointer",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(dart::B* pointer){static_assert(sizeof pointer==sizeof(void*));}",
        False,
    ),
    (
        "bare_sizeof_less_comparison",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(dart::B* pointer){static_assert(sizeof *pointer < 5);}",
        True,
    ),
    (
        "inherited_primitive_alias",
        "namespace dart {struct Base {using Scalar=int;};class A:Base{Scalar value;};}",
        "namespace dart {struct Base {using Scalar=double;};class A:Base{Scalar value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "inherited_primitive_alias_chain",
        "namespace dart {struct Base {using Scalar=int;using Next=Scalar;};class A:Base{Next "
        "value;};}",
        "namespace dart {struct Base {using Scalar=double;using Next=Scalar;};class A:Base{Next "
        "value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "array_indexed_extent",
        "namespace dart {constexpr unsigned N=2;class A{std::array<int,N> value;};}",
        "namespace dart {constexpr unsigned N=3;class A{std::array<int,N> value;};}",
        "static_assert(sizeof(dart::A)==2*sizeof(int));",
        True,
    ),
    (
        "vector_nested_array_extent",
        "namespace dart {constexpr unsigned N=2;class A{std::vector<std::array<int,N>> value;};}",
        "namespace dart {constexpr unsigned N=3;class A{std::vector<std::array<int,N>> value;};}",
        "static_assert(sizeof(dart::A)==sizeof(std::vector<int>));",
        False,
    ),
    (
        "smart_nested_array_extent",
        "namespace dart {constexpr unsigned N=2;class A{std::unique_ptr<std::array<int,N>> "
        "value;};}",
        "namespace dart {constexpr unsigned N=3;class A{std::unique_ptr<std::array<int,N>> "
        "value;};}",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "quoted_array_alias_field",
        "namespace dart {using Text=char['a'];class A{Text value;};}",
        "namespace dart {using Text=char['b'];class A{Text value;};}",
        "static_assert(sizeof(dart::A)==97);",
        True,
    ),
    (
        "quoted_array_alias_chain",
        "namespace dart {using Text=char['a'];using Next=Text;class A{Next value;};}",
        "namespace dart {using Text=char['b'];using Next=Text;class A{Next value;};}",
        "static_assert(sizeof(dart::A)==97);",
        True,
    ),
    (
        "quoted_alias_comment",
        "namespace dart {using Text=char['a'/*one*/];class A{Text value;};}",
        "namespace dart {using Text=char['a'/*two*/];class A{Text value;};}",
        "static_assert(sizeof(dart::A)==97);",
        False,
    ),
    (
        "quoted_alias_pointer",
        "namespace dart {using Text=char(*)['a'];class A{Text value;};}",
        "namespace dart {using Text=char(*)['b'];class A{Text value;};}",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "quoted_alias_unused",
        "namespace dart {using Text=char['a'];class A{int value;};}",
        "namespace dart {using Text=char['b'];class A{int value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        False,
    ),
    (
        "auto_factory_storage",
        "namespace dart { class B { int value; }; B make();}",
        "namespace dart { class B { double value; }; B make();}",
        "void use(){auto value=dart::make();}",
        True,
    ),
    (
        "auto_reference_copy_storage",
        "namespace dart { class B { int value; }; B& make();}",
        "namespace dart { class B { double value; }; B& make();}",
        "void use(){auto value=dart::make();}",
        True,
    ),
    (
        "decltype_factory_storage",
        "namespace dart { class B { int value; }; B make();}",
        "namespace dart { class B { double value; }; B make();}",
        "void use(){decltype(dart::make()) value=dart::make();}",
        True,
    ),
    (
        "decltype_auto_factory_storage",
        "namespace dart { class B { int value; }; B make();}",
        "namespace dart { class B { double value; }; B make();}",
        "void use(){decltype(auto) value=dart::make();}",
        True,
    ),
    (
        "init_capture_value_layout",
        "namespace dart { class B { int value; }; B make();}",
        "namespace dart { class B { double value; }; B make();}",
        "void use(){auto function=[value=dart::make()]{return "
        "value;};static_assert(sizeof(function)==sizeof(int));}",
        True,
    ),
    (
        "init_capture_reference_copy_layout",
        "namespace dart { class B { int value; }; B& make();}",
        "namespace dart { class B { double value; }; B& make();}",
        "void use(){auto function=[value=dart::make()]{return "
        "value;};static_assert(sizeof(function)==sizeof(int));}",
        True,
    ),
    (
        "init_capture_reference_layout",
        "namespace dart { class B { int value; }; B& make();}",
        "namespace dart { class B { double value; }; B& make();}",
        "void use(){auto function=[&value=dart::make()]{return "
        "value;};static_assert(sizeof(function)==sizeof(void*));}",
        False,
    ),
    (
        "api_reference_alias_object_sizeof",
        "namespace dart { class B { int value; }; using Ref=B&;}",
        "namespace dart { class B { double value; }; using Ref=B&;}",
        "void use(dart::Ref value){static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "api_reference_alias_factory_sizeof",
        "namespace dart { class B { int value; }; using Ref=B&;Ref make();}",
        "namespace dart { class B { double value; }; using Ref=B&;Ref make();}",
        "static_assert(sizeof(dart::make())==sizeof(int));",
        True,
    ),
    (
        "api_reference_alias_auto_copy",
        "namespace dart { class B { int value; }; using Ref=B&;Ref make();}",
        "namespace dart { class B { double value; }; using Ref=B&;Ref make();}",
        "void use(){auto value=dart::make();static_assert(sizeof(value)==sizeof(int));}",
        True,
    ),
    (
        "sizeof_address_of_reference",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(dart::B& value){static_assert(sizeof(&value)==sizeof(void*));}",
        False,
    ),
    (
        "auto_address_of_reference",
        "namespace dart { class B { int value; }; }",
        "namespace dart { class B { double value; }; }",
        "void use(dart::B& value){auto "
        "pointer=&value;static_assert(sizeof(pointer)==sizeof(void*));}",
        False,
    ),
    (
        "sizeof_overloaded_address_value",
        "namespace dart {class A{int value;public:A operator&();};}",
        "namespace dart {class A{double value;public:A operator&();};}",
        "void use(dart::A& value){static_assert(sizeof(&value)==sizeof(int));}",
        True,
    ),
    (
        "sizeof_overloaded_address_pointer",
        "namespace dart {class A{int value;public:A* operator&();};}",
        "namespace dart {class A{double value;public:A* operator&();};}",
        "void use(dart::A& value){static_assert(sizeof(&value)==sizeof(void*));}",
        False,
    ),
    (
        "tuple_pointer_factory",
        "namespace dart { class B { int value; }; std::tuple<B*> make();}",
        "namespace dart { class B { double value; }; std::tuple<B*> make();}",
        "void use(){auto "
        "value=dart::make();static_assert(sizeof(value)==sizeof(std::tuple<void*>));}",
        False,
    ),
    (
        "tuple_ref_factory",
        "namespace dart { class B { int value; }; std::tuple<B&> make();}",
        "namespace dart { class B { double value; }; std::tuple<B&> make();}",
        "void use(){auto "
        "value=dart::make();static_assert(sizeof(value)==sizeof(std::tuple<int&>));}",
        False,
    ),
    (
        "pair_pointer_factory",
        "namespace dart { class B { int value; }; std::pair<B*,B*> make();}",
        "namespace dart { class B { double value; }; std::pair<B*,B*> make();}",
        "void use(){auto "
        "value=dart::make();static_assert(sizeof(value)==sizeof(std::pair<void*,void*>));}",
        False,
    ),
    (
        "mixed_pair_pointer_factory",
        "namespace dart { class B { int value; }; std::pair<B*,int> make();}",
        "namespace dart { class B { double value; }; std::pair<B*,int> make();}",
        "void use(){auto "
        "value=dart::make();static_assert(sizeof(value)==sizeof(std::pair<void*,int>));}",
        False,
    ),
    (
        "tuple_value_factory",
        "namespace dart { class B { int value; }; std::tuple<B> make();}",
        "namespace dart { class B { double value; }; std::tuple<B> make();}",
        "void use(){auto value=dart::make();static_assert(sizeof(value)==sizeof(std::tuple<int>));}",
        True,
    ),
    (
        "array_value_factory",
        "namespace dart { class B { int value; }; std::array<B,2> make();}",
        "namespace dart { class B { double value; }; std::array<B,2> make();}",
        "void use(){auto value=dart::make();static_assert(sizeof(value)==2*sizeof(int));}",
        True,
    ),
    (
        "array_zero_factory",
        "namespace dart { class B { int value; }; std::array<B,0> make();}",
        "namespace dart { class B { double value; }; std::array<B,0> make();}",
        "void use(){auto "
        "value=dart::make();static_assert(sizeof(value)==sizeof(std::array<int,0>));}",
        False,
    ),
    (
        "decltype_literal_array_field",
        "namespace dart {class A{decltype(0) value;};}",
        "namespace dart {class A{decltype(0) value[2];};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "decltype_free_callee_field",
        "namespace dart {int choose();class A{decltype(choose()) value;};}",
        "namespace dart {double choose();class A{decltype(choose()) value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "decltype_static_callee_field",
        "namespace dart {struct A{static int choose();decltype(choose()) value;};}",
        "namespace dart {struct A{static double choose();decltype(choose()) value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "decltype_callee_owned_field",
        "namespace dart { class B { int value; }; B choose();class A{decltype(choose()) value;};}",
        "namespace dart { class B { double value; }; B choose();class A{decltype(choose()) "
        "value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "decltype_callee_pointer_field",
        "namespace dart { class B { int value; }; B* choose();class A{decltype(choose()) value;};}",
        "namespace dart { class B { double value; }; B* choose();class A{decltype(choose()) "
        "value;};}",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "decltype_callee_reference_field",
        "namespace dart { class B { int value; }; B& choose();class A{decltype(choose()) value;};}",
        "namespace dart { class B { double value; }; B& choose();class A{decltype(choose()) "
        "value;};}",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "ordinary_function_not_field",
        "namespace dart {class A{int choose();};}",
        "namespace dart {class A{double choose();};}",
        "static_assert(sizeof(dart::A)==1);",
        False,
    ),
    (
        "decltype_function_not_field",
        "namespace dart {class A{decltype(0) choose();};}",
        "namespace dart {class A{decltype(0.0) choose();};}",
        "static_assert(sizeof(dart::A)==1);",
        False,
    ),
    (
        "callback_not_field",
        "namespace dart {class A{int(*callback)();};}",
        "namespace dart {class A{double(*callback)();};}",
        "static_assert(sizeof(dart::A)==sizeof(void*));",
        False,
    ),
    (
        "decltype_private_static_callee",
        "namespace dart {class A{static int choose();decltype(choose()) value;};}",
        "namespace dart {class A{static double choose();decltype(choose()) value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "decltype_base_private_callee",
        "namespace dart {class Base{static int choose();decltype(choose()) value;};class A:Base{};}",
        "namespace dart {class Base{static double choose();decltype(choose()) value;};class "
        "A:Base{};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "decltype_inherited_private_friend_callee",
        "namespace dart {class Base{friend class A;static int choose();};class "
        "A:Base{decltype(choose()) value;};}",
        "namespace dart {class Base{friend class A;static double choose();};class "
        "A:Base{decltype(choose()) value;};}",
        "static_assert(sizeof(dart::A)==sizeof(int));",
        True,
    ),
    (
        "unnamed_padding_field",
        "namespace dart {class A{unsigned int :8;char field;};}",
        "namespace dart {class A{unsigned int :16;char field;};}",
        "static_assert(sizeof(dart::A)==2);",
        True,
    ),
    (
        "unnamed_only_field",
        "namespace dart {class A{unsigned int :8;};}",
        "namespace dart {class A{unsigned int :16;};}",
        "static_assert(sizeof(dart::A)==1);",
        True,
    ),
    (
        "unnamed_padding_constant",
        "namespace dart {constexpr unsigned W=8;class A{unsigned int :W;char field;};}",
        "namespace dart {constexpr unsigned W=16;class A{unsigned int :W;char field;};}",
        "static_assert(sizeof(dart::A)==2);",
        True,
    ),
    (
        "unnamed_zero_padding",
        "namespace dart {class A{unsigned int x:1;unsigned int :0;unsigned int y:1;};}",
        "namespace dart {class A{unsigned int x:1;unsigned int :1;unsigned int y:1;};}",
        "static_assert(sizeof(dart::A)==8);",
        True,
    ),
    (
        "unnamed_padding_aggregate",
        "namespace dart {struct A{unsigned int :8;char field;};}",
        "namespace dart {struct A{unsigned int :16;char field;};}",
        "void use(){dart::A value{'x'};static_assert(sizeof(value)==2);}",
        True,
    ),
]


@pytest.mark.parametrize(
    "name,before_header,after_header,source,expected",
    LAYOUT_CASES,
    ids=[case[0] for case in LAYOUT_CASES],
)
def test_layout_storage_declarations(
    extractor, tmp_path, name, before_header, after_header, source, expected
):
    before = _scan_receiver_api(extractor, tmp_path, before_header, source)
    after = _scan_receiver_api(extractor, tmp_path, after_header, source)
    assert (before != after) is expected


def test_forward_class_layout_omits_identity_alias_evidence(extractor, tmp_path):
    header = """namespace dart {
class A;
class A { int value; };
using Value = A;
class Holder { A direct; Value aliased; A* pointer; };
}"""
    inventory = _scan_receiver_api(
        extractor, tmp_path, header, "static_assert(sizeof(dart::Holder)>sizeof(int));"
    )
    declarations = _entries(inventory, "types")["dart::Holder"]["layout_declarations"]
    aliases = [
        alias
        for declaration in declarations
        for alias in declaration.get("type_alias_declarations", ())
    ]
    assert aliases == [{"class": "dart::Value", "signature": "using dart::Value = A"}]
    assert any(
        declaration["class"] == "dart::A" and declaration["signature"] == "int value"
        for declaration in declarations
    )


ROUND17_ROOT_CASES = []


def _union(name, header, old, new, source, changed=True, breaking=None):
    ROUND17_ROOT_CASES.append(
        (
            name,
            header,
            header.replace(old, new),
            source,
            changed,
            changed if breaking is None else breaking,
        )
    )


U = "namespace dart { union Value { int bits; double real; }; }"
_union(
    "union_member",
    U,
    "int bits",
    "int gone",
    "void use(){dart::Value value;value.bits=1;}",
)
_union(
    "union_arrow",
    U,
    "int bits",
    "int gone",
    "void use(dart::Value* value){value->bits=1;}",
)
_union(
    "union_value_receiver",
    "namespace dart { union Value { int bits; void hit(); }; Value make(); }",
    "void hit();",
    "void hit()=delete;",
    "void use(){dart::make().hit();}",
)
_union(
    "union_pointer_receiver",
    "namespace dart { union Value { int bits; void hit(); }; Value* make(); }",
    "void hit();",
    "void hit()=delete;",
    "void use(){dart::make()->hit();}",
)
_union(
    "union_alias",
    U + " namespace dart { using Alias=Value; }",
    "int bits",
    "int gone",
    "void use(){dart::Alias value;value.bits=1;}",
)
_union(
    "union_forward",
    "namespace dart { union Value; union Value { int bits; double real; }; }",
    "int bits",
    "int gone",
    "void use(){dart::Value value;value.bits=1;}",
)
_union(
    "union_nested",
    "namespace dart { struct Holder { union Value { int bits; double real; }; }; }",
    "int bits",
    "int gone",
    "void use(){dart::Holder::Value value;value.bits=1;}",
)
_union(
    "union_access",
    "namespace dart { union Value { int bits; }; }",
    "int bits;",
    "private: int bits;",
    "void use(){dart::Value value;value.bits=1;}",
)
_union(
    "union_ctor",
    "namespace dart { union Value { int bits; Value(); }; }",
    "Value();",
    "Value()=delete;",
    "void use(){dart::Value value;}",
)
_union(
    "union_template",
    "namespace dart { template<class T> union Value { T bits; double real; }; }",
    "T bits",
    "T gone",
    "void use(){dart::Value<int> value;value.bits=1;}",
)
_union(
    "union_alignment",
    "namespace dart { union alignas(16) Value { int bits; double real; }; }",
    "alignas(16)",
    "alignas(8)",
    "static_assert(alignof(dart::Value)==16);",
)
_union(
    "union_storage_kind",
    U,
    "union Value",
    "struct Value",
    "static_assert(sizeof(dart::Value)==sizeof(double));",
)
_union(
    "union_owned_storage_kind",
    U + " namespace dart { struct Holder { Value value; }; }",
    "union Value",
    "struct Value",
    "static_assert(sizeof(dart::Holder)==sizeof(double));",
)
_union(
    "union_owned_alias_storage_kind",
    U + " namespace dart { using Alias=Value; struct Holder { Alias value; }; }",
    "union Value",
    "struct Value",
    "static_assert(sizeof(dart::Holder)==sizeof(double));",
)
_union(
    "anonymous_union_member",
    "namespace dart { struct Holder { union { int bits; double real; }; }; }",
    "int bits",
    "int gone",
    "void use(){dart::Holder value;value.bits=1;}",
)
_union(
    "anonymous_union_arrow",
    "namespace dart { struct Holder { union { int bits; double real; }; }; }",
    "int bits",
    "int gone",
    "void use(dart::Holder* value){value->bits=1;}",
)
_union(
    "anonymous_union_namespace",
    "namespace dart { static union { int bits; double real; }; }",
    "int bits",
    "int gone",
    "void use(){dart::bits=1;}",
)
_union(
    "anonymous_union_grouping",
    "namespace dart { struct Holder { union { int a; double b; }; union { int c; double d; }; }; }",
    "union { int a; double b; }; union { int c; double d; };",
    "union { int a; double b; int c; double d; };",
    "static_assert(sizeof(dart::Holder)==2*sizeof(double));",
)
_union(
    "union_foreign_negative",
    "namespace dart { struct A { void hit(); }; } namespace foreign { union Value { int bits; }; }",
    "int bits",
    "int gone",
    "void use(dart::A* known){foreign::Value value;value.bits=1;}",
    False,
    True,
)
_union(
    "union_unused_negative",
    U + " namespace dart { struct A { void hit(); }; }",
    "int bits",
    "int gone",
    "void use(dart::A* value){value->hit();}",
    False,
)

for name, header in [
    ("namespace_block", 'namespace dart { extern "C" { void hit(); } }'),
    ("outer_block", 'extern "C" { namespace dart { void hit(); } }'),
    (
        "nested_namespace",
        'namespace dart { extern "C" { namespace detail { void hit(); } } }',
    ),
    (
        "nested_linkage",
        'namespace dart { extern "C++" { extern "C" { void hit(); } } }',
    ),
    ("linkage_comment", 'namespace dart { extern /* comment */ "C" { void hit(); } }'),
    ("linkage_grouped", 'namespace dart { extern "C" { void hit(), untouched(); } }'),
    ("linkage_inline", 'namespace dart { extern "C" { inline void hit() {} } }'),
    ("linkage_individual", 'namespace dart { extern "C" void hit(); }'),
]:
    source = (
        "void use(){dart::"
        + ("detail::" if name == "nested_namespace" else "")
        + "hit();}"
    )
    old = "void hit()"
    new = "void hit(int)"
    _union(name, header, old, new, source)
_union(
    "linkage_literal",
    'namespace dart { extern "C" { void hit(); } }',
    '"C"',
    '"C++"',
    "void use(){dart::hit();}",
    True,
    False,
)
_union(
    "linkage_member_negative",
    'namespace dart { extern "C" { struct A { void hit(); }; } }',
    '"C"',
    '"C++"',
    "void use(dart::A& value){value.hit();}",
    False,
)
_union(
    "linkage_other_block_negative",
    'namespace dart { extern "C" { void hit(); } extern "C++" { void unused(); } }',
    "void unused()",
    "void unused(int)",
    "void use(){dart::hit();}",
    False,
)
_union(
    "linkage_body_negative",
    'namespace dart { extern "C" { inline void hit(){int value=1;} } }',
    "value=1",
    "value=2",
    "void use(){dart::hit();}",
    False,
)
_union(
    "linkage_comment_negative",
    'namespace dart { extern "C" { /* text1 */ void hit(); } }',
    "text1",
    "text2",
    "void use(){dart::hit();}",
    False,
)
_union(
    "linkage_closed_scope",
    'namespace dart { extern "C" { void unused(); } void hit(); }',
    '"C"',
    '"C++"',
    "void use(){dart::hit();}",
    False,
)

_union(
    "linkage_variable_signature",
    'namespace dart { extern "C" { extern int value; } }',
    "int value",
    "int gone",
    "void use(){dart::value=1;}",
)
_union(
    "linkage_variable_literal",
    'namespace dart { extern "C" { extern int value; } }',
    '"C"',
    '"C++"',
    "void use(){dart::value=1;}",
    True,
    False,
)
_union(
    "linkage_variable_individual",
    'namespace dart { extern "C" int value; }',
    '"C"',
    '"C++"',
    "void use(){dart::value=1;}",
    True,
    False,
)
_union(
    "linkage_variable_const",
    'namespace dart { extern "C" { extern const int value; } }',
    '"C"',
    '"C++"',
    "int use(){return dart::value;}",
    True,
    False,
)
_union(
    "linkage_variable_closed_scope",
    'namespace dart { extern "C" { extern int unused; } extern int value; }',
    '"C"',
    '"C++"',
    "void use(){dart::value=1;}",
    False,
)
_union(
    "linkage_static_field_negative",
    'namespace dart { extern "C" { struct A{static int value;}; } }',
    '"C"',
    '"C++"',
    "void use(){dart::A::value=1;}",
    False,
)


@pytest.mark.parametrize(
    "name,before_header,after_header,source,changed,breaking",
    ROUND17_ROOT_CASES,
    ids=[x[0] for x in ROUND17_ROOT_CASES],
)
def test_union_and_language_linkage_declarations(
    extractor, tmp_path, name, before_header, after_header, source, changed, breaking
):
    before = _scan_receiver_api(extractor, tmp_path, before_header, source)
    after = _scan_receiver_api(extractor, tmp_path, after_header, source)
    assert (before != after) == changed


ASSOCIATIVE17_HEADER = """namespace dart {
struct A { void hit() const; A* operator->(); A& operator()(); A& operator[](int); };
struct Hash { std::size_t operator()(const A&) const; };
struct PointerHash { std::size_t operator()(A*) const; };
struct Factory { KIND values(); };
}"""
ASSOCIATIVE17_KINDS = [
    "std::set<A>",
    "std::multiset<A>",
    "std::unordered_set<A,Hash>",
    "std::unordered_multiset<A,Hash>",
]
ASSOCIATIVE17_USES = [
    ("arrow", "void use(dart::Factory& factory) { factory.values().begin()->hit(); }"),
    (
        "const_iterator",
        "void use(dart::Factory& factory) { factory.values().cbegin()->hit(); }",
    ),
    (
        "dereference",
        "void use(dart::Factory& factory) { (*factory.values().begin()).hit(); }",
    ),
    (
        "auto_copy",
        "void use(dart::Factory& factory) { auto values=factory.values(); auto iterator=values.begin(); auto copy=iterator; copy->hit(); }",
    ),
    (
        "auto_element",
        "void use(dart::Factory& factory) { auto value=*factory.values().begin(); value.hit(); }",
    ),
    (
        "range_for",
        "void use(dart::Factory& factory) { for(const auto& value:factory.values()) { value.hit(); } }",
    ),
    (
        "const_range",
        "void use(dart::Factory& factory) { const auto values=factory.values(); values.begin()->hit(); }",
    ),
    (
        "move",
        "void use(dart::Factory& factory) { std::move(factory.values()).begin()->hit(); }",
    ),
]


@pytest.mark.parametrize("kind", ASSOCIATIVE17_KINDS)
@pytest.mark.parametrize("name,source", ASSOCIATIVE17_USES)
def test_associative_returns_preserve_iterator_element_shapes(
    extractor, tmp_path, kind, name, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    text = ASSOCIATIVE17_HEADER.replace("KIND", kind)
    header.write_text(text)
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(before, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0
    assert not {"operator->", "operator[]", "operator()"} & set(
        _entries(before, "members")
    )
    header.write_text(text.replace("void hit() const;", "void hit() const = delete;"))
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) != before


@pytest.mark.parametrize("kind", ["std::set<A>", "std::multiset<A>"])
@pytest.mark.parametrize("method", ["rbegin", "crbegin"])
def test_associative_reverse_iterators_preserve_elements(
    extractor, tmp_path, kind, method
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(ASSOCIATIVE17_HEADER.replace("KIND", kind))
    source = (
        "void use(dart::Factory& factory) { factory.values()." + method + "()->hit(); }"
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0


@pytest.mark.parametrize(
    "kind",
    [
        "std::set<A*>",
        "std::multiset<A*>",
        "std::unordered_set<A*,PointerHash>",
        "std::unordered_multiset<A*,PointerHash>",
    ],
)
def test_associative_pointer_elements_keep_extra_indirection(extractor, tmp_path, kind):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(ASSOCIATIVE17_HEADER.replace("KIND", kind))
    source = "void use(dart::Factory& factory) { auto iterator=factory.values().begin(); (*iterator)->hit(); }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0
    assert "operator->" not in _entries(result, "members")


@pytest.mark.parametrize("kind", ["std::set<A>", "std::unordered_set<A,Hash>"])
@pytest.mark.parametrize(
    "prefix,source",
    [
        (
            "std::optional<KIND>",
            "void use(dart::Factory& factory) { factory.values().value().begin()->hit(); }",
        ),
        (
            "std::vector<KIND>",
            "void use(dart::Factory& factory) { factory.values().front().begin()->hit(); }",
        ),
        (
            "std::shared_ptr<KIND>",
            "void use(dart::Factory& factory) { factory.values().get()->begin()->hit(); }",
        ),
    ],
)
def test_associative_nested_returns_preserve_elements(
    extractor, tmp_path, kind, prefix, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        ASSOCIATIVE17_HEADER.replace("KIND", prefix.replace("KIND", kind))
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0


@pytest.mark.parametrize("kind", ["std::set<A>", "std::unordered_set<A,Hash>"])
@pytest.mark.parametrize(
    "extra,source",
    [
        (
            "",
            "using Values=std::set<dart::A>; struct Provider { Values values(); }; void use(Provider& factory) { factory.values().begin()->hit(); }",
        ),
        (
            "using Values=KIND; struct Aliased { Values values(); };",
            "void use(dart::Aliased& factory) { factory.values().begin()->hit(); }",
        ),
        (
            "using Values=KIND; using Other=Values; struct Aliased { Other values(); };",
            "void use(dart::Aliased& factory) { factory.values().begin()->hit(); }",
        ),
        (
            "",
            "using std::set; struct Provider { set<dart::A> values(); }; void use(Provider& factory) { factory.values().begin()->hit(); }",
        ),
        (
            "",
            "namespace st=std; struct Provider { st::set<dart::A> values(); }; void use(Provider& factory) { factory.values().begin()->hit(); }",
        ),
    ],
)
def test_associative_alias_and_import_context(extractor, tmp_path, kind, extra, source):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        ASSOCIATIVE17_HEADER.replace("KIND", kind)
        + "namespace dart {"
        + extra.replace("KIND", kind)
        + "}"
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0


@pytest.mark.parametrize(
    "kind",
    [
        "std::set<Foreign>",
        "std::multiset<Foreign>",
        "std::unordered_set<Foreign,ForeignHash>",
        "std::unordered_multiset<Foreign,ForeignHash>",
    ],
)
def test_associative_foreign_elements_suppress_same_named_dart_member(
    extractor, tmp_path, kind
):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(ASSOCIATIVE17_HEADER.replace("KIND", "std::set<A>"))
    source = (
        "struct Foreign { void hit() const; }; struct ForeignHash { std::size_t operator()(const Foreign&) const; }; struct Provider { "
        + kind
        + " values(); }; void use(dart::A* known,Provider& factory) { factory.values().begin()->hit(); }"
    )
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "hit" not in _entries(before, "members")
    header.write_text(
        ASSOCIATIVE17_HEADER.replace("KIND", "std::set<A>").replace(
            "void hit() const;", "void hit() const = delete;"
        )
    )
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) == before


@pytest.mark.parametrize("kind", ["std::set<A>", "std::unordered_set<A,Hash>"])
@pytest.mark.parametrize(
    "extra,source",
    [
        (
            "using Values=KIND;",
            "void use() { dart::Values values; values.begin()->hit(); }",
        ),
        ("KIND values();", "void use() { dart::values().begin()->hit(); }"),
        (
            "KIND values();",
            "void use() { auto factory=&dart::values; factory().cbegin()->hit(); }",
        ),
        (
            "",
            "struct Provider { std::set<dart::A> values(); }; void use(Provider& factory) { factory.values().begin()->hit(); }",
        ),
        (
            "",
            "using namespace std; struct Provider { set<dart::A> values(); }; void use(Provider& factory) { factory.values().begin()->hit(); }",
        ),
    ],
)
def test_associative_connected_declarations_and_factories(
    extractor, tmp_path, kind, extra, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        ASSOCIATIVE17_HEADER.replace("KIND", kind)
        + "namespace dart {"
        + extra.replace("KIND", kind)
        + "}"
    )
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0


def test_associative_source_derived_elements_keep_dart_bases(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(
        ASSOCIATIVE17_HEADER.replace("KIND", "std::set<A>")
    )
    source = "struct Derived:dart::A {}; struct Provider { std::set<Derived> values(); }; void use(Provider& factory) { factory.values().begin()->hit(); }"
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0


def test_associative_std_import_respects_source_container_shadow(extractor, tmp_path):
    headers = tmp_path / "dart"
    headers.mkdir()
    header = headers / "api.hpp"
    header.write_text(ASSOCIATIVE17_HEADER.replace("KIND", "std::set<A>"))
    source = """struct Foreign { void hit() const; }; struct Iterator { Foreign* operator->(); };
using namespace std; namespace provider { template<class T> struct set { Iterator begin(); }; struct Factory { set<dart::A> values(); }; }
void use(provider::Factory& factory) { factory.values().begin()->hit(); }"""
    before = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    assert "hit" not in _entries(before, "members")
    header.write_text(
        ASSOCIATIVE17_HEADER.replace("KIND", "std::set<A>").replace(
            "void hit() const;", "void hit() const = delete;"
        )
    )
    assert _scan(extractor, extractor.index_api(headers), tmp_path, source) == before


@pytest.mark.parametrize(
    "kind,source",
    [
        (
            "std::set<std::shared_ptr<A>>",
            "void use(dart::Factory& factory) { factory.values().begin()->get()->hit(); }",
        ),
        (
            "std::set<std::optional<A>>",
            "void use(dart::Factory& factory) { factory.values().cbegin()->value().hit(); }",
        ),
    ],
)
def test_associative_wrapped_elements_reuse_existing_receiver_shapes(
    extractor, tmp_path, kind, source
):
    headers = tmp_path / "dart"
    headers.mkdir()
    (headers / "api.hpp").write_text(ASSOCIATIVE17_HEADER.replace("KIND", kind))
    result = _scan(extractor, extractor.index_api(headers), tmp_path, source)
    hit = _entries(result, "members")["hit"]
    assert hit["receiver_types"] == ["dart::A"] and hit["unresolved_count"] == 0
    assert not {"operator->", "operator[]", "operator()"} & set(
        _entries(result, "members")
    )


FUNCTION_ANNOTATION_MACROS = """#define DART_COMMON_API
#define DART_TEST_API(value)
#define DART_ALIAS_API DART_COMMON_API
#define DART_HIT hit
#define DART_CLASS A
#define DART_NAMESPACE dart
"""
FUNCTION_ANNOTATION_CASES = [
    (
        "free",
        "namespace dart { DART_COMMON_API int hit(); }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "member",
        "namespace dart { struct A { DART_COMMON_API int hit(); }; }",
        "DART_COMMON_API",
        "",
        "void use(dart::A& a){a.hit();}",
        True,
    ),
    (
        "constructor",
        "namespace dart { struct A { DART_COMMON_API A(); }; }",
        "DART_COMMON_API",
        "",
        "void use(){dart::A a;}",
        True,
    ),
    (
        "static_member",
        "namespace dart { struct A { DART_COMMON_API static int hit(); }; }",
        "DART_COMMON_API",
        "",
        "void use(){dart::A::hit();}",
        True,
    ),
    (
        "template_free",
        "namespace dart { template<class T> DART_COMMON_API int hit(T); }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit(1);}",
        True,
    ),
    (
        "template_member",
        "namespace dart { struct A { template<class T> DART_COMMON_API int hit(T); }; }",
        "DART_COMMON_API",
        "",
        "void use(dart::A& a){a.hit(1);}",
        True,
    ),
    (
        "friend",
        "namespace dart { struct A { friend DART_COMMON_API int hit(A&); }; }",
        "DART_COMMON_API",
        "",
        "namespace dart { void use(A& a){hit(a);} }",
        True,
    ),
    (
        "prefix_group_first",
        "namespace dart { DART_COMMON_API int hit(), miss(); }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "prefix_group_second",
        "namespace dart { DART_COMMON_API int miss(), hit(); }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "suffix_group_first",
        "namespace dart { int hit() DART_COMMON_API, miss(); }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "suffix_group_unrelated",
        "namespace dart { int miss() DART_COMMON_API, hit(); }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit();}",
        False,
    ),
    (
        "suffix_group_second",
        "namespace dart { int miss(), hit() DART_COMMON_API; }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "function_macro",
        'namespace dart { DART_TEST_API("default") int hit(); }',
        'DART_TEST_API("default")',
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "function_macro_nested",
        'namespace dart { DART_TEST_API(visibility("default")) int hit(); }',
        'DART_TEST_API(visibility("default"))',
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "alias_macro",
        "namespace dart { DART_ALIAS_API int hit(); }",
        "DART_ALIAS_API",
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "name_macro",
        "namespace dart { DART_COMMON_API int DART_HIT(); }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "class_macro",
        "namespace dart { struct DART_CLASS { DART_COMMON_API int hit(); }; }",
        "DART_COMMON_API",
        "",
        "void use(dart::A& a){a.hit();}",
        True,
    ),
    (
        "namespace_macro",
        "namespace DART_NAMESPACE { DART_COMMON_API int hit(); }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "gnu_prefix",
        'namespace dart { __attribute__((visibility("default"))) int hit(); }',
        '__attribute__((visibility("default")))',
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "gnu_suffix",
        'namespace dart { int hit() __attribute__((visibility("default"))); }',
        '__attribute__((visibility("default")))',
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "standard_attribute",
        'namespace dart { [[gnu::visibility("default")]] int hit(); }',
        '[[gnu::visibility("default")]]',
        "",
        "void use(){dart::hit();}",
        True,
    ),
    (
        "attribute_string",
        'namespace dart { __attribute__((visibility("default"))) int hit(); }',
        '"default"',
        '"hidden"',
        "void use(){dart::hit();}",
        True,
    ),
    (
        "macro_string",
        'namespace dart { DART_TEST_API("default") int hit(); }',
        '"default"',
        '"hidden"',
        "void use(){dart::hit();}",
        True,
    ),
    (
        "annotation_comment",
        'namespace dart { __attribute__((visibility(/* first */"default"))) int hit(); }',
        "/* first */",
        "/* second, longer */",
        "void use(){dart::hit();}",
        False,
    ),
    (
        "unrelated_function",
        "namespace dart { int hit(); DART_COMMON_API int miss(); }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit();}",
        False,
    ),
    (
        "unrelated_class",
        "namespace dart { int hit(); struct DART_COMMON_API A {}; }",
        "DART_COMMON_API",
        "",
        "void use(){dart::hit();}",
        False,
    ),
    (
        "body_attribute",
        "namespace dart { inline int hit(){ __attribute__((unused)) int local=0; return 1; } }",
        "__attribute__((unused))",
        "",
        "void use(){dart::hit();}",
        False,
    ),
    (
        "parameter_attribute",
        "namespace dart { int hit(int __attribute__((unused)) value); }",
        "__attribute__((unused))",
        "",
        "void use(){dart::hit(1);}",
        True,
    ),
    (
        "ordinary_name",
        "namespace dart { int DART_HIT(); }",
        "#define DART_HIT hit",
        "#define DART_HIT hit",
        "void use(){dart::hit();}",
        False,
    ),
]


@pytest.mark.parametrize(
    ("label", "header", "old", "new", "source", "drift"), FUNCTION_ANNOTATION_CASES
)
def test_raw_function_export_annotation_evidence(
    extractor, tmp_path, label, header, old, new, source, drift
):
    original = FUNCTION_ANNOTATION_MACROS + header
    before = _scan_receiver_api(extractor, tmp_path, original, source)
    after = _scan_receiver_api(
        extractor,
        tmp_path,
        FUNCTION_ANNOTATION_MACROS + header.replace(old, new),
        source,
    )
    assert (before != after) == drift
    if label == "parameter_attribute":
        declarations = _entries(before, "free_functions")["dart::hit"]["declarations"]
        assert all(
            "declaration_annotations" not in declaration for declaration in declarations
        )
    if drift and label != "parameter_attribute":
        declarations = [
            declaration
            for section in ("free_functions", "members", "constructors")
            for row in before[section]
            for declaration in row.get("declarations", ())
        ]
        assert any(
            declaration.get("declaration_annotations") for declaration in declarations
        )


ROUND17_SPEC_CASES = [
    (
        "friend_declaration_linkage",
        'namespace dart { extern "C" { struct A { friend void hit(A*); }; } }',
        '"C"',
        '"C++"',
        "void use(dart::A* value) { hit(value); }",
        True,
        False,
    ),
    (
        "friend_definition_linkage",
        'namespace dart { extern "C" { struct A { friend void hit(A*) {} }; } }',
        '"C"',
        '"C++"',
        "void use(dart::A* value) { hit(value); }",
        True,
        False,
    ),
    (
        "out_of_line_member_linkage_negative",
        'namespace dart { struct A { void hit(); }; extern "C" { void A::hit() {} } }',
        '"C"',
        '"C++"',
        "void use(dart::A& value) { value.hit(); }",
        False,
        False,
    ),
    (
        "static_member_linkage_negative",
        'namespace dart { extern "C" { struct A { static void hit(); }; } }',
        '"C"',
        '"C++"',
        "void use() { dart::A::hit(); }",
        False,
        False,
    ),
    (
        "nearest_language_override_negative",
        'namespace dart { extern "C" { extern "C++" { void hit(); } } }',
        'extern "C"',
        'extern "C++"',
        "void use() { dart::hit(); }",
        False,
        False,
    ),
    (
        "nearest_language_changes",
        'namespace dart { extern "C" { extern "C++" { void hit(); } } }',
        'extern "C++"',
        'extern "C"',
        "void use() { dart::hit(); }",
        True,
        False,
    ),
    (
        "empty_owned_union_kind",
        "namespace dart { union Value {}; struct Holder { Value value; }; }",
        "union Value",
        "struct Value",
        "void use() { dart::Holder holder; static_assert(std::is_union_v<decltype(holder.value)>); }",
        True,
        True,
    ),
    (
        "empty_union_kind_control",
        "namespace dart { union Value {}; }",
        "union Value",
        "struct Value",
        "static_assert(std::is_union_v<dart::Value>);",
        True,
        True,
    ),
    (
        "empty_owned_union_alias_kind",
        "namespace dart { union Value {}; using Alias=Value; struct Holder { Alias value; }; }",
        "union Value",
        "struct Value",
        "void use() { dart::Holder holder; static_assert(std::is_union_v<decltype(holder.value)>); }",
        True,
        True,
    ),
    (
        "empty_union_pointer_field_negative",
        "namespace dart { union Value {}; struct Holder { Value* value; }; }",
        "union Value",
        "struct Value",
        "static_assert(sizeof(dart::Holder)==sizeof(void*));",
        False,
        False,
    ),
    (
        "empty_union_reference_field_negative",
        "namespace dart { union Value {}; struct Holder { Value& value; }; }",
        "union Value",
        "struct Value",
        "static_assert(sizeof(dart::Holder)==sizeof(void*));",
        False,
        False,
    ),
]


@pytest.mark.parametrize(
    "name,header,old,new,source,changed,breaking",
    ROUND17_SPEC_CASES,
    ids=[case[0] for case in ROUND17_SPEC_CASES],
)
def test_union_and_linkage_connected_declarations(
    extractor, tmp_path, name, header, old, new, source, changed, breaking
):
    before = _scan_receiver_api(extractor, tmp_path, header, source)
    after = _scan_receiver_api(extractor, tmp_path, header.replace(old, new), source)
    assert (before != after) == changed


@pytest.mark.parametrize(
    "declarator,source",
    [
        ("storage", "void use(){dart::Holder value; value.storage.bits=1;}"),
        ("storage", "static_assert(sizeof(dart::Holder)==sizeof(double));"),
        ("first, storage", "void use(){dart::Holder value; value.storage.bits=1;}"),
        ("storage[2]", "void use(){dart::Holder value; value.storage[1].bits=1;}"),
        ("*storage", "void use(dart::Holder& value){value.storage->bits=1;}"),
    ],
)
def test_inline_named_union_declarator_evidence(
    extractor, tmp_path, declarator, source
):
    header = (
        "namespace dart {struct Holder {union Value {int bits; double real;} "
        + declarator
        + ";};}"
    )
    before = _scan_receiver_api(extractor, tmp_path, header, source)
    after = _scan_receiver_api(
        extractor, tmp_path, header.replace("int bits", "int gone"), source
    )
    if "sizeof" in source:
        after = _scan_receiver_api(
            extractor, tmp_path, header.replace("union Value", "struct Value"), source
        )
    assert before != after


@pytest.mark.parametrize(
    "declarator,field",
    [("storage={'a'}", "storage"), ("first={'x'}, storage={'a'}", "storage")],
)
def test_inline_named_union_initializer_literal(extractor, tmp_path, declarator, field):
    header = (
        "namespace dart {struct Holder {union Value {char bits; int real;} "
        + declarator
        + ";};}"
    )
    source = (
        "constexpr dart::Holder value{}; static_assert(value." + field + ".bits=='a');"
    )
    before = _scan_receiver_api(extractor, tmp_path, header, source)
    after = _scan_receiver_api(
        extractor, tmp_path, header.replace("'a'", "'b'"), source
    )
    assert before != after
    declarations = _entries(before, "constructors")["dart::Holder"][
        "default_member_initializer_declarations"
    ]
    assert any(item.get("initializer_literals") == ["'a'"] for item in declarations)


@pytest.mark.parametrize(
    "header",
    [
        'namespace dart {extern "C" union Value {int bits;} value;}',
        'namespace dart {extern "C" {union Value {int bits;} value;}}',
    ],
)
def test_inline_named_union_variable_language_linkage(extractor, tmp_path, header):
    source = "void use(){dart::value.bits=1;}"
    before = _scan_receiver_api(extractor, tmp_path, header, source)
    after = _scan_receiver_api(
        extractor, tmp_path, header.replace('"C"', '"C++"'), source
    )
    assert before != after


def test_inline_named_union_static_field_preserves_pointer_layout(extractor, tmp_path):
    header = "namespace dart {struct Holder {static union Value {int bits;} storage;};}"
    source = "static_assert(sizeof(dart::Holder)==1);"
    before = _scan_receiver_api(extractor, tmp_path, header, source)
    after = _scan_receiver_api(
        extractor, tmp_path, header.replace("int bits", "double bits"), source
    )
    assert before == after
