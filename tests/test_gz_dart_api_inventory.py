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
    "members",
    "constructors",
    "data_members",
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
private:
  void privateOnly();
};
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
    assert widget["count"] == 8
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
            "d.hh": "namespace gz { struct Base {\n"
            "  using DartWidget = dart::dynamics::Widget;\n"
            "}; }\n",
            "e.cc": '#include "d.hh"\nvoid e() { DartWidget::compute(); }\n',
            "f.cc": "void f() { DartWidget::clear(); }\n",
        },
    )
    members = _entries(result, "members")
    assert members["getValue"]["examples"] == ["dartsim/src/a.cc:2"]
    assert members["isReady"]["examples"] == ["dartsim/src/c.cc:3"]
    assert members["getStaticType"]["count"] == 2
    assert members["compute"]["examples"] == ["dartsim/src/e.cc:2"]
    assert "clear" not in members


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
                if section in {"members", "data_members"}:
                    assert entry["declarations"]
                if section == "macros":
                    assert isinstance(entry["declared_in_dart"], bool)
                for declaration in entry.get("declarations", []):
                    assert isinstance(declaration["const"], bool)
                    assert isinstance(declaration["virtual"], bool)
                    assert declaration["signature"]
