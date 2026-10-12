"""Fast lexical and freshness checks for the DART C++ API inventory."""

import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "gz_dart_api_inventory.py"
INVENTORY = ROOT / "tools" / "gazebo" / "compat" / "dart_api_inventory.json"
SECTIONS = ("types", "members", "free_functions", "macros", "overrides")


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


def _scan(extractor, api, tmp_path, snippet):
    source = tmp_path / "source"
    folder = source / "dartsim" / "src"
    folder.mkdir(parents=True)
    (folder / "sample.cc").write_text(snippet)
    return extractor.scan_sources(api, source)


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
                if section == "types":
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
                    assert entry["declarations"]
                if section == "macros":
                    assert isinstance(entry["declared_in_dart"], bool)
                for declaration in entry.get("declarations", []):
                    assert isinstance(declaration["const"], bool)
                    assert isinstance(declaration["virtual"], bool)
                    assert declaration["signature"]
