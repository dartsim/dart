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
    assert solver["examples"] == ["dartsim/src/sample.cc:1", "dartsim/src/sample.cc:8"]
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
