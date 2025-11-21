/*
 * Minimal GUI stubs to keep Python demos runnable without legacy glut bindings.
 */

#include "gui/module.hpp"

#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/simulation/World.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace dart::python_nb {

namespace gui {

struct GUIActionAdapter
{
};

struct GUIEventAdapter
{
  enum class EventType
  {
    KEYDOWN,
    KEYUP
  };

  GUIEventAdapter(int key = 0, EventType type = EventType::KEYDOWN)
    : mKey(key), mType(type)
  {
  }

  EventType getEventType() const { return mType; }
  int getKey() const { return mKey; }

  int mKey;
  EventType mType;
};

struct GUIEventHandler
{
  virtual ~GUIEventHandler() = default;
  virtual bool handle(
      const GUIEventAdapter&, const GUIActionAdapter&) // NOLINT(*-unnecessary-virtual)
  {
    return false;
  }
};

struct PyGUIEventHandler : GUIEventHandler
{
  NB_TRAMPOLINE(GUIEventHandler, 1);
  bool handle(
      const GUIEventAdapter& ea, const GUIActionAdapter& aa) override
  {
    NB_OVERRIDE(handle, ea, aa);
  }
};

struct RealTimeWorldNode
{
  explicit RealTimeWorldNode(std::shared_ptr<dart::simulation::World> world)
    : mWorld(std::move(world))
  {
  }
  virtual ~RealTimeWorldNode() = default;

  virtual void customPreRefresh() {} // NOLINT(*-unnecessary-virtual)

  std::shared_ptr<dart::simulation::World> mWorld;
};

struct PyRealTimeWorldNode : RealTimeWorldNode
{
  NB_TRAMPOLINE(RealTimeWorldNode, 1);
  void customPreRefresh() override { NB_OVERRIDE(customPreRefresh); }
};

struct DragAndDropHandle
{
  explicit DragAndDropHandle(
      std::shared_ptr<dart::dynamics::SimpleFrame> f = nullptr)
    : frame(std::move(f))
  {
  }
  std::shared_ptr<dart::dynamics::SimpleFrame> frame;
};

struct SupportPolygonVisual
{
  SupportPolygonVisual(
      std::shared_ptr<dart::dynamics::Skeleton> skel, double elevation)
    : skeleton(std::move(skel)), elevation(elevation)
  {
  }

  std::shared_ptr<dart::dynamics::Skeleton> skeleton;
  double elevation;
};

struct Viewer
{
  void allowSimulation(bool allow) { mSimulationAllowed = allow; }

  void addWorldNode(const std::shared_ptr<RealTimeWorldNode>& node)
  {
    mWorldNodes.push_back(node);
  }

  template <typename T>
  T addAttachment(T attachment)
  {
    return attachment;
  }

  void addInstructionText(const std::string& text)
  {
    mInstructions += text;
  }

  std::string getInstructions() const { return mInstructions; }

  void addEventHandler(const std::shared_ptr<GUIEventHandler>& handler)
  {
    mHandlers.push_back(handler);
  }

  std::shared_ptr<DragAndDropHandle> enableDragAndDrop(
      const std::shared_ptr<dart::dynamics::SimpleFrame>& frame)
  {
    auto handle = std::make_shared<DragAndDropHandle>(frame);
    mDrag.push_back(handle);
    return handle;
  }

  void disableDragAndDrop(const std::shared_ptr<DragAndDropHandle>& handle)
  {
    auto it = std::remove(mDrag.begin(), mDrag.end(), handle);
    mDrag.erase(it, mDrag.end());
  }

  void setUpViewInWindow(int, int, int, int) {}
  void setCameraHomePosition(
      const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&)
  {
  }

  void run(int steps = 1)
  {
    for (int i = 0; i < steps; ++i) {
      for (auto& node : mWorldNodes) {
        if (node)
          node->customPreRefresh();
      }
    }
    std::printf("[dartpy_nb.gui] Headless viewer run completed.\n");
  }

  bool mSimulationAllowed = true;
  std::vector<std::shared_ptr<RealTimeWorldNode>> mWorldNodes;
  std::vector<std::shared_ptr<GUIEventHandler>> mHandlers;
  std::vector<std::shared_ptr<DragAndDropHandle>> mDrag;
  std::string mInstructions;
};

struct GridVisual
{
  enum class PlaneType
  {
    XY,
    XZ,
    ZX
  };

  void setPlaneType(PlaneType type) { mType = type; }
  PlaneType getPlaneType() const { return mType; }
  void setOffset(const Eigen::Vector3d& offset) { mOffset = offset; }
  Eigen::Vector3d getOffset() const { return mOffset; }

  PlaneType mType = PlaneType::XZ;
  Eigen::Vector3d mOffset = Eigen::Vector3d::Zero();
};

struct WorldNode : RealTimeWorldNode
{
  explicit WorldNode(const std::shared_ptr<dart::simulation::World>& world)
    : RealTimeWorldNode(world)
  {
  }

  static nb::object createDefaultShadowTechnique(Viewer&) { return nb::none(); }
};

struct InteractiveFrame : dart::dynamics::SimpleFrame
{
  InteractiveFrame(
      dart::dynamics::Frame* refFrame,
      const std::string& name,
      const Eigen::Isometry3d& tf)
    : dart::dynamics::SimpleFrame(refFrame, name, tf)
  {
  }
};

} // namespace gui

namespace {

Eigen::Vector4d toVec4(const nb::handle& h)
{
  try {
    return nb::cast<Eigen::Vector4d>(h);
  } catch (const nb::cast_error&) {
    nb::sequence seq = nb::cast<nb::sequence>(h);
    if (nb::len(seq) != 4)
      throw nb::type_error("Expected a length-4 sequence");
    Eigen::Vector4d vec;
    for (ssize_t i = 0; i < 4; ++i)
      vec[i] = nb::cast<double>(seq[i]);
    return vec;
  }
}

Eigen::Vector3d toVec3(const nb::handle& h)
{
  try {
    return nb::cast<Eigen::Vector3d>(h);
  } catch (const nb::cast_error&) {
    nb::sequence seq = nb::cast<nb::sequence>(h);
    if (nb::len(seq) != 3)
      throw nb::type_error("Expected a length-3 sequence");
    Eigen::Vector3d vec;
    for (ssize_t i = 0; i < 3; ++i)
      vec[i] = nb::cast<double>(seq[i]);
    return vec;
  }
}

} // namespace

void defGuiModule(nb::module_& m)
{
  using namespace gui;

  nb::enum_<GUIEventAdapter::EventType>(m, "EventType")
      .value("KEYDOWN", GUIEventAdapter::EventType::KEYDOWN)
      .value("KEYUP", GUIEventAdapter::EventType::KEYUP);

  nb::class_<GUIEventAdapter>(m, "GUIEventAdapter")
      .def(nb::init<int, GUIEventAdapter::EventType>(),
          nb::arg("key") = 0,
          nb::arg("type") = GUIEventAdapter::EventType::KEYDOWN)
      .def("getEventType", &GUIEventAdapter::getEventType)
      .def("getKey", &GUIEventAdapter::getKey);

  nb::class_<GUIActionAdapter>(m, "GUIActionAdapter")
      .def(nb::init<>());

  nb::class_<GUIEventHandler, PyGUIEventHandler>(m, "GUIEventHandler")
      .def(nb::init<>())
      .def("handle", &GUIEventHandler::handle);

  nb::class_<RealTimeWorldNode, PyRealTimeWorldNode>(
      m, "RealTimeWorldNode", nb::dynamic_attr())
      .def(nb::init<std::shared_ptr<dart::simulation::World>>(),
          nb::arg("world") = std::shared_ptr<dart::simulation::World>{})
      .def("customPreRefresh", &RealTimeWorldNode::customPreRefresh);

  nb::class_<WorldNode, RealTimeWorldNode>(m, "WorldNode")
      .def(nb::init<std::shared_ptr<dart::simulation::World>>(),
          nb::arg("world") = std::shared_ptr<dart::simulation::World>{})
      .def_static(
          "createDefaultShadowTechnique",
          &WorldNode::createDefaultShadowTechnique,
          nb::arg("viewer"));

  nb::class_<DragAndDropHandle>(m, "DragAndDropHandle")
      .def(nb::init<std::shared_ptr<dart::dynamics::SimpleFrame>>(),
          nb::arg("frame"));

  nb::class_<SupportPolygonVisual>(m, "SupportPolygonVisual")
      .def(nb::init<
          std::shared_ptr<dart::dynamics::Skeleton>,
          double>(),
          nb::arg("skeleton"),
          nb::arg("elevation"));

  nb::class_<Viewer>(m, "Viewer")
      .def(nb::init<>())
      .def(
          "__init__",
          [](Viewer* self, const nb::handle& clearColor) {
            (void) toVec4(clearColor); // validate but ignore
            new (self) Viewer();
          },
          nb::arg("clearColor"))
      .def("allowSimulation", &Viewer::allowSimulation, nb::arg("allow"))
      .def("addWorldNode", &Viewer::addWorldNode, nb::arg("node"))
      .def("addAttachment", &Viewer::addAttachment<nb::object>, nb::arg("obj"))
      .def("addInstructionText", &Viewer::addInstructionText, nb::arg("text"))
      .def("getInstructions", &Viewer::getInstructions)
      .def("addEventHandler", &Viewer::addEventHandler, nb::arg("handler"))
      .def(
          "enableDragAndDrop",
          &Viewer::enableDragAndDrop,
          nb::arg("frame"),
          nb::rv_policy::reference_internal)
      .def(
          "disableDragAndDrop",
          &Viewer::disableDragAndDrop,
          nb::arg("handle"))
      .def(
          "setUpViewInWindow",
          &Viewer::setUpViewInWindow,
          nb::arg("x"),
          nb::arg("y"),
          nb::arg("width"),
          nb::arg("height"))
      .def(
          "setCameraHomePosition",
          [](Viewer& self,
             const nb::handle& eye,
             const nb::handle& center,
             const nb::handle& up) {
            self.setCameraHomePosition(toVec3(eye), toVec3(center), toVec3(up));
          },
          nb::arg("eye"),
          nb::arg("center"),
          nb::arg("up"))
      .def("run", &Viewer::run, nb::arg("steps") = 1);

  auto gridClass = nb::class_<GridVisual>(m, "GridVisual");
  nb::enum_<GridVisual::PlaneType>(gridClass, "PlaneType")
      .value("XY", GridVisual::PlaneType::XY)
      .value("XZ", GridVisual::PlaneType::XZ)
      .value("ZX", GridVisual::PlaneType::ZX);
  gridClass.def(nb::init<>())
      .def("setPlaneType", &GridVisual::setPlaneType, nb::arg("planeType"))
      .def("getPlaneType", &GridVisual::getPlaneType)
      .def(
          "setOffset",
          [](GridVisual& self, const nb::handle& offset) {
            self.setOffset(toVec3(offset));
          },
          nb::arg("offset"))
      .def("getOffset", &GridVisual::getOffset);

  nb::class_<InteractiveFrame, dart::dynamics::SimpleFrame>(
      m, "InteractiveFrame")
      .def(nb::init<
          dart::dynamics::Frame*,
          const std::string&,
          const Eigen::Isometry3d&>(),
          nb::arg("refFrame"),
          nb::arg("name") = "",
          nb::arg("relativeTransform") = Eigen::Isometry3d::Identity());
}

} // namespace dart::python_nb
