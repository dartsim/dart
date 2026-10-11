#pragma once

#include <dart/gui/GUIEventHandler.hpp>
#include <dart/gui/ImGuiWidget.hpp>
#include <dart/gui/RealTimeWorldNode.hpp>
#include <dart/gui/Viewer.hpp>
#include <dart/gui/ViewerAttachment.hpp>
#include <dart/gui/WorldNode.hpp>

namespace dartnb::gui {
struct Ownership
{
  struct Edge
  {
    void* pointer;
    std::weak_ptr<void> native;
    nb::object wrapper;
    std::function<void()> detach;
    std::weak_ptr<dart::gui::WorldNode> node;
  };
  dart::gui::Viewer* owner;
  std::vector<Edge> active;
  explicit Ownership(dart::gui::Viewer* value) : owner(value) {}
  ~Ownership();
  void clear(PyObject* self);
};
std::shared_ptr<Ownership> ownership(dart::gui::Viewer* owner);
void retire(dart::gui::Viewer* owner, void* child);
const PyType_Slot* slots();
const PyType_Slot* nodeSlots();
std::vector<std::shared_ptr<dart::gui::WorldNode>> nodes(
    dart::gui::Viewer* owner);

template <class T, class Remove>
void retain(
    dart::gui::Viewer* owner, const std::shared_ptr<T>& child, Remove remove)
{
  if (!child)
    return;
  auto state = ownership(owner);
  for (const auto& edge : state->active)
    if (edge.pointer == child.get())
      return;
  std::weak_ptr<T> weak = child;
  Ownership::Edge edge{
      child.get(),
      child,
      nb::cast(child, nb::rv_policy::reference),
      [owner, weak, remove] {
        if (auto value = weak.lock())
          (owner->*remove)(value);
      },
      {}};
  if constexpr (std::is_base_of_v<dart::gui::WorldNode, T>)
    edge.node = child;
  state->active.push_back(std::move(edge));
}
} // namespace dartnb::gui

namespace dartnb {
template <>
inline std::vector<BaseEdge> extra_base_edges<dart::gui::Viewer>()
{
  return {base_edge<dart::gui::Viewer, dart::common::Subject>()};
}
template <>
inline const PyType_Slot* gc_slots<dart::gui::Viewer>()
{
  return gui::slots();
}
template <>
inline const PyType_Slot* gc_slots<dart::gui::WorldNode>()
{
  return gui::nodeSlots();
}
template <>
inline const PyType_Slot* gc_slots<dart::gui::RealTimeWorldNode>()
{
  return gui::nodeSlots();
}
} // namespace dartnb
