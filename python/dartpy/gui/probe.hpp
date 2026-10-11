#pragma once

#include <dart/gui/GUIEventHandler.hpp>
#include <dart/gui/Viewer.hpp>
#include <dart/gui/WorldNode.hpp>

namespace dart::python::gui_probe {
inline void refresh(dart::gui::WorldNode* node)
{
  node->refresh();
}

inline bool handle(
    dart::gui::GUIEventHandler* handler, dart::gui::Viewer* viewer, int key)
{
  dart::gui::GUIEventAdapter event(dart::gui::GUIEventAdapter::KEYDOWN, key);
  dart::gui::GUIActionAdapter action(*viewer);
  return handler->handle(event, action);
}
} // namespace dart::python::gui_probe
