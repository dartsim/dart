#include "WorkbenchTypes.hpp"

#include "WorkbenchWorldNode.hpp"

#include <cassert>

#include "../app/WorkbenchWidget.hpp"

namespace workbench {

bool ExampleRecord::hasFactory() const
{
  return static_cast<bool>(factory);
}

void WorkbenchServices::log(const std::string& message) const
{
  if (widget)
    widget->log(message);
}

void WorkbenchServices::requestReload() const
{
  if (widget)
    widget->reloadCurrentExample();
}

dart::gui::osg::ImGuiViewer& WorkbenchServices::viewerRef() const
{
  assert(viewer != nullptr);
  return *viewer;
}

WorkbenchWorldNode& WorkbenchServices::worldNodeRef() const
{
  assert(worldNode != nullptr);
  return *worldNode;
}

} // namespace workbench
