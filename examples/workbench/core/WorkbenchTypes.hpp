#ifndef DART_DOC_EXAMPLES_WORKBENCH_CORE_WORKBENCHTYPES_HPP_
#define DART_DOC_EXAMPLES_WORKBENCH_CORE_WORKBENCHTYPES_HPP_

#include <dart/gui/osg/ImGuiViewer.hpp>
#include <dart/simulation/World.hpp>

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace workbench {

class WorkbenchWidget;
class WorkbenchWorldNode;

struct WorkbenchServices
{
  WorkbenchWidget* widget{nullptr};
  dart::gui::osg::ImGuiViewer* viewer{nullptr};
  WorkbenchWorldNode* worldNode{nullptr};

  void log(const std::string& message) const;
  void requestReload() const;
  dart::gui::osg::ImGuiViewer& viewerRef() const;
  WorkbenchWorldNode& worldNodeRef() const;
};

struct ExampleSession
{
  dart::simulation::WorldPtr world;
  std::function<void(WorkbenchServices&, double)> onStep;
  std::function<void(WorkbenchServices&)> onPreRefresh;
  std::function<void(WorkbenchServices&)> onPostRefresh;
  std::function<void(WorkbenchServices&)> onActivate;
  std::function<void(WorkbenchServices&)> onDeactivate;
  std::function<void(WorkbenchServices&)> onOverviewUi;
  std::function<void(WorkbenchServices&)> onPropertiesUi;
};

struct ExampleRecord
{
  std::string id;
  std::string name;
  std::string category;
  std::string description;
  std::filesystem::path directory;
  std::function<ExampleSession(WorkbenchServices&)> factory;

  [[nodiscard]] bool hasFactory() const;
};

using ExampleList = std::vector<ExampleRecord>;
using CategoryMap = std::unordered_map<std::string, std::vector<std::size_t>>;

} // namespace workbench

#endif // DART_DOC_EXAMPLES_WORKBENCH_CORE_WORKBENCHTYPES_HPP_
