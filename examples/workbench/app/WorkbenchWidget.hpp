#ifndef DART_DOC_EXAMPLES_WORKBENCH_APP_WORKBENCHWIDGET_HPP_
#define DART_DOC_EXAMPLES_WORKBENCH_APP_WORKBENCHWIDGET_HPP_

#include <dart/gui/osg/ImGuiWidget.hpp>

#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <imgui.h>

#include "../core/WorkbenchTypes.hpp"

namespace workbench {

struct WorkbenchLayout
{
  ImVec2 viewportPos;
  ImVec2 viewportSize;
  ImVec2 origin;
  float leftWidth;
  float centerWidth;
  float rightWidth;
  float centerHeight;
  float bottomHeight;
  float padding;
  float topBarHeight;
};

class WorkbenchWidget : public dart::gui::osg::ImGuiWidget
{
public:
  WorkbenchWidget(
      ::osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer,
      ::osg::ref_ptr<WorkbenchWorldNode> worldNode,
      std::filesystem::path examplesRoot);

  void render() override;

  void log(const std::string& message);
  void reloadCurrentExample();

  dart::gui::osg::ImGuiViewer& viewer();
  WorkbenchWorldNode& worldNode();

private:
  WorkbenchLayout computeLayout() const;
  void updateMetrics();

  void drawTopBar(const WorkbenchLayout& layout);
  void drawExampleBrowser(const WorkbenchLayout& layout);
  void drawOverview(const WorkbenchLayout& layout);
  void drawProperties(const WorkbenchLayout& layout);
  void drawLog(const WorkbenchLayout& layout);

  void selectExample(std::size_t index, bool forceReload);
  void loadExample(std::size_t index);
  void unloadExample();
  void toggleSimulation();

  void rebuildCategoryView();
  void ensureSelection();

  ::osg::ref_ptr<dart::gui::osg::ImGuiViewer> mViewer;
  ::osg::ref_ptr<WorkbenchWorldNode> mWorldNode;

  WorkbenchServices mServices;
  std::filesystem::path mExamplesRoot;

  ExampleList mExamples;
  std::vector<std::string> mCategoryOrder;
  CategoryMap mCategoryIndices;
  std::shared_ptr<ExampleSession> mActiveSession;

  std::deque<std::string> mLogLines;
  bool mScrollLogToBottom{false};

  std::vector<float> mRtfHistory;

  std::size_t mSelectedIndex{std::numeric_limits<std::size_t>::max()};
};

} // namespace workbench

#endif // DART_DOC_EXAMPLES_WORKBENCH_APP_WORKBENCHWIDGET_HPP_
