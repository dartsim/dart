#include <dart/gui/osg/all.hpp>

#include <osg/GraphicsContext>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "../core/WorkbenchWorldNode.hpp"
#include "../registry/ExampleRegistry.hpp"
#include "../core/UiScaling.hpp"

#include "WorkbenchWidget.hpp"

int main()
{
#if defined(__linux__)
  if (!std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
    std::cerr
        << "DART Workbench requires an X11 or Wayland display. Set DISPLAY "
        << "before running.\n";
    return 0;
  }
#endif

  auto viewer = ::osg::ref_ptr<dart::gui::osg::ImGuiViewer>(
      new dart::gui::osg::ImGuiViewer);
  auto worldNode = ::osg::ref_ptr<workbench::WorkbenchWorldNode>(
      new workbench::WorkbenchWorldNode);
  viewer->addWorldNode(worldNode);

  const float initialScale = std::clamp(workbench::detectUiScale(), 0.5f, 3.0f);

  auto widget = std::make_shared<workbench::WorkbenchWidget>(
      viewer,
      worldNode,
      workbench::detectExamplesRoot(),
      initialScale);
  viewer->getImGuiHandler()->addWidget(widget);

  viewer->addInstructionText(
      "DART Workbench\n"
      "  * Left panel: select a scene or inspect standalone examples\n"
      "  * Properties panel: tweak simulation settings\n"
      "  * Bottom panel: review logs and performance metrics\n");

  const int windowWidth = static_cast<int>(std::round(1280.0f * initialScale));
  const int windowHeight = static_cast<int>(std::round(768.0f * initialScale));
  viewer->setUpViewInWindow(0, 0, std::max(windowWidth, 960), std::max(windowHeight, 600));
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(4.0f, 4.0f, 2.5f),
      ::osg::Vec3(0.0f, 0.0f, 0.5f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  viewer->simulate(true);
  viewer->realize();
  if (!viewer->isRealized()) {
    std::cerr << "Unable to realize the OSG viewer. Running without a GUI.\n";
    return 0;
  }

  viewer->run();
}
