#include <dart/gui/osg/all.hpp>

#include <osg/GraphicsContext>

#include <cstdlib>
#include <iostream>

#include "../core/WorkbenchWorldNode.hpp"
#include "../registry/ExampleRegistry.hpp"

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

  auto* wsi = ::osg::GraphicsContext::getWindowingSystemInterface();
  if (!wsi) {
    std::cerr << "No windowing system interface available; skipping launch.\n";
    return 0;
  }

  ::osg::ref_ptr<::osg::GraphicsContext::Traits> traits
      = new ::osg::GraphicsContext::Traits;
  traits->x = 0;
  traits->y = 0;
  traits->width = 1;
  traits->height = 1;
  traits->windowDecoration = false;
  traits->doubleBuffer = true;
  traits->pbuffer = false;
  traits->supportsResize = false;
  traits->windowName = "WorkbenchProbe";
  traits->sharedContext = nullptr;

  ::osg::ref_ptr<::osg::GraphicsContext> probeContext
      = ::osg::GraphicsContext::createGraphicsContext(traits.get());
  if (!probeContext) {
    const char* displayEnv = std::getenv("DISPLAY");
    std::cerr << "Unable to open display '"
              << (displayEnv ? displayEnv : "(unset)")
              << "'. Ensure an X11/Wayland server is available.\n";
    return 0;
  }
  probeContext->close();

  auto viewer = ::osg::ref_ptr<dart::gui::osg::ImGuiViewer>(
      new dart::gui::osg::ImGuiViewer);
  auto worldNode = ::osg::ref_ptr<workbench::WorkbenchWorldNode>(
      new workbench::WorkbenchWorldNode);
  viewer->addWorldNode(worldNode);

  auto widget = std::make_shared<workbench::WorkbenchWidget>(
      viewer,
      worldNode,
      workbench::detectExamplesRoot());
  viewer->getImGuiHandler()->addWidget(widget);

  viewer->addInstructionText(
      "DART Workbench\n"
      "  * Left panel: select a scene or inspect standalone examples\n"
      "  * Properties panel: tweak simulation settings\n"
      "  * Bottom panel: review logs and performance metrics\n");

  viewer->setUpViewInWindow(0, 0, 1280, 768);
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(4.0f, 4.0f, 2.5f),
      ::osg::Vec3(0.0f, 0.0f, 0.5f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  viewer->simulate(true);
  viewer->realize();
  if (!viewer->isRealized()) {
    std::cerr << "Unable to realize the OSG viewer. Ensure a graphics "
              << "context is available.\n";
    return 2;
  }

  viewer->run();
}
