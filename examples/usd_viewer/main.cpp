#include <dart/gui/osg/Viewer.hpp>
#include <dart/gui/osg/WorldNode.hpp>

#include <dart/utils/usd/UsdParser.hpp>

#include <iostream>

int main()
{
  auto world = dart::utils::UsdParser::readWorld(
      "dart://sample/usd/unitree_h1_minimal.usda");

  if (!world) {
    std::cerr
        << "Failed to load USD world from data/usd/unitree_h1_minimal.usda\n";
    return 1;
  }

  auto viewer = std::make_shared<dart::gui::osg::Viewer>();
  auto node = new dart::gui::osg::WorldNode(world);
  viewer->addWorldNode(node);
  viewer->setUpViewInWindow(100, 100, 1280, 720);
  viewer->run();

  return 0;
}
