#include "dart/gui/glfw/WorldScene.hpp"

#include "dart/common/StlHelpers.hpp"
#include "dart/gui/glfw/Entity.hpp"
#include "dart/math/Constants.hpp"
#include "dart/gui/glfw/Box.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
WorldScene::WorldScene(simulation::WorldPtr world)
  : Scene()
{
  setWorld(world);

  // For test
  auto box = std::make_shared<Box>();
  addEntity(box);
}

//==============================================================================
void WorldScene::setWorld(simulation::WorldPtr world)
{
  if (world == mWorld)
    return;

  mWorld = std::move(world);
}

//==============================================================================
simulation::WorldPtr WorldScene::getWorld()
{
  return mWorld;
}

//==============================================================================
simulation::ConstWorldPtr WorldScene::getWorld() const
{
  return mWorld;
}

} // namespace glfw
} // namespace gui
} // namespace dart
