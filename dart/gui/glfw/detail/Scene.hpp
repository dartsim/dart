#ifndef DART_GUI_GLFW_DETAIL_SCENE_HPP_
#define DART_GUI_GLFW_DETAIL_SCENE_HPP_

#include "dart/gui/glfw/Scene.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
template <typename EntityT, typename... Args>
std::shared_ptr<Entity> Scene::createEntity(const Args&... args)
{
  auto newEntity = EntityT::createShared(this, args...);
  addEntity(newEntity);

  return newEntity;
}

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_DETAIL_SCENE_HPP_
