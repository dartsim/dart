#ifndef DART_GUI_GLFW_DETAIL_VIEWER_HPP_
#define DART_GUI_GLFW_DETAIL_VIEWER_HPP_

#include "dart/gui/glfw/Viewer.hpp"

#include "dart/common/Memory.hpp"
#include "dart/gui/glfw/Scene.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
template <typename SceneT, typename... Args>
Scene* Viewer::createScene(Args&&... args)
{
  if (mScene)
    mScene->notifyMainWindowChanged(nullptr);

  mScene = common::make_unique<SceneT>(std::forward<Args>(args)...);

  return mScene.get();
}

//==============================================================================
template <typename SceneT>
SceneT* Viewer::getSceneAs() const
{
  assert(nullptr != std::dynamic_pointer_cast<SceneT>(mScene));

  return static_cast<SceneT*>(mScene.get());
}

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // MDART_GUI_GLFW_DETAIL_VIEWER_HPP_
