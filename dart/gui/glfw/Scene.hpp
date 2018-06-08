#ifndef DART_GUI_GLFW_SCENE_HPP_
#define DART_GUI_GLFW_SCENE_HPP_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "dart/gui/Program.hpp"
#include "dart/gui/VertexArrayObject.hpp"
#include "dart/gui/VertexBufferObject.hpp"
#include "dart/gui/glfw/Camera.hpp"
#include "dart/gui/glfw/Light.hpp"

namespace dart {
namespace gui {
namespace glfw {

class Entity;
class Viewer;

// TODO: create base class and rename this to MainScene (or something else)
class Scene
{
public:
  friend class Viewer;

  Scene(
      const std::string& name = "Default Scene", GLFWwindow* window = nullptr);
  // TODO: make this protected

  virtual ~Scene();

  template <typename EntityT, typename... Args>
  std::shared_ptr<Entity> createEntity(const Args&... args);

  void addEntity(const std::shared_ptr<Entity>& entity);

  void removeEntity(const std::shared_ptr<Entity>& entity);

  void removeAllEntities();

  //  const std::vector<std::shared_ptr<Entity>>& getEntities() const;

  virtual void notifyMainWindowChanged(GLFWwindow* window);
  virtual void notifySubWindowAdded(GLFWwindow* window);
  virtual void notifySubWindowRemoved(GLFWwindow* window);

  virtual void renderSinglePass(
      Program& shader, const Eigen::Isometry3f& worldToCameraMatrix);

protected:
  std::string mName;

  GLFWwindow* mGlfwWindow;

  std::vector<GLFWwindow*> mGlfwSubWindows;

  std::unordered_set<std::shared_ptr<Entity>> mEntities;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#include "dart/gui/glfw/detail/Scene.hpp"

#endif // DART_GUI_GLFW_SCENE_HPP_
