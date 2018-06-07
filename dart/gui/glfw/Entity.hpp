#ifndef DART_GUI_GLFW_ENTITY_HPP_
#define DART_GUI_GLFW_ENTITY_HPP_

#include <memory>
#include <Eigen/Dense>
#include "dart/gui/glfw/LoadGlfw.hpp"
#include "dart/gui/glfw/Shader.hpp"

namespace dart {

class Drawable;
class Scene;

class Entity
{
public:
  friend class Scene;
  friend class Viewer;
  // TODO: remove once we create Drawable class

  Entity(
      Scene* scene = nullptr,
      const Eigen::Isometry3f& tf = Eigen::Isometry3f::Identity());
  // TODO: scene shouldn't be nullptr, remove the default value once camera and
  // light are moved from viewer to scene.

  virtual ~Entity() = default;

  /// Returns Scene this Entity belongs to.
  Scene* getScene();

  /// Returns const Scene this Entity belongs to.
  const Scene* getScene() const;

  void setDrawable(const std::shared_ptr<Drawable>& drawable);

  const Eigen::Isometry3f& getTransform() const;

  const Eigen::Vector3f getTranslation() const;

  /// Translate the object in world-space
  void translateWorld(const Eigen::Vector3f& v);
  // TODO(JS): Use Frame* instead

  /// Translate the object in local-space
  void translateLocal(const Eigen::Vector3f& v);
  // TODO(JS): Use Frame* instead

  /// Rotate the object in world-space
  void rotateWorld(const Eigen::Vector3f& axis, float angle);
  // TODO(JS): Use Frame* instead

  /// Rotate the object in local-space
  void rotateLocal(const Eigen::Vector3f& axis, float angle);
  // TODO(JS): Use Frame* instead

  /// Rotate around a world-space point
  void rotateAroundWorldPoint(
      const Eigen::Vector3f& axis,
      float angle,
      const Eigen::Vector3f& worldPoint);

  /// Rotate around a local-space point
  void rotateAroundLocalPoint(
      const Eigen::Vector3f& axis,
      float angle,
      const Eigen::Vector3f& localPoint);

protected:
  virtual void createGlObjectsFor(
      GLFWwindow* window, GLFWwindow* sharing = nullptr);
  virtual void destroyGlObjectsFor(
      GLFWwindow* window, GLFWwindow* sharing = nullptr);

  virtual void render(
      GLFWwindow* window,
      GLFWwindow* sharing,
      Shader& shader,
      const Eigen::Isometry3d& worldToCameraMatrix);

  virtual void render(
      Shader& shader, const Eigen::Isometry3f& worldToCameraMatrix);

protected:
  Scene* mScene;
  std::shared_ptr<Drawable> mDrawable;
  Eigen::Isometry3f mTransform;
};
// TODO: make this class pure virtual class
// TODO: seperate Entity into Node and Drawable

} // namespace dart

#endif // DART_GUI_GLFW_ENTITY_HPP_
