#include "dart/gui/glfw/Entity.hpp"

#include "dart/common/Console.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
Entity::Entity(Scene* scene, const Eigen::Isometry3f& tf)
  : mScene(scene), mTransform(tf)
{
  // Do nothing
}

//==============================================================================
Scene* Entity::getScene()
{
  return mScene;
}

//==============================================================================
const Scene* Entity::getScene() const
{
  return mScene;
}

//==============================================================================
void Entity::setDrawable(const std::shared_ptr<Drawable>& /*drawable*/)
{
  // Do nothing
}

//==============================================================================
const Eigen::Isometry3f& Entity::getTransform() const
{
  return mTransform;
}

//==============================================================================
const Eigen::Vector3f Entity::getTranslation() const
{
  return mTransform.translation();
}

//==============================================================================
void Entity::translateWorld(const Eigen::Vector3f& v)
{
  mTransform = Eigen::Translation3f(v) * mTransform;
}

//==============================================================================
void Entity::translateLocal(const Eigen::Vector3f& v)
{
  mTransform = mTransform * Eigen::Translation3f(v);
}

//==============================================================================
void Entity::rotateWorld(const Eigen::Vector3f& axis, float angle)
{
  mTransform = Eigen::AngleAxisf(angle, axis) * mTransform;
}

//==============================================================================
void Entity::rotateLocal(const Eigen::Vector3f& axis, float angle)
{
  mTransform = mTransform * Eigen::AngleAxisf(angle, axis);
}

//==============================================================================
void Entity::rotateAroundWorldPoint(
    const Eigen::Vector3f& axis, float angle, const Eigen::Vector3f& worldPoint)
{
  mTransform = Eigen::Translation3f(worldPoint) * Eigen::AngleAxisf(angle, axis)
               * Eigen::Translation3f(-worldPoint) * mTransform;
}

//==============================================================================
void Entity::rotateAroundLocalPoint(
    const Eigen::Vector3f& axis, float angle, const Eigen::Vector3f& worldPoint)
{
  // Convert the world point into the local coordinate system
  Eigen::Vector3f localPoint = mTransform.inverse() * worldPoint;

  mTransform = mTransform * Eigen::Translation3f(localPoint)
               * Eigen::AngleAxisf(angle, axis)
               * Eigen::Translation3f(-localPoint);
}

//==============================================================================
void Entity::createGlObjectsFor(GLFWwindow* /*window*/, GLFWwindow* /*sharing*/)
{
  // Do nothing
}

//==============================================================================
void Entity::destroyGlObjectsFor(
    GLFWwindow* /*window*/, GLFWwindow* /*sharing*/)
{
  // Do nothing
}

//==============================================================================
void Entity::render(
    GLFWwindow* /*window*/,
    GLFWwindow* /*sharing*/,
    Program& /*program*/,
    const Eigen::Isometry3d& /*worldToCameraMatrix*/)
{
  // TODO(JS): Not implemented
}

//==============================================================================
void Entity::render(
    Program& /*program*/, const Eigen::Isometry3f& /*worldToCameraMatrix*/)
{
  // Do nothing
}

} // namespace glfw
} // namespace gui
} // namespace dart
