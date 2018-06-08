#ifndef DART_GUI_GLFW_LIGHT_HPP_
#define DART_GUI_GLFW_LIGHT_HPP_

#include "dart/gui/LoadOpengl.hpp"
#include "dart/gui/glfw/Entity.hpp"
#include "dart/math/Helpers.hpp"

namespace dart {
namespace gui {
namespace glfw {

class Light final : public Entity
{
public:
  // Constructor
  Light(
      const Eigen::Vector4f& diffuseColor = Eigen::Vector4f::Ones(),
      const Eigen::Vector4f& specularColor = Eigen::Vector4f::Ones(),
      bool active = true);

  // Destructor
  virtual ~Light();

  Eigen::Vector4f getDiffuseColor() const;

  void setDiffuseColor(const Eigen::Vector4f& color);

  Eigen::Vector4f getSpecularColor() const;

  void setSpecularColor(const Eigen::Vector4f& color);

  bool isActive() const;

  void init();

  void enable();

  void disable();

protected:
  GLuint mLightID;

  Eigen::Vector4f mDiffuseColor;

  Eigen::Vector4f mSpecularColor;

  bool mIsActive;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_LIGHT_HPP_
