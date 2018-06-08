#include "dart/gui/glfw/Light.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
Light::Light(
    const Eigen::Vector4f& diffuseColor,
    const Eigen::Vector4f& specularColor,
    bool active)
  : mDiffuseColor(diffuseColor),
    mSpecularColor(specularColor),
    mIsActive(active)
{
  // Do nothing
}

//==============================================================================
Light::~Light()
{
  // Do nothing
}

//==============================================================================
Eigen::Vector4f Light::getDiffuseColor() const
{
  return mDiffuseColor;
}

//==============================================================================
void Light::setDiffuseColor(const Eigen::Vector4f& color)
{
  mDiffuseColor = color;
}

//==============================================================================
Eigen::Vector4f Light::getSpecularColor() const
{
  return mSpecularColor;
}

//==============================================================================
void Light::setSpecularColor(const Eigen::Vector4f& color)
{
  mSpecularColor = color;
}

//==============================================================================
bool Light::isActive() const
{
  return mIsActive;
}

//==============================================================================
void Light::init()
{
  enable();
}

//==============================================================================
void Light::enable()
{
  mIsActive = true;

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0 + mLightID);
}

//==============================================================================
void Light::disable()
{
  mIsActive = false;

  glDisable(GL_LIGHT0 + mLightID);
}

} // namespace glfw
} // namespace gui
} // namespace dart
