#ifndef DART_GUI_GLFW_CAMERA_HPP_
#define DART_GUI_GLFW_CAMERA_HPP_

#include <Eigen/Dense>
#include "dart/gui/glfw/Entity.hpp"

namespace dart {
namespace gui {
namespace glfw {

class Camera : public Entity
{
public:
  // Constructor
  Camera();

  // Destructor
  ~Camera();

  // Get the projection matrix
  const Eigen::Matrix4f& getProjectionMatrix() const;

  // Set the dimensions of the camera
  void setDimensions(uint width, uint height);

  // Get the radius of the scene the camera should capture
  float getSceneRadius() const;

  // Set the radius of the scene the camera should capture
  // This will update the clipping planes accordingly
  void setSceneRadius(float radius);

  // Set the clipping planes
  void setClippingPlanes(float near, float far);

  // Set the field of view
  void setFieldOfView(float fov);

  // Set the zoom of the camera (a fraction between 0 and 1)
  void setZoom(float fraction);

  // Translate the camera go a given point using the dx, dy fraction
  void translateCamera(float dx, float dy, const Eigen::Vector3f& worldPoint);

  // Get the near clipping plane
  float getNearClippingPlane() const;

  // Get the far clipping plane
  float getFarClippingPlane() const;

  // Get the width
  uint getWidth() const;

  // Get the height
  uint getHeight() const;

protected:
  // Update the projection matrix
  void updateProjectionMatrix();

protected:
  // Field of view
  float mFieldOfView;

  // Radius of the scene
  float mSceneRadius;

  // Near plane
  float mNearPlane;

  // Far plane
  float mFarPlane;

  // Width of the camera
  uint mWidth;

  // Height of the camera
  uint mHeight;

  // Projection matrix
  Eigen::Matrix4f mProjectionMatrix;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_CAMERA_HPP_
