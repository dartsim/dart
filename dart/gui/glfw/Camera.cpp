#include "dart/gui/glfw/Camera.hpp"

#include "dart/math/Constants.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
Camera::Camera() : Entity()
{
  // Set default values
  mFieldOfView = 45.0f;
  mSceneRadius = 1.0f;
  mNearPlane = 0.1f;
  mFarPlane = 10.0f;
  mWidth = 1;
  mHeight = 1;

  // Update the projection matrix
  updateProjectionMatrix();
}

//==============================================================================
Camera::~Camera()
{
  // Do nothing
}

//==============================================================================
const Eigen::Matrix4f& Camera::getProjectionMatrix() const
{
  return mProjectionMatrix;
}

//==============================================================================
void Camera::setDimensions(uint width, uint height)
{
  mWidth = width;
  mHeight = height;
  updateProjectionMatrix();
}

//==============================================================================
float Camera::getSceneRadius() const
{
  return mSceneRadius;
}

//==============================================================================
void Camera::setSceneRadius(float radius)
{
  mSceneRadius = radius;
  setClippingPlanes(0.01f * radius, 10.0f * radius);
}

//==============================================================================
void Camera::setClippingPlanes(float near, float far)
{
  mNearPlane = near;
  mFarPlane = far;
  updateProjectionMatrix();
}

//==============================================================================
void Camera::setFieldOfView(float fov)
{
  mFieldOfView = fov;
  updateProjectionMatrix();
}

//==============================================================================
void Camera::setZoom(float fraction)
{
  const Eigen::Vector3f zoomVector(0, 0, mSceneRadius * fraction);
  translateLocal(zoomVector);
}

//==============================================================================
void Camera::translateCamera(
    float dx, float dy, const Eigen::Vector3f& worldPoint)
{
  // Transform the world point into camera coordinates
  Eigen::Vector3f pointCamera = mTransform.inverse() * worldPoint;

  // Get the depth
  float z = -pointCamera[2];

  // Find the scaling of dx and dy from windows coordinates to near plane
  // coordinates
  // and from there to camera coordinates at the object's depth
  float aspect = float(mWidth) / float(mHeight);
  float top = mNearPlane * tan(mFieldOfView * math::constantsf::pi() / 360.0f);
  float right = top * aspect;

  // Translate the camera
  translateLocal(
      Eigen::Vector3f(
          2.0f * dx * right / mNearPlane * z,
          -2.0f * dy * top / mNearPlane * z,
          0.0f));
}

//==============================================================================
float Camera::getNearClippingPlane() const
{
  return mNearPlane;
}

//==============================================================================
float Camera::getFarClippingPlane() const
{
  return mFarPlane;
}

//==============================================================================
uint Camera::getWidth() const
{
  return mWidth;
}

//==============================================================================
uint Camera::getHeight() const
{
  return mHeight;
}

//==============================================================================
void Camera::updateProjectionMatrix()
{
  // Compute the aspect ratio
  const float aspect = float(mWidth) / float(mHeight);

  const float top
      = mNearPlane
        * std::tan((mFieldOfView / 2.0f) * (math::constantsf::pi() / 180.0f));
  const float bottom = -top;
  const float left = bottom * aspect;
  const float right = top * aspect;

  const float fx = 2.0f * mNearPlane / (right - left);
  const float fy = 2.0f * mNearPlane / (top - bottom);
  const float fz = -(mFarPlane + mNearPlane) / (mFarPlane - mNearPlane);
  const float fw = -2.0f * mFarPlane * mNearPlane / (mFarPlane - mNearPlane);

  // Recompute the projection matrix
  mProjectionMatrix.setZero();
  mProjectionMatrix(0, 0) = fx;
  mProjectionMatrix(1, 1) = fy;
  mProjectionMatrix(2, 2) = fz;
  mProjectionMatrix(2, 3) = fw;
  mProjectionMatrix(3, 2) = -1;
}

} // namespace glfw
} // namespace gui
} // namespace dart
