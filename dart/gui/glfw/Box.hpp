#ifndef DART_GUI_GLFW_BOX_HPP_
#define DART_GUI_GLFW_BOX_HPP_

#include <unordered_map>

#include <Eigen/Dense>

#include "dart/common/SmartPointer.hpp"
#include "dart/gui/VertexArrayObject.hpp"
#include "dart/gui/VertexBufferObject.hpp"
#include "dart/gui/glfw/Entity.hpp"

namespace dart {
namespace gui {
namespace glfw {

class Box : public Entity
{
public:
  virtual ~Box();

  void render(
      Shader& shader, const Eigen::Isometry3f& worldToCameraMatrix) override;

protected:
  Box(Scene* scene = nullptr,
      const Eigen::Vector3f& size = Eigen::Vector3f::Constant(0.5f),
      const Eigen::Isometry3f& tf = Eigen::Isometry3f::Identity());

  Box(Scene* scene,
      float sizeX,
      float sizeY,
      float sizeZ,
      const Eigen::Isometry3f& tf = Eigen::Isometry3f::Identity());

  void createGlObjectsFor(
      GLFWwindow* window, GLFWwindow* sharing = nullptr) override;
  void destroyGlObjectsFor(
      GLFWwindow* window, GLFWwindow* sharing = nullptr) override;

protected:
  Eigen::Vector3f mHalfSize;

  Eigen::Matrix4f mScalingMatrix;

  struct SharableGlObjects
  {
    size_t numEntities;

    /// Vertex Buffer Object for the vertices data used to render the box with
    /// OpenGL
    std::unique_ptr<BufferObject> vboVertices;

    /// Vertex Buffer Object for the normales used to render the box with OpenGL
    std::unique_ptr<BufferObject> vboNormals;
  };

  struct UnsharableGlObjects
  {
    size_t numEntities;

    /// Vertex Array Object for the vertex data
    std::unique_ptr<VertexArrayObject> vao;
  };

  static std::unordered_map<GLFWwindow*, SharableGlObjects> mSharableGlObjects;
  static std::unordered_map<GLFWwindow*, UnsharableGlObjects>
      mUnsharableGlObjects;

  /// Vertex Buffer Object for the vertices data used to render the box with
  /// OpenGL
  BufferObject* mVboVertices;

  /// Vertex Buffer Object for the normales used to render the box with OpenGL
  BufferObject* mVboNormals;

  /// Vertex Array Object for the vertex data
  VertexArrayObject* mVao;

private:
  void createGlObjectsForMainViewer(GLFWwindow* window);
  void destroyGlObjectsForMainViewer(GLFWwindow* window);
  void createGlObjectsForSubViewer(GLFWwindow* window, GLFWwindow* sharing);
  void destroyGlObjectsForSubViewer(GLFWwindow* window, GLFWwindow* sharing);
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_BOX_HPP_
