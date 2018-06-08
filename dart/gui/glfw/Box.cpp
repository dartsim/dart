#include "dart/gui/glfw/Box.hpp"

#include "dart/math/Helpers.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
std::unordered_map<GLFWwindow*, Box::SharableGlObjects> Box::mSharableGlObjects;
  std::unordered_map<GLFWwindow*, Box::UnsharableGlObjects>
    Box::mUnsharableGlObjects;

// clang-format off

/// Vertices coordinates of the triangles of the box
static GLfloat mCubeVertices[108] = {
    -1.0f,-1.0f,-1.0f, // triangle 1 : begin
    -1.0f,-1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f, // triangle 1 : end
    1.0f, 1.0f,-1.0f, // triangle 2 : begin
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f, // triangle 2 : end
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f
};

  /// Vertices normals of the triangles of the box
static GLfloat mCubeNormals[108] = {
    -1.0f, 0.0f, 0.0f, // triangle 1 : begin
    -1.0f, 0.0f, 0.0f,
    -1.0f, 0.0f, 0.0f, // triangle 1 : end
    0.0f, 0.0f,-1.0f, // triangle 2 : begin
    0.0f, 0.0f,-1.0f,
    0.0f, 0.0f,-1.0f, // triangle 2 : end
    0.0f,-1.0f, 0.0f,
    0.0f,-1.0f, 0.0f,
    0.0f,-1.0f, 0.0f,//
    0.0f, 0.0f,-1.0f,
    0.0f, 0.0f,-1.0f,
    0.0f, 0.0f,-1.0f,//
    -1.0f, 0.0f, 0.0f,
    -1.0f, 0.0f, 0.0f,
    -1.0f, 0.0f,0.0f,//
    0.0f,-1.0f, 0.0f,
    0.0f,-1.0f, 0.0f,
    0.0f,-1.0f, 0.0f,//
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f,//
    1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,//
    1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,//
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,//
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,//
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f//
};

// clang-format on

//==============================================================================
Box::Box(Scene* scene, const Eigen::Vector3f& size, const Eigen::Isometry3f& tf)
  : Entity(scene, tf),
    mHalfSize(size),
    mScalingMatrix(
        Eigen::Vector4f(mHalfSize[0], mHalfSize[1], mHalfSize[2], 1.0)
            .asDiagonal()),
    mVboVertices(nullptr),
    mVboNormals(nullptr),
    mVao(nullptr)
{
  mTransform = mTransform * mScalingMatrix;
}

//==============================================================================
Box::Box(
    Scene* scene,
    float sizeX,
    float sizeY,
    float sizeZ,
    const Eigen::Isometry3f& tf)
  : Entity(scene, tf),
    mHalfSize(Eigen::Vector3f(sizeX, sizeY, sizeZ)),
    mScalingMatrix(
        Eigen::Vector4f(mHalfSize[0], mHalfSize[1], mHalfSize[2], 1.0)
            .asDiagonal()),
    mVboVertices(nullptr),
    mVboNormals(nullptr),
    mVao(nullptr)
{
  mTransform = mTransform * mScalingMatrix;
}

//==============================================================================
Box::~Box()
{
  // Do nothing
}

//==============================================================================
void Box::render(Shader& shader, const Eigen::Isometry3f& worldToCameraMatrix)
{
  // Bind the VAO
  mVao->bind();

  // Bind the shader
  shader.bind();

  mVboVertices->bind();

  // Set the model to camera matrix
  shader.setMatrix4x4Uniform(std::string("localToWorldMatrix"), mTransform);
  shader.setMatrix4x4Uniform(
      std::string("worldToCameraMatrix"), worldToCameraMatrix);

  // Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix
  // of the model-view matrix)
  const Eigen::Isometry3f localToCameraMatrix
      = worldToCameraMatrix * mTransform;
  const Eigen::Matrix3f normalMatrix
      = localToCameraMatrix.linear().inverse().transpose();
  shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

  // Set the vertex color
  Eigen::Vector4f color = Color::Orange(1.0f);
  shader.setVector4Uniform("vertexColor", color, false);

  // Get the location of shader attribute variables
  GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
  GLint vertexNormalLoc = shader.getAttribLocation("vertexNormal", false);

  glEnableVertexAttribArray(vertexPositionLoc);
  glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, NULL);

  mVboNormals->bind();

  if (vertexNormalLoc != -1)
    glEnableVertexAttribArray(vertexNormalLoc);
  if (vertexNormalLoc != -1)
    glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, 0, NULL);

  // Draw the geometry of the box
  glDrawArrays(GL_TRIANGLES, 0, 36);

  glDisableVertexAttribArray(vertexPositionLoc);
  if (vertexNormalLoc != -1)
    glDisableVertexAttribArray(vertexNormalLoc);

  mVboNormals->unbind();
  mVboVertices->unbind();

  // Unbind the VAO
  mVao->unbind();

  // Unbind the shader
  shader.unbind();
}

//==============================================================================
void Box::createGlObjectsFor(GLFWwindow* window, GLFWwindow* sharing)
{
  if (nullptr == sharing)
  {
    createGlObjectsForMainViewer(window);
    createGlObjectsForSubViewer(window, window);
  }
  else
  {
    createGlObjectsForSubViewer(window, sharing);
  }
}

//==============================================================================
void Box::destroyGlObjectsFor(GLFWwindow* window, GLFWwindow* sharing)
{
  if (nullptr == sharing)
  {
    destroyGlObjectsForMainViewer(window);
    destroyGlObjectsForSubViewer(window, window);
  }
  else
  {
    destroyGlObjectsForSubViewer(window, sharing);
  }
}

//==============================================================================
void Box::createGlObjectsForMainViewer(GLFWwindow* window)
{
  assert(nullptr != window);

  auto pair
      = mSharableGlObjects.insert(std::make_pair(window, SharableGlObjects()));

  auto& sharableGlObjects = pair.first->second;
  sharableGlObjects.numEntities++;

  if (pair.second)
  {
    // If the sharable GL objects are newly created for the Viewer, this Entity
    // should be the first one in the Viewer.
    assert(1u == sharableGlObjects.numEntities);

    // Create the VBO for the vertices data
    sharableGlObjects.vboVertices.reset(new BufferObject(GL_ARRAY_BUFFER));
    sharableGlObjects.vboVertices->bind();
    sharableGlObjects.vboVertices->copyDataIntoBO(
        sizeof(mCubeVertices), mCubeVertices, GL_STATIC_DRAW);
    sharableGlObjects.vboVertices->unbind();

    // Create th VBO for the normals data
    sharableGlObjects.vboNormals.reset(new BufferObject(GL_ARRAY_BUFFER));
    sharableGlObjects.vboNormals->bind();
    sharableGlObjects.vboNormals->copyDataIntoBO(
        sizeof(mCubeNormals), mCubeNormals, GL_STATIC_DRAW);
    sharableGlObjects.vboNormals->unbind();
  }

  mVboVertices = sharableGlObjects.vboVertices.get();
  mVboNormals = sharableGlObjects.vboNormals.get();
}

//==============================================================================
void Box::destroyGlObjectsForMainViewer(GLFWwindow* window)
{
  assert(nullptr != window);

  auto found = mSharableGlObjects.find(window);
  assert(found != mSharableGlObjects.end());

  auto& sharableGlObjects = found->second;
  sharableGlObjects.numEntities--;

  // If this Entity is the last one in the main viewer, then destroy the OpenGL
  // objects.
  if (0u == sharableGlObjects.numEntities)
    mSharableGlObjects.erase(window);

  mVboVertices = nullptr;
  mVboNormals = nullptr;
}

//==============================================================================
void Box::createGlObjectsForSubViewer(GLFWwindow* window, GLFWwindow* sharing)
{
  assert(nullptr != sharing);
  assert(nullptr != window);

  auto found = mSharableGlObjects.find(sharing);
  assert(mSharableGlObjects.end() != found);
  auto& sharableGlObjects = found->second;

  auto unsharablePair = mUnsharableGlObjects.insert(
      std::make_pair(window, UnsharableGlObjects()));

  auto& unsharableGlObjects = unsharablePair.first->second;
  unsharableGlObjects.numEntities++;

  if (unsharablePair.second)
  {
    unsharableGlObjects.vao.reset(new VertexArrayObject());

    // Create the VAO for both VBOs
    unsharableGlObjects.vao->bind();

    // Bind the VBO of vertices
    sharableGlObjects.vboVertices->bind();

    // Bind the VBO of indices
    sharableGlObjects.vboNormals->bind();

    // Unbind the VAO
    unsharableGlObjects.vao->unbind();
  }

  mVao = unsharableGlObjects.vao.get();
}

//==============================================================================
void Box::destroyGlObjectsForSubViewer(GLFWwindow* window, GLFWwindow* sharing)
{
  DART_GUI_GLFW_UNUSED(sharing);
  assert(nullptr != sharing);
  assert(nullptr != window);

  auto found = mUnsharableGlObjects.find(window);

  assert(found != mUnsharableGlObjects.end());

  auto& unsharableGlObjects = found->second;
  unsharableGlObjects.numEntities--;

  if (0u == unsharableGlObjects.numEntities)
    mUnsharableGlObjects.erase(window);

  mVao = nullptr;
}

} // namespace glfw
} // namespace gui
} // namespace dart
