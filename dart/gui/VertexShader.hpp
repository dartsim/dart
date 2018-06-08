#ifndef DART_GUI_GLFW_VERTEXSHADER_HPP_
#define DART_GUI_GLFW_VERTEXSHADER_HPP_

#include <string>
#include <Eigen/Dense>

#include "dart/gui/glfw/LoadGlfw.hpp"

namespace dart {
namespace gui {

class VertexShader final
{
public:
  VertexShader(const std::string& vertexShaderFilepath = "");
  // TODO: remove the default parameters
  // TODO: change to URI

  ~VertexShader();

  GLuint getId() const;

protected:
  std::string mVertexShaderFilepath;

  GLuint mVertexShaderId;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_VERTEXSHADER_HPP_
