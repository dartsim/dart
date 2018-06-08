#ifndef DART_GUI_GLFW_FRAGMENTSHADER_HPP_
#define DART_GUI_GLFW_FRAGMENTSHADER_HPP_

#include <string>
#include <Eigen/Dense>

#include "dart/gui/glfw/LoadGlfw.hpp"

namespace dart {
namespace gui {

class FragmentShader final
{
public:
  FragmentShader(const std::string& vertexVertexShaderFilepath = "");
  // TODO: remove the default parameters
  // TODO: change to URI

  ~FragmentShader();

  GLuint getId() const;

protected:
  std::string mFragmentShaderFilepath;

  GLuint mFragmentShaderID;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_FRAGMENTSHADER_HPP_
