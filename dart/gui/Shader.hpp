#ifndef DART_GUI_GLFW_SHADER_HPP_
#define DART_GUI_GLFW_SHADER_HPP_

#include <string>
#include <Eigen/Dense>

#include "dart/common/ResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/gui/glfw/LoadGlfw.hpp"

namespace dart {
namespace gui {

class Shader
{
public:
  Shader(
      GLuint type,
      const common::Uri& shaderUri,
      common::ResourceRetriever* retriever = nullptr);
  // TODO: remove the default parameters
  // TODO: change to URI

  virtual ~Shader();

  GLuint getId() const;

protected:
  GLuint mShaderId;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_SHADER_HPP_
