#ifndef DART_GUI_SHADER_HPP_
#define DART_GUI_SHADER_HPP_

#include <string>

#include "dart/common/ResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/gui/glfw/LoadGlfw.hpp"

namespace dart {
namespace gui {

template <GLuint ShaderType>
class Shader
{
public:
  /// Constructs Shader from string.
  explicit Shader(const char* shaderString);

  /// Constructs Shader from std::string.
  explicit Shader(const std::string& shaderString);

  /// Constructs Shader from URI.
  explicit Shader(
      const common::Uri& shaderUri,
      common::ResourceRetriever* retriever = nullptr);

  /// Destructor
  virtual ~Shader();

  /// Returns the OpenGL shader ID.
  GLuint getId() const;

protected:
  /// OpenGL shader ID.
  GLuint mShaderId;

private:
  /// Compiles a OpenGL shader given string.
  void compile(const char* shaderString);
};

using VertexShader = Shader<GL_VERTEX_SHADER>;
using FragmentShader = Shader<GL_FRAGMENT_SHADER>;

} // namespace gui
} // namespace dart

#include "dart/gui/detail/Shader-impl.hpp"

#endif // DART_GUI_SHADER_HPP_
