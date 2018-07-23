#ifndef DART_GUI_SHADERCODE_HPP_
#define DART_GUI_SHADERCODE_HPP_

#include <string>

#include "dart/common/ResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/gui/LoadOpengl.hpp"

namespace dart {
namespace gui {

template <GLuint ShaderType_>
class ShaderCode final
{
public:
  constexpr static GLuint ShaderType = ShaderType_;

  /// Constructs Shader from std::string.
  explicit ShaderCode(const char* shaderString);

  /// Constructs Shader from std::string.
  explicit ShaderCode(const std::string& shaderString);

  /// Constructs Shader from URI.
  explicit ShaderCode(
      const common::Uri& shaderUri,
      common::ResourceRetriever* retriever = nullptr);

  /// Destructor
  ~ShaderCode() = default;

  /// Returns code in const char*
  const char* getCString() const;

  /// Returns code in const std::string
  const std::string& getString() const;

private:
  /// Shader string
  std::string mCode;
};

using VertexShaderCode = ShaderCode<GL_VERTEX_SHADER>;
using FragmentShaderCode = ShaderCode<GL_FRAGMENT_SHADER>;

} // namespace gui
} // namespace dart

#include "dart/gui/detail/ShaderCode-impl.hpp"

#endif // DART_GUI_SHADERCODE_HPP_
