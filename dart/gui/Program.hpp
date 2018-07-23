#ifndef DART_GUI_GLFW_PROGRAM_HPP_
#define DART_GUI_GLFW_PROGRAM_HPP_

#include <string>
#include <unordered_map>
#include <Eigen/Dense>

#include "dart/gui/ShaderCode.hpp"
#include "dart/gui/glfw/LoadGlfw.hpp"

namespace dart {
namespace gui {

class Program final
{
public:
  /// Constructor
  Program(
      const VertexShaderCode& vertexShader,
      const FragmentShaderCode& fragmentShader);
  // TODO: remove the default parameters

  /// Destructor
  ~Program() = default;

  void bind();

  void unbind();

  GLint getUniformLocation(
      const std::string& variableName, bool errorIfMissing = true) const;

  GLint getAttribLocation(
      const std::string& variableName, bool errorIfMissing = true) const;

  void setFloatUniform(
      const std::string& variableName,
      float value,
      bool errorIfMissing = true) const;

  void setIntUniform(
      const std::string& variableName,
      int value,
      bool errorIfMissing = true) const;

  void setVector2Uniform(
      const std::string& variableName,
      const Eigen::Vector2f& v,
      bool errorIfMissing = true) const;

  void setVector3Uniform(
      const std::string& variableName,
      const Eigen::Vector3f& v,
      bool errorIfMissing = true) const;

  void setVector4Uniform(
      const std::string& variableName,
      const Eigen::Vector4f& v,
      bool errorIfMissing = true) const;

  void setMatrix3x3Uniform(
      const std::string& variableName,
      const float* matrix,
      bool transpose = false,
      bool errorIfMissing = true) const;

  void setMatrix3x3Uniform(
      const std::string& variableName,
      const Eigen::Matrix3f& matrix,
      bool errorIfMissing = true) const;

  void setMatrix4x4Uniform(
      const std::string& variableName,
      const float* matrix,
      bool transpose = false,
      bool errorIfMissing = true) const;

  void setMatrix4x4Uniform(
      const std::string& variableName,
      const Eigen::Matrix4f& matrix,
      bool errorIfMissing = true) const;

  void setMatrix4x4Uniform(
      const std::string& variableName,
      const Eigen::Affine3f& matrix,
      bool errorIfMissing = true) const;

  void setMatrix4x4Uniform(
      const std::string& variableName,
      const Eigen::Isometry3f& matrix,
      bool errorIfMissing = true) const;

  GLint getUniform(const std::string& name) const;

  GLint getAttribute(const std::string& name) const;

//protected:
  /// Compiles a OpenGL shader given string.
  //
  // Dev Note: We intentionally have shader compiler functionality in this class
  // instead of ShaderCode. This is because compiling shader code and linking
  // program must be done in the same OpenGL context.
  // TODO(JS): Move this code to the implementation header.
  //
  template <typename ShaderCodeT>
  GLuint compileShader(const ShaderCodeT& code)
  {
    const GLuint shaderId = glCreateShader(ShaderCodeT::ShaderType);

    assert(shaderId != GL_INVALID_ENUM);
    assert(shaderId != 0);

    const char* c_str = code.getCString();
    glShaderSource(shaderId, 1, &c_str, nullptr);
    glCompileShader(shaderId);

    GLint compiled = GL_FALSE;
    glGetShaderiv(shaderId, GL_COMPILE_STATUS, &compiled);

    if (compiled == 0)
    {
      int logLength;
      glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &logLength);
      std::vector<GLchar> str(static_cast<std::size_t>(logLength));
      glGetShaderInfoLog(shaderId, logLength, nullptr, str.data());
      dterr << "Shader Error: " << std::string(str.begin(), str.end()) << "\n";
      assert(false);
      // TODO(JS): Needs better error handling
      return 0;
    }

//    glDeleteShader(shaderId);

    return shaderId;
  }

  void generateUniformLists();

  GLint addUniform(const std::string& name);

  GLint addAttribute(const std::string& name);

  GLuint mProgramId;

  std::unordered_map<std::string, GLint> mUniformLocationMap;

  std::unordered_map<std::string, GLint> mAttributeLocationMap;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_PROGRAM_HPP_
