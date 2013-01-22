// URDF exceptions
#ifndef URDF_INTERFACE_EXCEPTION_H_
#define URDF_INTERFACE_EXCEPTION_H_

#include <string>
#include <stdexcept>

namespace urdf
{

class ParseError: public std::runtime_error
{
public:
  ParseError(const std::string &error_msg) : std::runtime_error(error_msg) {};
};

}

#endif
