#pragma once

#include <dart/dynamics/Frame.hpp>

namespace dartnb {

inline dart::dynamics::Frame* frameOrWorld(dart::dynamics::Frame* frame)
{
  return frame ? frame : dart::dynamics::Frame::World();
}

inline const dart::dynamics::Frame* frameOrWorld(
    const dart::dynamics::Frame* frame)
{
  return frame ? frame : dart::dynamics::Frame::World();
}

} // namespace dartnb
