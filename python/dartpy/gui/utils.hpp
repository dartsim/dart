#pragma once

#include <osg/ref_ptr>

#include <memory>
#include <utility>

namespace dart::python_nb {

/// Shared_ptr wrapper that cooperates with osg::ref_ptr reference counting.
template <typename T>
std::shared_ptr<T> makeOsgShared(T* raw)
{
  if (!raw) {
    return nullptr;
  }
  raw->ref();
  return std::shared_ptr<T>(raw, [](T* ptr) {
    if (ptr) {
      ptr->unref();
    }
  });
}

template <typename T, typename... Args>
std::shared_ptr<T> makeOsgShared(Args&&... args)
{
  return makeOsgShared(new T(std::forward<Args>(args)...));
}

template <typename T>
std::shared_ptr<T> osgRefToShared(const ::osg::ref_ptr<T>& ref)
{
  if (!ref) {
    return nullptr;
  }
  ref->ref();
  return std::shared_ptr<T>(ref.get(), [](T* ptr) {
    if (ptr) {
      ptr->unref();
    }
  });
}

template <typename T>
::osg::ref_ptr<T> sharedToOsgRef(const std::shared_ptr<T>& ptr)
{
  if (!ptr) {
    return {};
  }
  return ::osg::ref_ptr<T>(ptr.get());
}

} // namespace dart::python_nb
