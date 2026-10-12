#pragma once

#include <dart/dynamics/HierarchicalIK.hpp>

namespace dartnb {
/// Constructs a HierarchicalIK wrapper that also keeps its Skeleton alive,
/// which HierarchicalIK holds only weakly. The wrapper stays the IK's only
/// owner, so the garbage collector can traverse it.
template <class T>
struct HierarchicalIKOwner
{
  std::shared_ptr<T> ik;

  std::shared_ptr<T> get_shared() const
  {
    return ik;
  }
  T* get() const
  {
    return ik.get();
  }
  dart::dynamics::SkeletonPtr retained() const
  {
    return ik->getSkeleton();
  }
};

template <class T>
nb::object hierarchicalIKInstance(const std::shared_ptr<T>& ik)
{
  if (!ik)
    return nb::none();
  bool is_new = false;
  auto result = nb::steal<nb::object>(wrap(
      typeid(T),
      typeid(*ik),
      complete_address(ik.get()),
      ik.get(),
      nb::rv_policy::reference,
      {},
      &is_new));
  retain_shared_owner(result, is_new, ik, complete_address(ik.get()));
  if (is_new)
    // As in nativeInstance: the Skeleton only, never a second IK owner.
    keepAlive(result, HierarchicalIKOwner<T>{ik}.retained());
  return result;
}
} // namespace dartnb
