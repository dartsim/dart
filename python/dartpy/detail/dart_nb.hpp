#pragma once

// Include first in every binding TU: generic caster selection must agree.
#define DART_NANOBIND_CASTER_GUARD 1

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <array>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <utility>

namespace dart {
namespace common {
class Subject;
}
namespace dynamics {
class Entity;
class Node;
class Joint;
class DegreeOfFreedom;
class Skeleton;
class BodyNode;
class ShapeFrame;
class InverseKinematics;
template <class T>
class TemplateBodyNodePtr;
} // namespace dynamics
namespace collision {
struct CollisionOption;
}
namespace simulation {
class World;
}
} // namespace dart

namespace nb = nanobind;

#include "gc.hpp"
// Missing optional STL casters must fail compilation, never bind opaque values.
namespace nanobind::detail {
template <class T, std::size_t N>
struct type_caster<std::array<T, N>>;
template <class R, class... Args>
struct type_caster<std::function<R(Args...)>>;
template <class Key, class T, class Compare, class Allocator>
struct type_caster<std::map<Key, T, Compare, Allocator>>;
template <class T, class U>
struct type_caster<std::pair<T, U>>;
template <class Key, class Compare, class Allocator>
struct type_caster<std::set<Key, Compare, Allocator>>;
template <class T, class Deleter>
struct type_caster<std::unique_ptr<T, Deleter>>;
template <class T, class Allocator>
struct type_caster<std::vector<T, Allocator>>;
template <class Key, class T, class Hash, class Equal, class Allocator>
struct type_caster<std::unordered_map<Key, T, Hash, Equal, Allocator>>;
template <class Key, class Hash, class Equal, class Allocator>
struct type_caster<std::unordered_set<Key, Hash, Equal, Allocator>>;
} // namespace nanobind::detail

#include "construction.hpp"
#include "polymorphic.hpp"

namespace dartnb {
struct CasterSelectionProbe
{
  virtual ~CasterSelectionProbe() = default;
};
static_assert(std::is_base_of_v<
              polymorphic_caster<CasterSelectionProbe>,
              nb::detail::make_caster<CasterSelectionProbe>>);
static_assert(std::is_base_of_v<
              shared_caster<CasterSelectionProbe>,
              nb::detail::make_caster<std::shared_ptr<CasterSelectionProbe>>>);
} // namespace dartnb
