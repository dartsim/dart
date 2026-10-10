#pragma once

#include "dart_nb.hpp"

#include <dart/dynamics/InverseKinematics.hpp>

namespace dartnb {
void enumerate_gc(dart::dynamics::InverseKinematics::ErrorMethod&, GcEdges&);

template <>
inline constexpr bool value_base<
    dart::dynamics::InverseKinematics::ErrorMethod::Properties> = true;
template <>
inline constexpr bool value_base<dart::dynamics::InverseKinematics::
                                     TaskSpaceRegion::UniqueProperties> = true;
template <>
inline constexpr bool value_base<
    dart::dynamics::InverseKinematics::TaskSpaceRegion::Properties> = true;
static_assert(std::is_base_of_v<
              polymorphic_caster<
                  dart::dynamics::InverseKinematics::ErrorMethod::Properties>,
              nb::detail::make_caster<
                  dart::dynamics::InverseKinematics::ErrorMethod::Properties>>);
static_assert(std::is_base_of_v<
              polymorphic_caster<dart::dynamics::InverseKinematics::
                                     TaskSpaceRegion::UniqueProperties>,
              nb::detail::make_caster<dart::dynamics::InverseKinematics::
                                          TaskSpaceRegion::UniqueProperties>>);
static_assert(
    std::is_base_of_v<
        polymorphic_caster<
            dart::dynamics::InverseKinematics::TaskSpaceRegion::Properties>,
        nb::detail::make_caster<
            dart::dynamics::InverseKinematics::TaskSpaceRegion::Properties>>);
static_assert(GcOwner<dart::dynamics::InverseKinematics::ErrorMethod>::value);
static_assert(
    GcOwner<dart::dynamics::InverseKinematics::TaskSpaceRegion>::value);
static_assert(GcOwner<dart::dynamics::InverseKinematics::TaskSpaceRegion::
                          UniqueProperties>::value);
static_assert(
    GcOwner<
        dart::dynamics::InverseKinematics::TaskSpaceRegion::Properties>::value);
} // namespace dartnb
