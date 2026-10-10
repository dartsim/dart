#pragma once

#include "dart_nb.hpp"

#include <dart/optimizer/GradientDescentSolver.hpp>

namespace dartnb {
template <>
inline constexpr bool value_base<dart::optimizer::Solver::Properties> = true;
template <>
inline constexpr bool
    value_base<dart::optimizer::GradientDescentSolver::UniqueProperties> = true;
template <>
inline constexpr bool
    value_base<dart::optimizer::GradientDescentSolver::Properties> = true;
template <>
struct GcProperties<dart::optimizer::Solver::Properties> : std::true_type
{
};
template <>
struct GcProperties<dart::optimizer::GradientDescentSolver::Properties>
  : std::true_type
{
};
static_assert(std::is_base_of_v<
              polymorphic_caster<dart::optimizer::Solver::Properties>,
              nb::detail::make_caster<dart::optimizer::Solver::Properties>>);
static_assert(std::is_base_of_v<
              polymorphic_caster<
                  dart::optimizer::GradientDescentSolver::UniqueProperties>,
              nb::detail::make_caster<
                  dart::optimizer::GradientDescentSolver::UniqueProperties>>);
static_assert(
    std::is_base_of_v<
        polymorphic_caster<dart::optimizer::GradientDescentSolver::Properties>,
        nb::detail::make_caster<
            dart::optimizer::GradientDescentSolver::Properties>>);
} // namespace dartnb
