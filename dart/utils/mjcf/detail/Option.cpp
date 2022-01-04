/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/utils/mjcf/detail/Option.hpp"

#include "dart/utils/XmlHelpers.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors Option::read(tinyxml2::XMLElement* element)
{
  Errors errors;

  if (std::string(element->Name()) != "option")
  {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <option> from the provided element");
    return errors;
  }

  // timestep
  if (hasAttribute(element, "timestep"))
  {
    // TODO(JS): Error handling
    mTimestep = getAttributeDouble(element, "timestep");
  }

  // apirate
  if (hasAttribute(element, "apirate"))
  {
    mApiRate = getAttributeDouble(element, "apirate");
  }

  // impratio
  if (hasAttribute(element, "impratio"))
  {
    mImpRatio = getAttributeDouble(element, "impratio");
  }

  // gravity
  if (hasAttribute(element, "gravity"))
  {
    mGravity = getAttributeVector3d(element, "gravity");
  }

  // wind
  if (hasAttribute(element, "wind"))
  {
    mWind = getAttributeVector3d(element, "wind");
  }

  // magnetic
  if (hasAttribute(element, "magnetic"))
  {
    mMagnetic = getAttributeVector3d(element, "magnetic");
  }

  // density
  if (hasAttribute(element, "density"))
  {
    mDensity = getAttributeDouble(element, "density");
  }

  // viscosity
  if (hasAttribute(element, "viscosity"))
  {
    mViscosity = getAttributeDouble(element, "viscosity");
  }

  // integrator
  if (hasAttribute(element, "integrator"))
  {
    const std::string integrator = getAttributeString(element, "integrator");
    if (integrator == "Euler")
    {
      mIntegrator = Integrator::EULER;
    }
    else if (integrator == "RK4")
    {
      mIntegrator = Integrator::RK4;
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'integrator': " + integrator);
      return errors;
    }
  }

  // collision
  if (hasAttribute(element, "collision"))
  {
    const std::string collision = getAttributeString(element, "collision");
    if (collision == "all")
    {
      mCollision = CollisionType::ALL;
    }
    else if (collision == "predefined")
    {
      mCollision = CollisionType::PREDEFINED;
    }
    else if (collision == "dynamic")
    {
      mCollision = CollisionType::DYNAMIC;
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'collision': " + collision);
      return errors;
    }
  }

  // cone
  if (hasAttribute(element, "cone"))
  {
    const std::string cone = getAttributeString(element, "cone");
    if (cone == "pyramidal")
    {
      mCone = ConeType::PYRAMIDAL;
    }
    else if (cone == "elliptic")
    {
      mCone = ConeType::ELLIPTIC;
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'cone': " + cone);
      return errors;
    }
  }

  // jacobian
  if (hasAttribute(element, "jacobian"))
  {
    const std::string jacobian = getAttributeString(element, "jacobian");
    if (jacobian == "dense")
    {
      mJacobian = JacobianType::DENSE;
    }
    else if (jacobian == "sparse")
    {
      mJacobian = JacobianType::SPARSE;
    }
    else if (jacobian == "auto")
    {
      mJacobian = JacobianType::AUTO;
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'jacobian': " + jacobian);
      return errors;
    }
  }

  // solver
  if (hasAttribute(element, "solver"))
  {
    const std::string solver = getAttributeString(element, "solver");
    if (solver == "PGS")
    {
      mSolver = SolverType::PGS;
    }
    else if (solver == "CG")
    {
      mSolver = SolverType::CG;
    }
    else if (solver == "Newton")
    {
      mSolver = SolverType::NEWTON;
    }
    else
    {
      errors.emplace_back(
          ErrorCode::ATTRIBUTE_INVALID,
          "Invalid attribute for 'solver': " + solver);
      return errors;
    }
  }

  // iterations
  if (hasAttribute(element, "iterations"))
  {
    mIterations = getAttributeInt(element, "iterations");
  }

  // tolerance
  if (hasAttribute(element, "tolerance"))
  {
    mTolerance = getAttributeDouble(element, "tolerance");
  }

  // noslip_iterations
  if (hasAttribute(element, "noslip_iterations"))
  {
    mNoSlipTolerance = getAttributeInt(element, "noslip_iterations");
  }

  // noslip_tolerance
  if (hasAttribute(element, "noslip_tolerance"))
  {
    mNoSlipTolerance = getAttributeDouble(element, "noslip_tolerance");
  }

  // mpr_iterations
  if (hasAttribute(element, "mpr_iterations"))
  {
    mMprIterations = getAttributeInt(element, "mpr_iterations");
  }

  // mpr_tolerance
  if (hasAttribute(element, "mpr_tolerance"))
  {
    mMprTolerance = getAttributeDouble(element, "mpr_tolerance");
  }

  return errors;
}

//==============================================================================
double Option::getTimestep() const
{
  return mTimestep;
}

//==============================================================================
double Option::getApiRate() const
{
  return mTimestep;
}

//==============================================================================
double Option::getImpRatio() const
{
  return mTimestep;
}

//==============================================================================
const Eigen::Vector3d& Option::getGravity() const
{
  return mGravity;
}

//==============================================================================
const Eigen::Vector3d& Option::getWind() const
{
  return mWind;
}

//==============================================================================
const Eigen::Vector3d& Option::getMagnetic() const
{
  return mMagnetic;
}

//==============================================================================
double Option::getDensity() const
{
  return mDensity;
}

//==============================================================================
double Option::getViscosity() const
{
  return mViscosity;
}

//==============================================================================
Integrator Option::getIntegrator() const
{
  return mIntegrator;
}

//==============================================================================
CollisionType Option::getCollision() const
{
  return mCollision;
}

//==============================================================================
ConeType Option::getCone() const
{
  return mCone;
}

//==============================================================================
JacobianType Option::getJacobian() const
{
  return mJacobian;
}

//==============================================================================
SolverType Option::getSolver() const
{
  return mSolver;
}

//==============================================================================
int Option::getIterations() const
{
  return mIterations;
}

//==============================================================================
double Option::getTolerance() const
{
  return mTolerance;
}

//==============================================================================
int Option::getNoSlipIterations() const
{
  return mNoSlipIterations;
}

//==============================================================================
double Option::getNoSlipTolerance() const
{
  return mNoSlipTolerance;
}

//==============================================================================
int Option::getMprIterations() const
{
  return mMprIterations;
}

//==============================================================================
double Option::getMprTolerance() const
{
  return mMprTolerance;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
