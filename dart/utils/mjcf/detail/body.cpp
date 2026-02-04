/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/utils/mjcf/detail/body.hpp"

#include "dart/common/macros.hpp"
#include "dart/utils/mjcf/detail/compiler.hpp"
#include "dart/utils/mjcf/detail/size.hpp"
#include "dart/utils/mjcf/detail/types.hpp"
#include "dart/utils/mjcf/detail/utils.hpp"
#include "dart/utils/xml_helpers.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

//==============================================================================
Errors Body::read(
    tinyxml2::XMLElement* element,
    const std::optional<Size>& size,
    const Defaults& defaults,
    const Default* currentDefault)
{
  Errors errors;

  if (std::string(element->Name()) != "body") {
    errors.emplace_back(
        ErrorCode::INCORRECT_ELEMENT_TYPE,
        "Failed to find <Body> from the provided element");
    return errors;
  }

  // childclass
  if (hasAttribute(element, "childclass")) {
    const std::string className = getAttributeString(element, "childclass");
    const auto& defaultClass = defaults.getDefault(className);
    if (defaultClass) {
      currentDefault = &(*defaultClass);
    } else {
      errors.push_back(Error(
          ErrorCode::ATTRIBUTE_INVALID,
          "Failed to find default with childclass name '" + className + "'"));
    }
  }

  // Read attributes
  const Errors attrErrors = appendBodyAttributes(mAttributes, element, size);
  errors.insert(errors.end(), attrErrors.begin(), attrErrors.end());

  // Read <inertial>
  if (hasElement(element, "inertial")) {
    auto inertialElement = getElement(element, "inertial");
    DART_ASSERT(inertialElement);
    mAttributes.mInertial = Inertial();
    const Errors inertialErrors = mAttributes.mInertial->read(inertialElement);
    errors.insert(errors.end(), inertialErrors.begin(), inertialErrors.end());
  }

  // Read multiple <joint>
  ElementEnumerator jointElements(element, "joint");
  while (jointElements.next()) {
    Joint joint = Joint();
    const Errors jointErrors = joint.read(
        jointElements.get(), defaults, currentDefault->getJointAttributes());
    errors.insert(errors.end(), jointErrors.begin(), jointErrors.end());
    mJoints.emplace_back(std::move(joint));
  }

  // Read <freejoint> — creates a free joint that ignores defaults
  ElementEnumerator freejointElements(element, "freejoint");
  while (freejointElements.next()) {
    // Create a Joint with FREE type and zero dynamics, ignoring defaults
    Joint freeJoint;

    // freejoint does NOT inherit from default classes — use blank attributes
    JointAttributes blankAttrs;
    blankAttrs.mType = JointType::FREE;
    blankAttrs.mStiffness = 0;
    blankAttrs.mDamping = 0;
    blankAttrs.mFrictionLoss = 0;
    blankAttrs.mArmature = 0;

    // Read name if present
    if (hasAttribute(freejointElements.get(), "name")) {
      blankAttrs.mName = getAttributeString(freejointElements.get(), "name");
    }

    // Body is a friend of Joint, so set attributes directly
    freeJoint.mAttributes = blankAttrs;

    mJoints.emplace_back(std::move(freeJoint));
  }

  // Validate: at most one free joint per body (from <freejoint> or <joint
  // type="free">)
  {
    int freeJointCount = 0;
    for (const auto& joint : mJoints) {
      if (joint.mAttributes.mType == JointType::FREE) {
        ++freeJointCount;
      }
    }
    if (freeJointCount > 1) {
      errors.emplace_back(
          ErrorCode::ELEMENT_INVALID,
          "Body cannot have more than one free joint (from <freejoint> and/or "
          "<joint type=\"free\">)");
      return errors;
    }
  }

  warnUnknownElements(
      element, {"inertial", "joint", "freejoint", "geom", "site", "body"});

  // Read multiple <geom>
  ElementEnumerator geomElements(element, "geom");
  while (geomElements.next()) {
    Geom geom = Geom();
    const Errors geomErrors = geom.read(
        geomElements.get(), defaults, currentDefault->getGeomAttributes());
    errors.insert(errors.end(), geomErrors.begin(), geomErrors.end());
    mGeoms.emplace_back(std::move(geom));
  }

  // Read multiple <site>
  ElementEnumerator siteElements(element, "site");
  while (siteElements.next()) {
    Site site = Site();
    const Errors siteErrors = site.read(siteElements.get());
    errors.insert(errors.end(), siteErrors.begin(), siteErrors.end());
    mSites.emplace_back(std::move(site));
  }

  // Read childrend <body>
  ElementEnumerator bodyElements(element, "body");
  while (bodyElements.next()) {
    Body childBody = Body();
    const Errors bodyErrors
        = childBody.read(bodyElements.get(), size, defaults, currentDefault);
    errors.insert(errors.end(), bodyErrors.begin(), bodyErrors.end());
    mChildBodies.emplace_back(std::move(childBody));
  }

  return errors;
}

//==============================================================================
Errors Body::preprocess(const Compiler& compiler)
{
  Errors errors;

  if (mAttributes.mName) {
    mName = *mAttributes.mName;
  }

  mMocap = mAttributes.mMocap;

  // Transforms will be handled in compile()

  mUser = mAttributes.mUser;

  // Inertial will be handled in compile()

  for (Geom& geom : mGeoms) {
    const Errors geomErrors = geom.preprocess(compiler, true);
    errors.insert(errors.end(), geomErrors.begin(), geomErrors.end());
  }

  for (Site& site : mSites) {
    const Errors siteErrors = site.preprocess(compiler);
    errors.insert(errors.end(), siteErrors.begin(), siteErrors.end());
  }

  for (Joint& joint : mJoints) {
    const Errors jointErrors = joint.preprocess(compiler);
    errors.insert(errors.end(), jointErrors.begin(), jointErrors.end());
  }

  for (Body& body : mChildBodies) {
    const Errors bodyErrors = body.preprocess(compiler);
    errors.insert(errors.end(), bodyErrors.begin(), bodyErrors.end());
  }

  return errors;
}

//==============================================================================
Errors Body::compile(const Compiler& compiler)
{
  Errors errors;

  // Set inertial
  if (mAttributes.mInertial) {
    mInertial = *mAttributes.mInertial;
    const Errors inertialErrors = mInertial.compile(compiler);
    errors.insert(errors.end(), inertialErrors.begin(), inertialErrors.end());
  } else {
    for (const Geom& geom : mGeoms) {
      if (geom.getType() == GeomType::MESH) {
        errors.push_back(Error(
            ErrorCode::ELEMENT_MISSING,
            "<inertial> element must be specified if a <body> include a geom "
            "of mesh type."));
        break;
      }
    }

    if (!mGeoms.empty()) {
      mInertial
          = computeInertialFromGeoms(std::span<const Geom>{mGeoms}, compiler);
    }
  }

  for (Geom& geom : mGeoms) {
    const Errors geomErrors = geom.compile(compiler);
    errors.insert(errors.end(), geomErrors.begin(), geomErrors.end());
  }

  for (Site& site : mSites) {
    const Errors siteErrors = site.compile(compiler);
    errors.insert(errors.end(), siteErrors.begin(), siteErrors.end());
  }

  for (Joint& joint : mJoints) {
    const Errors jointErrors = joint.compile(compiler);
    errors.insert(errors.end(), jointErrors.begin(), jointErrors.end());
  }

  for (Body& body : mChildBodies) {
    const Errors bodyErrors = body.compile(compiler);
    errors.insert(errors.end(), bodyErrors.begin(), bodyErrors.end());
  }

  return errors;
}

//==============================================================================
Errors Body::postprocess(const Body* parent, const Compiler& compiler)
{
  Errors errors;

  if (mAttributes.mPos) {
    if (compiler.getCoordinate() == Coordinate::LOCAL) {
      mRelativeTransform.translation() = *mAttributes.mPos;
      mRelativeTransform.linear() = compileRotation(
          mAttributes.mQuat,
          mAttributes.mAxisAngle,
          mAttributes.mEuler,
          mAttributes.mXYAxes,
          mAttributes.mZAxis,
          compiler);
      if (!math::verifyTransform(mRelativeTransform)) {
        DART_WARN(
            "[MjcfBody] Non-finite transform detected while parsing MJCF body. "
            "The model file may contain extreme or invalid pose values.");
        mRelativeTransform = Eigen::Isometry3d::Identity();
      }
    } else {
      mWorldTransform.translation() = *mAttributes.mPos;
      mWorldTransform.linear() = compileRotation(
          mAttributes.mQuat,
          mAttributes.mAxisAngle,
          mAttributes.mEuler,
          mAttributes.mXYAxes,
          mAttributes.mZAxis,
          compiler);
      if (!math::verifyTransform(mWorldTransform)) {
        DART_WARN(
            "[MjcfBody] Non-finite transform detected while parsing MJCF body. "
            "The model file may contain extreme or invalid pose values.");
        mWorldTransform = Eigen::Isometry3d::Identity();
      }
      if (mAttributes.mInertial) {
        mInertial.setRelativeTransform(
            mWorldTransform.inverse() * mInertial.getWorldTransform());
      }
    }
  } else {
    if (compiler.getCoordinate() == Coordinate::LOCAL) {
      mRelativeTransform = mInertial.getRelativeTransform();
      if (!math::verifyTransform(mRelativeTransform)) {
        DART_WARN(
            "[MjcfBody] Non-finite transform detected while parsing MJCF body. "
            "The model file may contain extreme or invalid pose values.");
        mRelativeTransform = Eigen::Isometry3d::Identity();
        mInertial.setRelativeTransform(Eigen::Isometry3d::Identity());
      }
    } else {
      mWorldTransform = mInertial.getWorldTransform();
      if (!math::verifyTransform(mWorldTransform)) {
        DART_WARN(
            "[MjcfBody] Non-finite transform detected while parsing MJCF body. "
            "The model file may contain extreme or invalid pose values.");
        mWorldTransform = Eigen::Isometry3d::Identity();
      }
      if (parent != nullptr) {
        mRelativeTransform
            = parent->getWorldTransform().inverse() * mWorldTransform;
        if (!math::verifyTransform(mRelativeTransform)) {
          DART_WARN(
              "[MjcfBody] Non-finite transform detected while parsing MJCF "
              "body. "
              "The model file may contain extreme or invalid pose values.");
          mRelativeTransform = Eigen::Isometry3d::Identity();
        }
      } else {
        mRelativeTransform = mWorldTransform;
      }
      mInertial.setRelativeTransform(Eigen::Isometry3d::Identity());
    }
  }

  for (Geom& geom : mGeoms) {
    const Errors geomErrors = geom.postprocess(this, compiler);
    errors.insert(errors.end(), geomErrors.begin(), geomErrors.end());
  }

  for (Site& site : mSites) {
    const Errors siteErrors = site.postprocess(this, compiler);
    errors.insert(errors.end(), siteErrors.begin(), siteErrors.end());
  }

  for (Joint& joint : mJoints) {
    const Errors jointErrors = joint.postprocess(this, compiler);
    errors.insert(errors.end(), jointErrors.begin(), jointErrors.end());
  }

  for (Body& body : mChildBodies) {
    const Errors bodyErrors = body.postprocess(this, compiler);
    errors.insert(errors.end(), bodyErrors.begin(), bodyErrors.end());
  }

  return errors;
}

//==============================================================================
const std::string& Body::getName() const
{
  return mName;
}

//==============================================================================
bool Body::getMocap() const
{
  return mMocap;
}

//==============================================================================
const Inertial& Body::getInertial() const
{
  return mInertial;
}

//==============================================================================
std::size_t Body::getNumJoints() const
{
  return mJoints.size();
}

//==============================================================================
const Joint& Body::getJoint(std::size_t index) const
{
  return mJoints[index];
}

//==============================================================================
std::size_t Body::getNumChildBodies() const
{
  return mChildBodies.size();
}

//==============================================================================
const Body& Body::getChildBody(std::size_t index) const
{
  return mChildBodies[index];
}

//==============================================================================
std::size_t Body::getNumGeoms() const
{
  return mGeoms.size();
}

//==============================================================================
const Geom& Body::getGeom(std::size_t index) const
{
  return mGeoms[index];
}

//==============================================================================
std::size_t Body::getNumSites() const
{
  return mSites.size();
}

//==============================================================================
const Site& Body::getSite(std::size_t index) const
{
  return mSites[index];
}

//==============================================================================
void Body::setRelativeTransform(const Eigen::Isometry3d& tf)
{
  if (!math::verifyTransform(tf)) {
    DART_WARN(
        "[MjcfBody] Non-finite transform detected while parsing MJCF body. "
        "The model file may contain extreme or invalid pose values.");
    mRelativeTransform = Eigen::Isometry3d::Identity();
    return;
  }
  mRelativeTransform = tf;
}

//==============================================================================
const Eigen::Isometry3d& Body::getRelativeTransform() const
{
  return mRelativeTransform;
}

//==============================================================================
void Body::setWorldTransform(const Eigen::Isometry3d& tf)
{
  if (!math::verifyTransform(tf)) {
    DART_WARN(
        "[MjcfBody] Non-finite transform detected while parsing MJCF body. "
        "The model file may contain extreme or invalid pose values.");
    mWorldTransform = Eigen::Isometry3d::Identity();
    return;
  }
  mWorldTransform = tf;
}

//==============================================================================
const Eigen::Isometry3d& Body::getWorldTransform() const
{
  return mWorldTransform;
}

//==============================================================================
Inertial Body::computeInertialFromGeoms(
    std::span<const Geom> geoms, const Compiler& compiler)
{
  Inertial inertial;

  // TODO(JS): Handle this error properly instead of seg-faulting
  if (geoms.empty()) {
    DART_ERROR(
        "[MjcfParser] Faled to infer <inertial> because of no <geom> found.");
    DART_ASSERT(false);
    return inertial;
  }

  // TODO(JS): Single geom assumed for now
  DART_WARN_IF(
      geoms.size() > 1,
      "[MjcfParser] Unsupported number of <geom> in inferring <inertial>. We "
      "use only the first <geom> for now.");

  if (compiler.getCoordinate() == Coordinate::LOCAL) {
    inertial.setRelativeTransform(geoms[0].getRelativeTransform());
  } else {
    inertial.setWorldTransform(geoms[0].getWorldTransform());
  }

  inertial.setMass(geoms[0].getMass());

  const Eigen::Matrix3d& moi = geoms[0].getInertia();

  const Eigen::Vector3d diag = moi.diagonal();
  Eigen::Vector3d offDiag;
  offDiag[0] = moi(0, 1);
  offDiag[1] = moi(0, 2);
  offDiag[2] = moi(1, 2);
  inertial.setDiagInertia(diag);
  inertial.setOffDiagInertia(offDiag);

  return inertial;
}

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart
