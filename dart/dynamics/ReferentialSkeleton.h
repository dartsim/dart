/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_DYNAMICS_REFERENTIALSKELETON_H_
#define DART_DYNAMICS_REFERENTIALSKELETON_H_

#include <unordered_map>

#include "dart/dynamics/MetaSkeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/DegreeOfFreedom.h"

namespace dart {
namespace dynamics {

/// ReferentialSkeleton is a base class used to implement Linkage, Disjointment,
/// and other classes that are used to reference subsections of Skeletons.
class ReferentialSkeleton : public MetaSkeleton
{
public:

  /// Default destructor
  virtual ~ReferentialSkeleton() = default;

  //----------------------------------------------------------------------------
  /// \{ \name Name
  //----------------------------------------------------------------------------

  // Documentation inherited
  const std::string& setName(const std::string& _name) override;

  // Documentation inherited
  const std::string& getName() const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Structural Properties
  //----------------------------------------------------------------------------

  // Documentation inherited
  size_t getNumBodyNodes() const override;

  // Documentation inherited
  BodyNode* getBodyNode(size_t _idx) override;

  // Documentation inherited
  const BodyNode* getBodyNode(size_t _idx) const override;

  // Documentation inherited
  const std::vector<BodyNode*>& getBodyNodes() override;

  // Documentation inherited
  std::vector<const BodyNode*> getBodyNodes() const override;

  // Documentation inherited
  size_t getIndexOf(const BodyNode* _bn) const override;

  // Documentation inherited
  size_t getNumJoints() const override;

  // Documentation inherited
  Joint* getJoint(size_t _idx) override;

  // Documentation inherited
  const Joint* getJoint(size_t _idx) const override;

  // Documentation inherited
  size_t getIndexOf(const Joint* _joint) const override;

  // Documentation inherited
  size_t getNumDofs() const override;

  // Documentation inherited
  DegreeOfFreedom* getDof(size_t _idx) override;

  // Documentation inherited
  const DegreeOfFreedom* getDof(size_t _idx) const override;

  // Documentation inherited
  const std::vector<DegreeOfFreedom*>& getDofs() override;

  // Documentation inherited
  std::vector<const DegreeOfFreedom*> getDofs() const override;

  // Documentation inherited
  size_t getIndexOf(const DegreeOfFreedom* _dof) const override;

  /// \}

protected:
  /// Name of this ReferentialSkeleton
  std::string mName;

  /// BodyNodes that this ReferentialSkeleton references
  std::vector<BodyNodePtr> mBodyNodes;

  /// Map for getting the index of a BodyNode within this ReferentialSkeleton
  std::map<BodyNodePtr, size_t> mMapForBodyNodes;

  /// DegreesOfFreedom that this ReferentialSkeleton references
  std::vector<DegreeOfFreedomPtr> mDofs;

  /// Map for getting the index of a dof within this ReferentialSkeleton
  std::map<DegreeOfFreedomPtr, size_t> mMapForDofs;
};

typedef std::shared_ptr<ReferentialSkeleton> ReferentialSkeletonPtr;
typedef std::shared_ptr<const ReferentialSkeleton> ConstReferentialSkeletonPtr;

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_REFERENTIALSKELETON_H_
