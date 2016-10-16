/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2013, Humanoid Robotics Lab.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: A. Huaman */
/* @date 2013/06/20 */

#pragma once

#include <string>

#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_model/twist.h>
#include <urdf_world/world.h>

#include "dart/common/Uri.hpp"
#include "dart/common/ResourceRetriever.hpp"
#include "dart/utils/urdf/URDFTypes.hpp"

namespace dart {
namespace utils {
namespace urdf_parsing {

/// We need a customized version of the Entity class, because we need to keep
/// track of a Skeleton's uri in order to correctly handle relative file paths.
class Entity
{
public:

  Entity() = default;

  /// Copy over a standard urdfEntity
  Entity(const urdf::Entity& urdfEntity);

  urdf_shared_ptr<urdf::ModelInterface> model;
  urdf::Pose origin;
  urdf::Twist twist;

  dart::common::Uri uri;

};

class World
{
public:

  std::string name;
  std::vector<Entity> models;
};

std::shared_ptr<World> parseWorldURDF(const std::string &xml_string,
    const dart::common::Uri& _baseUri);

} // namespace urdf_parsing
} // namespace utils
} // namespace dart
