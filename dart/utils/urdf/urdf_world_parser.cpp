/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

#include "dart/utils/urdf/urdf_world_parser.hpp"

#include "dart/common/logging.hpp"
#include "dart/utils/urdf/include_urdf.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

const bool debug = false;

namespace dart {
namespace utils {
namespace urdf_parsing {

namespace {

bool parsePose(urdf::Pose& pose, tinyxml2::XMLElement* xml)
{
  pose.clear();
  if (xml) {
    const char* xyz_str = xml->Attribute("xyz");
    if (xyz_str) {
      try {
        pose.position.init(xyz_str);
      } catch (urdf::ParseError& e) {
        DART_ERROR("{}", e.what());
        return false;
      }
    }

    const char* rpy_str = xml->Attribute("rpy");
    if (rpy_str) {
      try {
        pose.rotation.init(rpy_str);
      } catch (urdf::ParseError& e) {
        DART_ERROR("{}", e.what());
        return false;
      }
    }
  }
  return true;
}

} // namespace

Entity::Entity(const urdf::Entity& urdfEntity)
  : model(urdfEntity.model), origin(urdfEntity.origin), twist(urdfEntity.twist)
{
  // Do nothing
}

/**
 * @function parseWorldURDF
 */
std::shared_ptr<World> parseWorldURDF(
    std::string_view xmlString,
    const dart::common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  tinyxml2::XMLDocument xml_doc;
  const std::string xmlStringCopy(xmlString);
  const auto result = xml_doc.Parse(xmlStringCopy.c_str());
  if (result != tinyxml2::XML_SUCCESS) {
    DART_WARN(
        "{}{}.",
        "[parseWorldURDF] Failed parsing XML: TinyXML2 returned error code ",
        result);
    return nullptr;
  }

  auto* world_xml = xml_doc.FirstChildElement("world");
  if (!world_xml) {
    DART_WARN(
        "{}",
        "[parseWorldURDF] ERROR: Could not find a <world> element in XML, "
        "exiting and not loading! \n");
    return nullptr;
  }

  // Get world name
  const char* name = world_xml->Attribute("name");
  if (!name) {
    DART_WARN(
        "{}",
        "[parseWorldURDF] ERROR: World does not have a name tag specified. "
        "Exiting and not loading! \n");
    return nullptr;
  }

  std::shared_ptr<World> world = std::make_shared<World>();
  world->name = std::string(name);
  if (debug)
    std::cout << "World name: " << world->name << std::endl;

  // Get all include filenames
  int count = 0;
  std::map<std::string, std::string> includedFiles;

  for (auto* include_xml = world_xml->FirstChildElement("include");
       include_xml != nullptr;
       include_xml = include_xml->NextSiblingElement("include")) {
    ++count;
    const char* filename = include_xml->Attribute("filename");
    const char* model_name = include_xml->Attribute("model_name");
    std::string string_filename(filename);
    std::string string_model_name(model_name);
    includedFiles[string_model_name] = string_filename;
    if (debug)
      std::cout << "Include: Model name: " << model_name
                << " filename: " << filename << std::endl;
  }
  if (debug)
    std::cout << "Found " << count << " include filenames " << std::endl;

  // Get all entities
  count = 0;
  for (auto* entity_xml = world_xml->FirstChildElement("entity");
       entity_xml != nullptr;
       entity_xml = entity_xml->NextSiblingElement("entity")) {
    count++;
    dart::utils::urdf_parsing::Entity entity;
    try {
      const char* entity_model = entity_xml->Attribute("model");
      std::string string_entity_model(entity_model);

      // Find the model
      if (!includedFiles.contains(string_entity_model)) {
        DART_WARN(
            "[parseWorldURDF] Cannot find the model [{}], did you provide the "
            "correct name? We will return a nullptr.\\n",
            string_entity_model);
        return nullptr;
      } else {
        const std::string& fileName = includedFiles.at(string_entity_model);

        dart::common::Uri absoluteUri;
        if (!absoluteUri.fromRelativeUri(baseUri, std::string_view{fileName})) {
          DART_WARN(
              "[parseWorldURDF] Failed resolving mesh URI '{}' relative to "
              "'{}'. We will return a nullptr.",
              fileName,
              baseUri.toString());
          return nullptr;
        }

        entity.uri = absoluteUri;

        // Parse model
        const auto xml_model_string = retriever->readAll(absoluteUri);
        entity.model = urdf::parseURDF(xml_model_string);

        if (!entity.model) {
          DART_WARN(
              "[parseWorldURDF] Could not find a model named [{}] from [{}]. "
              "We will return a nullptr.",
              xml_model_string,
              absoluteUri.toString());
          return nullptr;
        } else {
          // Parse location
          auto* origin = entity_xml->FirstChildElement("origin");
          if (origin) {
            if (!urdf_parsing::parsePose(entity.origin, origin)) {
              DART_WARN(
                  "[ERROR] Missing origin tag for '{}'",
                  entity.model->getName());
              return world;
            }
          }

          // If name is defined
          const char* entity_name = entity_xml->Attribute("name");
          if (entity_name) {
            std::string string_entity_name(entity_name);
            entity.model->name_ = string_entity_name;
          }

          // Store in world
          world->models.push_back(entity);
        }
      } // end of include read
    } catch (urdf::ParseError& /*e*/) {
      if (debug)
        std::cout << "Entity xml not initialized correctly \n";
      // entity->reset();
      // world->reset();
      return world;
    }

  } // end for
  if (debug)
    std::cout << "Found " << count << " entities \n";

  return world;
}

} // namespace urdf_parsing
} // namespace utils
} // namespace dart
