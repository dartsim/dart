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

#include "dart/common/Console.hpp"

#include <tinyxml2.h>
#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_world/world.h>

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
  if (xml)
  {
    const char* xyz_str = xml->Attribute("xyz");
    if (xyz_str)
    {
      try
      {
        pose.position.init(xyz_str);
      }
      catch (urdf::ParseError& e)
      {
        dterr << e.what() << "\n";
        return false;
      }
    }

    const char* rpy_str = xml->Attribute("rpy");
    if (rpy_str)
    {
      try
      {
        pose.rotation.init(rpy_str);
      }
      catch (urdf::ParseError& e)
      {
        dterr << e.what() << "\n";
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
    const std::string& _xml_string,
    const dart::common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  tinyxml2::XMLDocument xml_doc;
  const auto result = xml_doc.Parse(&_xml_string.front());
  if (result != tinyxml2::XML_SUCCESS)
  {
    dtwarn << "[parseWorldURDF] Failed parsing XML: TinyXML2 returned error"
              " code "
           << result << ".\n";
    return nullptr;
  }

  auto* world_xml = xml_doc.FirstChildElement("world");
  if (!world_xml)
  {
    dtwarn << "[parseWorldURDF] ERROR: Could not find a <world> element in "
              "XML, exiting and not loading! \n";
    return nullptr;
  }

  // Get world name
  const char* name = world_xml->Attribute("name");
  if (!name)
  {
    dtwarn << "[parseWorldURDF] ERROR: World does not have a name tag "
              "specified. Exiting and not loading! \n";
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
       include_xml = include_xml->NextSiblingElement("include"))
  {
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
       entity_xml = entity_xml->NextSiblingElement("entity"))
  {
    count++;
    dart::utils::urdf_parsing::Entity entity;
    try
    {
      const char* entity_model = entity_xml->Attribute("model");
      std::string string_entity_model(entity_model);

      // Find the model
      if (includedFiles.find(string_entity_model) == includedFiles.end())
      {
        dtwarn << "[parseWorldURDF] Cannot find the model ["
               << string_entity_model << "], did you provide the correct name? "
               << "We will return a nullptr.\n"
               << std::endl;
        return nullptr;
      }
      else
      {
        std::string fileName = includedFiles.find(string_entity_model)->second;

        dart::common::Uri absoluteUri;
        if (!absoluteUri.fromRelativeUri(_baseUri, fileName))
        {
          dtwarn << "[parseWorldURDF] Failed resolving mesh URI '" << fileName
                 << "' relative to '" << _baseUri.toString()
                 << "'. We will return a nullptr.\n";
          return nullptr;
        }

        entity.uri = absoluteUri;

        // Parse model
        const auto xml_model_string = retriever->readAll(absoluteUri);
        entity.model = urdf::parseURDF(xml_model_string);

        if (!entity.model)
        {
          dtwarn << "[parseWorldURDF] Could not find a model named ["
                 << xml_model_string << "] from [" << absoluteUri.toString()
                 << "]. We will return a nullptr.\n";
          return nullptr;
        }
        else
        {
          // Parse location
          auto* origin = entity_xml->FirstChildElement("origin");
          if (origin)
          {
            if (!parsePose(entity.origin, origin))
            {
              dtwarn << "[ERROR] Missing origin tag for '"
                     << entity.model->getName() << "'\n";
              return world;
            }
          }

          // If name is defined
          const char* entity_name = entity_xml->Attribute("name");
          if (entity_name)
          {
            std::string string_entity_name(entity_name);
            entity.model->name_ = string_entity_name;
          }

          // Store in world
          world->models.push_back(entity);
        }
      } // end of include read
    }
    catch (urdf::ParseError& /*e*/)
    {
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
