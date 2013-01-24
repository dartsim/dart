/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Wim Meeussen */

#include "../urdf_parser/urdf_parser.h"
#include "../urdfdom_headers/urdf_model/model.h"
#include "../urdfdom_headers/urdf_world/world.h"
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <tinyxml.h>

const bool debug = false;

namespace urdf{

  // Added by achq on 2012/10/13 *******//
  
  bool parseEntity( Entity &_entity, TiXmlElement *_config, bool &_isRobot );
  bool parsePose(Pose &pose, TiXmlElement* xml);
  
  /**
   * @function parseWorldURDF
   */
  boost::shared_ptr<World> parseWorldURDF(const std::string &_xml_string, std::string _path ) {
    
    boost::shared_ptr<World> world( new World );
    world->clear();
    
    TiXmlDocument xml_doc;
    xml_doc.Parse( _xml_string.c_str() );
    
    TiXmlElement *world_xml = xml_doc.FirstChildElement("world");
    
    if( !world_xml ) {
      if(debug) printf ( "[parseWorldURDF] ERROR: Could not find a world, exiting! \n" );
      // world.reset();
      return world;
    }
    
    // Get world name
    const char *name = world_xml->Attribute("name");
    if(!name) {
      if(debug) printf ("No name given for the world! \n");
      // world.reset();
      return world;
    }
    world->name = std::string(name);
    if(debug) std::cout<< "World name: "<< world->name << std::endl;
    
    
    // Get all include filenames
    int count = 0;
    std::map<std::string, std::string> includedFiles;

    for( TiXmlElement* include_xml = world_xml->FirstChildElement("include");
	 include_xml; include_xml = include_xml->NextSiblingElement("include") ) {
      count++;
      const char *filename = include_xml->Attribute("filename");
      const char *model_name = include_xml->Attribute("model_name");
      std::string string_filename( filename );
      std::string string_model_name( model_name );
      includedFiles[string_model_name] = string_filename;
      if(debug) printf ("Include: %s %s \n", model_name, filename);
    }
    if(debug) printf ("Found %d include filenames \n", count);
    
    // Get all entities
    count = 0;
    for( TiXmlElement* entity_xml = world_xml->FirstChildElement("entity");
	 entity_xml; entity_xml = entity_xml->NextSiblingElement("entity") ) {
      count++;
      Entity entity;
      try {

	const char* entity_model = entity_xml->Attribute("model");
	std::string string_entity_model( entity_model );
	
	// Find the model
	if( includedFiles.find( string_entity_model ) == includedFiles.end() ) {
	  if(debug) printf ("[ERROR] Include the model you want to use \n");
	  return world;
	} 
	else {
	  std::string modelName = includedFiles.find( string_entity_model )->second;
	  std::string modelFullName = _path;
	  modelFullName.append( modelName );
	  if(debug) std::cout<< "Model full name: "<< modelFullName << std::endl;
	  
	  // Parse model
	  std::string xml_model_string;
	  
	  std::fstream xml_file( modelFullName.c_str(), std::fstream::in );
	  while( xml_file.good() ) {
	    std::string line;
	    std::getline( xml_file, line );
	    xml_model_string += (line + "\n");
	  }
	  xml_file.close();

	  boost::shared_ptr<ModelInterface> model;
	  model = parseURDF( xml_model_string );
	  entity.model = model;  

	  // Parse location
	  TiXmlElement *o = entity_xml->FirstChildElement("origin");
	  if( o ) {
	    if( !parsePose( entity.origin, o ) ) {
	      if(debug) printf ("[ERROR] Write the pose for your entity! \n");
	      return world; }
	  }

	// If name is defined
	const char* entity_name = entity_xml->Attribute("name");
	if( entity_name ) {
		std::string string_entity_name( entity_name );
		entity.model->name_ = string_entity_name;	
	}
	  
	  // Store in world
	  if( urdf::isRobotURDF( xml_model_string ) ) {
	    world->robotModels.push_back( entity  );
	  }
	  else if( urdf::isObjectURDF( xml_model_string ) ) {
	    world->objectModels.push_back( entity );
	  } 

	  
	} // end of include read
	
	
      }
      catch( ParseError &e ) {
	if(debug) printf ("Entity xml not initialized correctly \n");
	//entity->reset();
	//world->reset();
	return world;
      }
      
    } // end for
    if(debug) printf ("Found %d entities \n", count);
    
    return world;
    
  }	
  


// ***********************************//

bool parseWorld(World &world, TiXmlElement* config)
{

  // to be implemented

  return true;
}

bool exportWorld(World &world, TiXmlElement* xml)
{
  TiXmlElement * world_xml = new TiXmlElement("world");
  world_xml->SetAttribute("name", world.name);

  // to be implemented
  // exportModels(*world.models, world_xml);

  xml->LinkEndChild(world_xml);

  return true;
}

}
