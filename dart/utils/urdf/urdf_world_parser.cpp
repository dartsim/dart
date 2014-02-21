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

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_world/world.h>
#include <urdf_model/pose.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <tinyxml.h>

#include "urdf_world_parser.h"

const bool debug = false;

namespace urdf{

  // Implemented in urdf_parser/src/pose.cpp, for some reason nobody thought of putting it in the header
  bool parsePose(Pose &pose, TiXmlElement* xml);
  
  /**
   * @function parseWorldURDF
   */
  World* parseWorldURDF(const std::string &_xml_string, std::string _root_to_world_path) {
    
    World* world = new World();
    TiXmlDocument xml_doc;
    xml_doc.Parse( _xml_string.c_str() );
    TiXmlElement *world_xml = xml_doc.FirstChildElement("world");
    if( !world_xml ) {
      printf ( "[parseWorldURDF] ERROR: Could not find a <world> element in XML, exiting and not loading! \n" );
      return NULL;
    }
    
    // Get world name
    const char *name = world_xml->Attribute("name");
    if(!name) {
      printf ("[parseWorldURDF] ERROR: World does not have a name specified. Exiting and not loading! \n");
      return NULL;
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
      if(debug) std::cout<<"Include: Model name: "<<  model_name <<" filename: "<< filename <<std::endl;
    }
    if(debug) std::cout<<"Found "<<count<<" include filenames "<<std::endl;
    
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
	  std::cout<<"[parseWorldURDF] ERROR: I cannot find the model you want to use, did you write the name right? Exiting and not loading! \n"<<std::endl;
	  return NULL;
	} 
	else {
	  std::string fileName = includedFiles.find( string_entity_model )->second;
	  std::string fileFullName = _root_to_world_path;
	  fileFullName.append( fileName );
	  if(debug) std::cout<< "Entity full filename: "<< fileFullName << std::endl;
	  
	  // Parse model
	  std::string xml_model_string;
	  std::fstream xml_file( fileFullName.c_str(), std::fstream::in );
	  while( xml_file.good() ) {
	    std::string line;
	    std::getline( xml_file, line );
	    xml_model_string += (line + "\n");
	  }
	  xml_file.close();
	  entity.model = parseURDF( xml_model_string );

	  if( !entity.model ) {
	    std::cout<< "[parseWorldURDF] Model in "<<fileFullName<<" not found. Exiting and not loading!" <<std::endl;
	    return NULL;
	  }
	  else {
	    // Parse location
	    TiXmlElement *o = entity_xml->FirstChildElement("origin");
	    if( o ) {
	      if( !parsePose( entity.origin, o ) ) {
		printf ("[ERROR] Write the pose for your entity! \n");
		return world;
	      }
	    }
	    
	    // If name is defined
	    const char* entity_name = entity_xml->Attribute("name");
	    if( entity_name ) {
	      std::string string_entity_name( entity_name );
	      entity.model->name_ = string_entity_name;	
	    }
	    
	    // Store in world
	    world->models.push_back( entity );
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
  


} // end namespace
