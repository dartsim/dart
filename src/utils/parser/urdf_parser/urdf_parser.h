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

#ifndef URDF_PARSER_URDF_PARSER_H
#define URDF_PARSER_URDF_PARSER_H

#include <string>
#include <map>
#include <tinyxml.h>
#include <boost/function.hpp>
//#include <urdf_model/model.h>
#include "../urdfdom_headers/urdf_model/model.h"
#include "../urdfdom_headers/urdf_world/world.h"
#include "../urdfdom_headers/urdf_model/color.h"

namespace urdf_export_helpers {

std::string values2str(unsigned int count, const double *values, double (*conv)(double) = NULL);
std::string values2str(urdf::Vector3 vec);
std::string values2str(urdf::Rotation rot);
std::string values2str(urdf::Color c);
std::string values2str(double d);

}

namespace urdf{

  boost::shared_ptr<ModelInterface> parseURDF(const std::string &xml_string);
  TiXmlDocument*  exportURDF(boost::shared_ptr<ModelInterface> &model);
   // Added functions by achq on 2012/10/13  ********* //
  bool isObjectURDF( const std::string &_xml_string );
  bool isRobotURDF( const std::string &_xml_string );
  boost::shared_ptr<World> parseWorldURDF(const std::string &xml_string, std::string _path );
  // ********************************************** //
}

#endif
