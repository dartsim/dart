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

/* Author: Wim Meeussen, John Hsu */


#include "../urdfdom_headers/urdf_model/pose.h"
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <tinyxml.h>
#include "../urdf_parser/urdf_parser.h"

namespace urdf_export_helpers {

std::string values2str(unsigned int count, const double *values, double (*conv)(double))
{
    std::stringstream ss;
    for (unsigned int i = 0 ; i < count ; i++)
    {
        if (i > 0)
            ss << " ";
        ss << (conv ? conv(values[i]) : values[i]);
    }
    return ss.str();
}
std::string values2str(urdf::Vector3 vec)
{
    double xyz[3];
    xyz[0] = vec.x;
    xyz[1] = vec.y;
    xyz[2] = vec.z;
    return values2str(3, xyz);
}
std::string values2str(urdf::Rotation rot)
{
    double rpy[3];
    rot.getRPY(rpy[0], rpy[1], rpy[2]);
    return values2str(3, rpy);
}
std::string values2str(urdf::Color c)
{
    double rgba[4];
    rgba[0] = c.r;
    rgba[1] = c.g;
    rgba[2] = c.b;
    rgba[3] = c.a;
    return values2str(4, rgba);
}
std::string values2str(double d)
{
    return values2str(1, &d);
}
}

namespace urdf{

bool parsePose(Pose &pose, TiXmlElement* xml)
{
  pose.clear();
  if (xml)
  {
    const char* xyz_str = xml->Attribute("xyz");
    if (xyz_str != NULL)
    {
      try {
        pose.position.init(xyz_str);
      }
      catch (ParseError &e) {
	std::cout <<e.what() << std::endl;
        return false;
      }
    }

    const char* rpy_str = xml->Attribute("rpy");
    if (rpy_str != NULL)
    {
      try {
        pose.rotation.init(rpy_str);
      }
      catch (ParseError &e) {
	std::cout<<e.what()<<std::endl;
        return false;
      }
    }
  }
  return true;
}

bool exportPose(Pose &pose, TiXmlElement* xml)
{
  TiXmlElement *origin = new TiXmlElement("origin");
  std::string pose_xyz_str = urdf_export_helpers::values2str(pose.position);
  std::string pose_rpy_str = urdf_export_helpers::values2str(pose.rotation);
  origin->SetAttribute("xyz", pose_xyz_str);
  origin->SetAttribute("rpy", pose_rpy_str);
  xml->LinkEndChild(origin);
  return true;
}

}


