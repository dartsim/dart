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
#include "../urdfdom_headers/urdf_model/link.h"
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <tinyxml.h>

namespace urdf{

const bool debug = false;

bool parsePose(Pose &pose, TiXmlElement* xml);

bool parseMaterial(Material &material, TiXmlElement *config)
{
  bool has_rgb = false;
  bool has_filename = false;

  material.clear();

  if (!config->Attribute("name"))
  {
    if(debug) printf ("Material must contain a name attribute \n");
    return false;
  }
  
  material.name = config->Attribute("name");

  // texture
  TiXmlElement *t = config->FirstChildElement("texture");
  if (t)
  {
    if (t->Attribute("filename"))
    {
      material.texture_filename = t->Attribute("filename");
      has_filename = true;
    }
  }

  // color
  TiXmlElement *c = config->FirstChildElement("color");
  if (c)
  {
    if (c->Attribute("rgba")) {

      try {
        material.color.init(c->Attribute("rgba"));
        has_rgb = true;
      }
      catch (ParseError &e) {
        material.color.clear();
	std::cout<<"Material [" << material.name <<"] has malformed color rgba values: "<<e.what() << std::endl;
      }
    }
  }

  if (!has_rgb && !has_filename) {
    if (!has_rgb) std::cout<< "Material ["<<material.name<<"] color has no rgba"<< std::endl;
    if (!has_filename) std::cout<<"Material ["<<material.name<<"] not defined in file"<< std::endl;
    return false;
  }
  return true;
}


bool parseSphere(Sphere &s, TiXmlElement *c)
{
  s.clear();

  s.type = Geometry::SPHERE;
  if (!c->Attribute("radius"))
  {
    if(debug) printf ("Sphere shape must have a radius attribute \n");
    return false;
  }

  try
  {
    s.radius = boost::lexical_cast<double>(c->Attribute("radius"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    std::stringstream stm;
    stm << "radius [" << c->Attribute("radius") << "] is not a valid float: " << e.what();
    std::cout<< stm.str().c_str() << std::endl;
    return false;
  }
  
  return true;
}

bool parseBox(Box &b, TiXmlElement *c)
{
  b.clear();
  
  b.type = Geometry::BOX;
  if (!c->Attribute("size"))
  {
    if(debug) printf ("Box shape has no size attribute \n");
    return false;
  }
  try
  {
    b.dim.init(c->Attribute("size"));
  }
  catch (ParseError &e)
  {
    b.dim.clear();
    std::cout << e.what() << std::endl;
    return false;
  }
  return true;
}

bool parseCylinder(Cylinder &y, TiXmlElement *c)
{
  y.clear();

  y.type = Geometry::CYLINDER;
  if (!c->Attribute("length") ||
      !c->Attribute("radius"))
  {
    if(debug) printf ("Cylinder shape must have both length and radius attributes \n");
    return false;
  }

  try
  {
    y.length = boost::lexical_cast<double>(c->Attribute("length"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    std::stringstream stm;
    stm << "length [" << c->Attribute("length") << "] is not a valid float";
    if(debug) printf ("%s", stm.str().c_str());
    return false;
  }

  try
  {
    y.radius = boost::lexical_cast<double>(c->Attribute("radius"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    std::stringstream stm;
    stm << "radius [" << c->Attribute("radius") << "] is not a valid float";
    if(debug) printf ("%s", stm.str().c_str());
    return false;
  }
  return true;
}


bool parseMesh(Mesh &m, TiXmlElement *c)
{
  m.clear();

  m.type = Geometry::MESH;
  if (!c->Attribute("filename")) {
    if(debug) printf ("Mesh must contain a filename attribute \n");
    return false;
  }

  m.filename = c->Attribute("filename");

  if (c->Attribute("scale")) {
    try {
      m.scale.init(c->Attribute("scale"));
    }
    catch (ParseError &e) {
      m.scale.clear();
      if(debug) printf ("Mesh scale was specified, but could not be parsed: %s", e.what());
      return false;
    }
  }
  else
  {
    m.scale.x = m.scale.y = m.scale.z = 1;
  }
  return true;
}

boost::shared_ptr<Geometry> parseGeometry(TiXmlElement *g)
{
  boost::shared_ptr<Geometry> geom;
  if (!g) return geom;

  TiXmlElement *shape = g->FirstChildElement();
  if (!shape)
  {
    if(debug) printf ("Geometry tag contains no child element. \n");
    return geom;
  }

  std::string type_name = shape->ValueStr();
  if (type_name == "sphere")
  {
    Sphere *s = new Sphere();
    geom.reset(s);
    if (parseSphere(*s, shape))
      return geom;
  }
  else if (type_name == "box")
  {
    Box *b = new Box();
    geom.reset(b);
    if (parseBox(*b, shape))
      return geom;
  }
  else if (type_name == "cylinder")
  {
    Cylinder *c = new Cylinder();
    geom.reset(c);
    if (parseCylinder(*c, shape))
      return geom;
  }
  else if (type_name == "mesh")
  {
    Mesh *m = new Mesh();
    geom.reset(m);
    if (parseMesh(*m, shape))
      return geom;    
  }
  else
  {
    if(debug) printf ("Unknown geometry type '%s' \n", type_name.c_str());
    return geom;
  }
  
  return boost::shared_ptr<Geometry>();
}

bool parseInertial(Inertial &i, TiXmlElement *config)
{
  i.clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o)
  {
    if (!parsePose(i.origin, o))
      return false;
  }

  TiXmlElement *mass_xml = config->FirstChildElement("mass");
  if (!mass_xml)
  {
    if(debug) printf ("Inertial element must have a mass element \n");
    return false;
  }
  if (!mass_xml->Attribute("value"))
  {
    if(debug) printf ("Inertial: mass element must have value attribute \n");
    return false;
  }

  try
  {
    i.mass = boost::lexical_cast<double>(mass_xml->Attribute("value"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    std::stringstream stm;
    stm << "Inertial: mass [" << mass_xml->Attribute("value")
        << "] is not a float";
    if(debug) printf ("%s", stm.str().c_str());
    return false;
  }

  TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
  if (!inertia_xml)
  {
    if(debug) printf ("Inertial element must have inertia element \n");
    return false;
  }
  if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
        inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
        inertia_xml->Attribute("izz")))
  {
    if(debug) printf ("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes \n");
    return false;
  }
  try
  {
    i.ixx  = boost::lexical_cast<double>(inertia_xml->Attribute("ixx"));
    i.ixy  = boost::lexical_cast<double>(inertia_xml->Attribute("ixy"));
    i.ixz  = boost::lexical_cast<double>(inertia_xml->Attribute("ixz"));
    i.iyy  = boost::lexical_cast<double>(inertia_xml->Attribute("iyy"));
    i.iyz  = boost::lexical_cast<double>(inertia_xml->Attribute("iyz"));
    i.izz  = boost::lexical_cast<double>(inertia_xml->Attribute("izz"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    std::stringstream stm;
    stm << "Inertial: one of the inertia elements is not a valid double:"
        << " ixx [" << inertia_xml->Attribute("ixx") << "]"
        << " ixy [" << inertia_xml->Attribute("ixy") << "]"
        << " ixz [" << inertia_xml->Attribute("ixz") << "]"
        << " iyy [" << inertia_xml->Attribute("iyy") << "]"
        << " iyz [" << inertia_xml->Attribute("iyz") << "]"
        << " izz [" << inertia_xml->Attribute("izz") << "]";
    if(debug) printf ("%s", stm.str().c_str());
    return false;
  }
  return true;
}

bool parseVisual(Visual &vis, TiXmlElement *config)
{
  vis.clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    if (!parsePose(vis.origin, o))
      return false;
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  vis.geometry = parseGeometry(geom);
  if (!vis.geometry)
    return false;

  // Material
  TiXmlElement *mat = config->FirstChildElement("material");
  if (mat) {
    // get material name
    if (!mat->Attribute("name")) {
      if(debug) printf ("Visual material must contain a name attribute \n");
      return false;
    }
    vis.material_name = mat->Attribute("name");
    
    // try to parse material element in place
    vis.material.reset(new Material());
    if (!parseMaterial(*vis.material, mat))
    {
      //vis.material.reset();
      //return false;
      if(debug) printf ("material has only name, actual material definition may be in the model \n");
    }
  }
  
  // Group Tag (optional)
  // collision blocks without a group tag are designated to the "default" group
  const char *group_name_char = config->Attribute("group");
  if (!group_name_char)
    vis.group_name = std::string("default");
  else
    vis.group_name = std::string(group_name_char);
  return true;
}

bool parseCollision(Collision &col, TiXmlElement* config)
{  
  col.clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    if (!parsePose(col.origin, o))
      return false;
  }
  
  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  col.geometry = parseGeometry(geom);
  if (!col.geometry)
    return false;

  // Group Tag (optional)
  // collision blocks without a group tag are designated to the "default" group
  const char *group_name_char = config->Attribute("group");
  if (!group_name_char)
    col.group_name = std::string("default");
  else
    col.group_name = std::string(group_name_char);
  return true;
}

bool parseLink(Link &link, TiXmlElement* config)
{
  
  link.clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    if(debug) printf ("No name given for the link. \n");
    return false;
  }
  link.name = std::string(name_char);

  // Inertial (optional)
  TiXmlElement *i = config->FirstChildElement("inertial");
  if (i)
  {
    link.inertial.reset(new Inertial());
    if (!parseInertial(*link.inertial, i))
    {
      if(debug) printf ("Could not parse inertial element for Link [%s] \n", link.name.c_str());
      return false;
    }
  }

  // Multiple Visuals (optional)
  for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
  {

    boost::shared_ptr<Visual> vis;
    vis.reset(new Visual());
    if (parseVisual(*vis, vis_xml))
    {
      boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > viss = link.getVisuals(vis->group_name);
      if (!viss)
      {
        // group does not exist, create one and add to map
        viss.reset(new std::vector<boost::shared_ptr<Visual > >);
        // new group name, create vector, add vector to map and add Visual to the vector
        link.visual_groups.insert(make_pair(vis->group_name,viss));
        if(debug) printf ("successfully added a new visual group name '%s' \n",vis->group_name.c_str());
      }
      
      // group exists, add Visual to the vector in the map
      viss->push_back(vis);
      if(debug) printf ("successfully added a new visual under group name '%s' \n",vis->group_name.c_str());
    }
    else
    {
      vis.reset();
      if(debug) printf ("Could not parse visual element for Link [%s] \n", link.name.c_str());
      return false;
    }
  }

  // Visual (optional)
  // Assign one single default visual pointer from the visual_groups map
  link.visual.reset();
  boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > default_visual = link.getVisuals("default");
  if (!default_visual)
  {
    //("No 'default' visual group for Link '%s'", this->name.c_str());
  }
  else if (default_visual->empty())
  {
    //("'default' visual group is empty for Link '%s'", this->name.c_str());
  }
  else
  {
    if (default_visual->size() > 1)
    {
      //("'default' visual group has %d visuals for Link '%s', taking the first one as default",(int)default_visual->size(), this->name.c_str());
    }
    link.visual = (*default_visual->begin());
  }


  // Multiple Collisions (optional)
  for (TiXmlElement* col_xml = config->FirstChildElement("collision"); col_xml; col_xml = col_xml->NextSiblingElement("collision"))
  {
    boost::shared_ptr<Collision> col;
    col.reset(new Collision());
    if (parseCollision(*col, col_xml))
    {      
      boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > cols = link.getCollisions(col->group_name);  
      
      if (!cols)
      {
        // group does not exist, create one and add to map
        cols.reset(new std::vector<boost::shared_ptr<Collision > >);
        // new group name, create vector, add vector to map and add Collision to the vector
        link.collision_groups.insert(make_pair(col->group_name,cols));
        if(debug) printf ("successfully added a new collision group name '%s' \n",col->group_name.c_str());
      }

      // group exists, add Collision to the vector in the map
      cols->push_back(col);
      if(debug) printf ("successfully added a new collision under group name '%s' \n",col->group_name.c_str());
    }
    else
    {
      col.reset();
      if(debug) printf ("Could not parse collision element for Link [%s] \n",  link.name.c_str());
      return false;
    }
  }
  
  // Collision (optional)
  // Assign one single default collision pointer from the collision_groups map
  link.collision.reset();
  boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > default_collision = link.getCollisions("default");

  if (!default_collision)
  {
    if(debug) printf ("No 'default' collision group for Link '%s' \n", link.name.c_str());
  }
  else if (default_collision->empty())
  {
    if(debug) printf ("'default' collision group is empty for Link '%s' \n", link.name.c_str());
  }
  else
  {
    if (default_collision->size() > 1)
    {
      if(debug) printf ("'default' collision group has %d collisions for Link '%s', taking the first one as default \n",(int)default_collision->size(), link.name.c_str());
    }
    link.collision = (*default_collision->begin());
  }
}

/* exports */
bool exportPose(Pose &pose, TiXmlElement* xml);

bool exportMaterial(Material &material, TiXmlElement *xml)
{
  TiXmlElement *material_xml = new TiXmlElement("material");
  material_xml->SetAttribute("name", material.name);

  TiXmlElement* texture = new TiXmlElement("texture");
  if (!material.texture_filename.empty())
    texture->SetAttribute("filename", material.texture_filename);
  material_xml->LinkEndChild(texture);

  TiXmlElement* color = new TiXmlElement("color");
  color->SetAttribute("rgba", urdf_export_helpers::values2str(material.color));
  material_xml->LinkEndChild(color);
  xml->LinkEndChild(material_xml);
  return true;
}

bool exportSphere(Sphere &s, TiXmlElement *xml)
{
  // e.g. add <sphere radius="1"/>
  TiXmlElement *sphere_xml = new TiXmlElement("sphere");
  sphere_xml->SetAttribute("radius", urdf_export_helpers::values2str(s.radius));
  xml->LinkEndChild(sphere_xml);
  return true;
}

bool exportBox(Box &b, TiXmlElement *xml)
{
  // e.g. add <box size="1 1 1"/>
  TiXmlElement *box_xml = new TiXmlElement("box");
  box_xml->SetAttribute("size", urdf_export_helpers::values2str(b.dim));
  xml->LinkEndChild(box_xml);
  return true;
}

bool exportCylinder(Cylinder &y, TiXmlElement *xml)
{
  // e.g. add <cylinder radius="1"/>
  TiXmlElement *cylinder_xml = new TiXmlElement("cylinder");
  cylinder_xml->SetAttribute("radius", urdf_export_helpers::values2str(y.radius));
  cylinder_xml->SetAttribute("length", urdf_export_helpers::values2str(y.length));
  xml->LinkEndChild(cylinder_xml);
  return true;
}

bool exportMesh(Mesh &m, TiXmlElement *xml)
{
  // e.g. add <mesh filename="my_file" scale="1 1 1"/>
  TiXmlElement *mesh_xml = new TiXmlElement("mesh");
  if (!m.filename.empty())
    mesh_xml->SetAttribute("filename", m.filename);
  mesh_xml->SetAttribute("scale", urdf_export_helpers::values2str(m.scale));
  xml->LinkEndChild(mesh_xml);
  return true;
}

bool exportGeometry(boost::shared_ptr<Geometry> &geom, TiXmlElement *xml)
{
  TiXmlElement *geometry_xml = new TiXmlElement("geometry");
  if (boost::dynamic_pointer_cast<Sphere>(geom))
  {
    exportSphere((*(boost::dynamic_pointer_cast<Sphere>(geom).get())), geometry_xml);
  }
  else if (boost::dynamic_pointer_cast<Box>(geom))
  {
    exportBox((*(boost::dynamic_pointer_cast<Box>(geom).get())), geometry_xml);
  }
  else if (boost::dynamic_pointer_cast<Cylinder>(geom))
  {
    exportCylinder((*(boost::dynamic_pointer_cast<Cylinder>(geom).get())), geometry_xml);
  }
  else if (boost::dynamic_pointer_cast<Mesh>(geom))
  {
    exportMesh((*(boost::dynamic_pointer_cast<Mesh>(geom).get())), geometry_xml);
  }
  else
  {
    if(debug) printf ("geometry not specified, I'll make one up for you! \n");
    Sphere *s = new Sphere();
    s->radius = 0.03;
    geom.reset(s);
    exportSphere((*(boost::dynamic_pointer_cast<Sphere>(geom).get())), geometry_xml);
  }

  xml->LinkEndChild(geometry_xml);
  return true;
}

bool exportInertial(Inertial &i, TiXmlElement *xml)
{
  // adds <inertial>
  //        <mass value="1"/>
  //        <pose xyz="0 0 0" rpy="0 0 0"/>
  //        <inertia ixx="1" ixy="0" />
  //      </inertial>
  TiXmlElement *inertial_xml = new TiXmlElement("inertial");

  TiXmlElement *mass_xml = new TiXmlElement("mass");
  mass_xml->SetAttribute("value", urdf_export_helpers::values2str(i.mass));
  inertial_xml->LinkEndChild(mass_xml);

  exportPose(i.origin, inertial_xml);

  TiXmlElement *inertia_xml = new TiXmlElement("inertia");
  inertia_xml->SetAttribute("ixx", urdf_export_helpers::values2str(i.ixx));
  inertia_xml->SetAttribute("ixy", urdf_export_helpers::values2str(i.ixy));
  inertia_xml->SetAttribute("ixz", urdf_export_helpers::values2str(i.ixz));
  inertia_xml->SetAttribute("iyy", urdf_export_helpers::values2str(i.iyy));
  inertia_xml->SetAttribute("iyz", urdf_export_helpers::values2str(i.iyz));
  inertia_xml->SetAttribute("izz", urdf_export_helpers::values2str(i.izz));
  inertial_xml->LinkEndChild(inertia_xml);

  xml->LinkEndChild(inertial_xml);
  
  return true;
}

bool exportVisual(Visual &vis, TiXmlElement *xml)
{
  // <visual group="default">
  //   <origin rpy="0 0 0" xyz="0 0 0"/>
  //   <geometry>
  //     <mesh filename="mesh.dae"/>
  //   </geometry>
  //   <material name="Grey"/>
  // </visual>
  TiXmlElement * visual_xml = new TiXmlElement("visual");

  exportPose(vis.origin, visual_xml);

  exportGeometry(vis.geometry, visual_xml);

  if (vis.material)
    exportMaterial(*vis.material, visual_xml);

  if (!vis.group_name.empty())
    visual_xml->SetAttribute("group", vis.group_name);

  xml->LinkEndChild(visual_xml);

  return true;
}

bool exportCollision(Collision &col, TiXmlElement* xml)
{  
  // <collision group="default">
  //   <origin rpy="0 0 0" xyz="0 0 0"/>
  //   <geometry>
  //     <mesh filename="mesh.dae"/>
  //   </geometry>
  //   <material name="Grey"/>
  // </collision>
  TiXmlElement * collision_xml = new TiXmlElement("collision");

  exportPose(col.origin, collision_xml);

  exportGeometry(col.geometry, collision_xml);

  if (!col.group_name.empty())
    collision_xml->SetAttribute("group", col.group_name);

  xml->LinkEndChild(collision_xml);

  return true;
}

bool exportLink(Link &link, TiXmlElement* xml)
{
  TiXmlElement * link_xml = new TiXmlElement("link");
  link_xml->SetAttribute("name", link.name);

  exportInertial(*link.inertial, link_xml);
  exportVisual(*link.visual, link_xml);
  exportCollision(*link.collision, link_xml);

  xml->LinkEndChild(link_xml);

  return true;
}

}
