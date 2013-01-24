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

#ifndef URDF_INTERFACE_MODEL_H
#define URDF_INTERFACE_MODEL_H

#include <string>
#include <map>
#include <boost/function.hpp>
#include "link.h"
#include "../urdf_exception/exception.h"

namespace urdf {

  /**
   * @class ModelInterface
   */
  class ModelInterface
  {
  public:

    /**
     * @function getRoot
     */
    boost::shared_ptr<const Link> getRoot(void) const{
      return this->root_link_;
    };

    /**
     * @function getLink
     */
    boost::shared_ptr<const Link> getLink(const std::string& name) const {
      boost::shared_ptr<const Link> ptr;
      if (this->links_.find(name) == this->links_.end())
	ptr.reset();
      else
	ptr = this->links_.find(name)->second;
      return ptr;
    };
    
    /**
     * @function getJoint
     */
    boost::shared_ptr<const Joint> getJoint(const std::string& name) const {
      boost::shared_ptr<const Joint> ptr;
      if (this->joints_.find(name) == this->joints_.end())
	ptr.reset();
      else
	ptr = this->joints_.find(name)->second;
      return ptr;
    };
    
    /**
     * @function getName
     */
    const std::string& getName() const {
      return name_;
    };
    
    /**
     * @function getLinks()
     */
    void getLinks(std::vector<boost::shared_ptr<Link> >& links) const {
      for (std::map<std::string,boost::shared_ptr<Link> >::const_iterator link = this->links_.begin();
	   link != this->links_.end(); link++)
	{
	  links.push_back(link->second);
	}
    };
    
    /**
     * @function clear()
     */
    void clear() {
      name_.clear();
      this->links_.clear();
      this->joints_.clear();
      this->materials_.clear();
      this->root_link_.reset();
    };
    
    /**
     * @function getLink()
     * @brief non-const getLink()
     */
    void getLink(const std::string& name,boost::shared_ptr<Link> &link) const {
      boost::shared_ptr<Link> ptr;
      if (this->links_.find(name) == this->links_.end())
	ptr.reset();
      else
	ptr = this->links_.find(name)->second;
      link = ptr;
    };
    
    /**
     * @function getMaterial
     * @brief non-const getMaterial()
     */
    boost::shared_ptr<Material> getMaterial(const std::string& name) const {
      boost::shared_ptr<Material> ptr;
      if (this->materials_.find(name) == this->materials_.end())
	ptr.reset();
      else
	ptr = this->materials_.find(name)->second;
      return ptr;
    };
    
    /**
     * @function initTree
     * @brief Loop through all joints, for every link, assign children links and children joints
     */
    void initTree(std::map<std::string, std::string> &parent_link_tree) {
      
      for (std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = this->joints_.begin();
	   joint != this->joints_.end(); joint++) {
	std::string parent_link_name = joint->second->parent_link_name;
	std::string child_link_name = joint->second->child_link_name;
	
	if (parent_link_name.empty() || child_link_name.empty())
	  {
	    throw ParseError("Joint [" + joint->second->name + "] is missing a parent and/or child link specification.");
	  }
	else
	  {
	    // find child and parent links
	    boost::shared_ptr<Link> child_link, parent_link;
	    this->getLink(child_link_name, child_link);
	    if (!child_link)
	      {
		throw ParseError("child link [" + child_link_name + "] of joint [" + joint->first + "] not found");
	      }
	    this->getLink(parent_link_name, parent_link);
	    if (!parent_link)
	      {
		throw ParseError("parent link [" + parent_link_name + "] of joint [" + joint->first + "] not found.  This is not valid according to the URDF spec. Every link you refer to from a joint needs to be explicitly defined in the robot description. To fix this problem you can either remove this joint [" + joint->first + "] from your urdf file, or add \"<link name=\"" + parent_link_name + "\" />\" to your urdf file.");
	      }
	    
	    //set parent link for child link
	    child_link->setParent(parent_link);
	    
	    //set parent joint for child link        
	    child_link->parent_joint = joint->second;
	    
	    //set child joint for parent link
	    parent_link->child_joints.push_back(joint->second);
	    
	    //set child link for parent link
	    parent_link->child_links.push_back(child_link);
	      
	    // fill in child/parent string map
	    parent_link_tree[child_link->name] = parent_link_name;
	  }
      }
    }
    
    /**
     * @function initRoot
     */
    void initRoot(const std::map<std::string, std::string> &parent_link_tree) { 
      this->root_link_.reset();
      
      // find the links that have no parent in the tree
      for (std::map<std::string, boost::shared_ptr<Link> >::const_iterator l=this->links_.begin(); l!=this->links_.end(); l++)  
	{
	  std::map<std::string, std::string >::const_iterator parent = parent_link_tree.find(l->first);
	  if (parent == parent_link_tree.end())
	    {
	      // store root link
	      if (!this->root_link_)
		{
		  getLink(l->first, this->root_link_);
		}
	      // we already found a root link
	      else
		{
		  throw ParseError("Two root links found: [" + this->root_link_->name + "] and [" + l->first + "]");
		}
	    }
	}
      if (!this->root_link_)
	{
	  throw ParseError("No root link found. The robot xml is not a valid tree.");
	}
    }
    
    
    /// \brief complete list of Links
    std::map<std::string, boost::shared_ptr<Link> > links_;
    /// \brief complete list of Joints
    std::map<std::string, boost::shared_ptr<Joint> > joints_;
    /// \brief complete list of Materials
    std::map<std::string, boost::shared_ptr<Material> > materials_;
    
    std::string name_;

    /// ModelInterface is restricted to a tree for now, which means there exists one root link
    ///  typically, root link is the world(inertial).  Where world is a special link
    /// or is the root_link_ the link attached to the world by PLANAR/FLOATING joint?
    ///  hmm...
    boost::shared_ptr<Link> root_link_;
    
  };

}

#endif /** URDF_INTERFACE_MODEL_H */
