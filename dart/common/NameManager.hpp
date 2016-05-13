/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_COMMON_NAMEMANAGER_HPP_
#define DART_COMMON_NAMEMANAGER_HPP_

#include <map>
#include <string>

namespace dart {
namespace common {

/// \brief class NameManager
///
/// Typical usage:
/// \code{.cpp}
/// using namespace dart;
///
/// NameManager<BodyNode*> nameMgr;
///
/// BodyNode* bodyNode = new BodyNode();
/// std::string name = "Link";
///
/// if (!nameMgr.hasName(name)
///   nameMgr.addName(name, bodyNode);  // "Link"
/// else
///   name = nameMgr.issueNewNameAndAdd(name, bodyNode);  // "Link1"
///
/// bodyNode->setName(name);
/// \endcode
template <typename T>
class NameManager
{
public:
  /// Constructor
  NameManager(const std::string& _managerName = "default",
              const std::string& _defaultName = "default");

  /// Destructor
  virtual ~NameManager() = default;

  /// Set a new pattern for name generation.
  ///
  /// Use %s to indicate the base name and use %d to indicate where the number
  /// belongs. The pattern must contain both a %s and a %d.
  ///
  /// Examples:
  /// "%s(%d)" : name -> name(1) -> name(2)
  /// "%d-%s" : name -> 1-name -> 2-name
  ///
  /// returns false if the pattern was invalid (i.e. did not contain b
  /// oth %s and %d)
  bool setPattern(const std::string& _newPattern);

  /// Issue new unique combined name of given base name and number suffix
  std::string issueNewName(const std::string& _name) const;

  /// Call issueNewName() and add the result to the map
  std::string issueNewNameAndAdd(const std::string& _name, const T& _obj);

  /// Add an object to the map
  bool addName(const std::string& _name, const T& _obj);

  /// Remove an object from the Manager based on its name
  bool removeName(const std::string& _name);

  /// Remove an object from the Manager based on reverse lookup
  bool removeObject(const T& _obj);

  /// Remove _name using the forward lookup and _obj using the reverse lookup.
  /// This will allow you to add _obj under the name _name without any conflicts
  void removeEntries(const std::string& _name, const T& _obj);

  /// Clear all the objects
  void clear();

  /// Return true if the name is contained
  bool hasName(const std::string& _name) const;

  /// Return true if the object is contained
  bool hasObject(const T& _obj) const;

  /// Get the number of the objects currently stored by the NameManager
  std::size_t getCount() const;

  /// Get object by given name
  /// \param[in] _name
  ///   Name of the requested object
  /// \return
  ///   The object if it exists, or nullptr if it does not exist
  T getObject(const std::string& _name) const;

  /// Use a reverse lookup to get the name that the manager has _obj listed
  /// under. Returns an empty string if it is not in the list.
  std::string getName(const T& _obj) const;

  /// Change the name of a currently held object. This will do nothing if the
  /// object is already using _newName or if the object is not held by this
  /// NameManager.
  ///
  /// If the object is held, its new name is returned (which might
  /// be different than _newName if there was a duplicate naming conflict). If
  /// the object is not held, an empty string will be returned.
  std::string changeObjectName(const T& _obj, const std::string& _newName);

  /// Set the name that will be provided to objects passed in with an empty
  /// string for a name
  void setDefaultName(const std::string& _defaultName);

  /// Get the name that will be provided to objects passed in with an empty
  /// string for a name
  const std::string& getDefaultName() const;

  /// Set the name of this NameManager so that it can be printed in error reports
  void setManagerName(const std::string& _managerName);

  /// Get the name of this NameManager
  const std::string& getManagerName() const;

protected:
  /// Name of this NameManager. This is used to report errors.
  std::string mManagerName;

  /// Map of objects that have been added to the NameManager
  std::map<std::string, T> mMap;

  /// Reverse map of objects that have been added to the NameManager
  std::map<T, std::string> mReverseMap;

  /// String which will be used as a name for any object which is passed in with
  /// an empty string name
  std::string mDefaultName;

  /// Internal variable used to arrange the text when resolving duplicate names
  bool mNameBeforeNumber;

  /// The chunk of text that gets prepended to a duplicate name
  std::string mPrefix;

  /// The chunk of text that comes between a duplicate name and its duplication
  /// number
  std::string mInfix;

  /// The chunk of text that gets appended to a duplicate name
  std::string mAffix;
};

} // namespace common
} // namespace dart

#include "dart/common/detail/NameManager.hpp"

#endif // DART_COMMON_NAMEMANAGER_HPP_
