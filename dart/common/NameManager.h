#ifndef DART_COMMON_NAMEMANAGER_H_
#define DART_COMMON_NAMEMANAGER_H_

#include <map>
#include <sstream>
#include "dart/common/Console.h"

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
///   nameMgr.addName(name);  // "Link"
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
  NameManager(const std::string& _defaultName="default") :
    mDefaultName(_defaultName),
    mNameBeforeNumber(true),
    mPrefix(""), mInfix("("), mAffix(")") {}

  std::string mDefaultName;

  /// Destructor
  virtual ~NameManager() {}

  /// Set a new pattern for name generation.
  ///
  /// Use %s to indicate the base name and use %d to indicate where the number belongs.
  /// The pattern must contain both a %s and a %d.
  ///
  /// Examples:
  /// "%s(%d)" : name -> name(1) -> name(2)
  /// "%d-%s" : name -> 1-name -> 2-name
  ///
  /// returns false if the pattern was invalid (i.e. did not contain both %s and %d)
  bool setPattern(const std::string& newPattern)
  {
    size_t name_start = newPattern.find("%s");
    size_t number_start = newPattern.find("%d");

    if(name_start == std::string::npos || number_start == std::string::npos)
      return false;

    if(name_start < number_start)
      mNameBeforeNumber = true;
    else
      mNameBeforeNumber = false;

    size_t prefix_end = std::min(name_start, number_start);
    size_t infix_end = std::max(name_start, number_start);

    mPrefix = newPattern.substr(0, prefix_end);
    mInfix = newPattern.substr(prefix_end+2, infix_end-prefix_end-2);
    mAffix = newPattern.substr(infix_end+2);

    return true;
  }

  /// Issue new unique combined name of given base name and number suffix
  std::string issueNewName(const std::string& _name) const
  {
    if(!hasName(_name))
      return _name;

    int count = 1;
    std::string newName;
    do
    {
      std::stringstream ss;
      if(mNameBeforeNumber)
          ss << mPrefix << _name << mInfix << count++ << mAffix;
      else
          ss << mPrefix << count++ << mInfix << _name << mAffix;
      newName = ss.str();
    } while (hasName(newName));

    return newName;
  }

  /// Call issueNewName() and add the result to the map
  std::string issueNewNameAndAdd(const std::string& _name, T _obj)
  {
    const std::string& checkEmpty = _name.empty()? mDefaultName : _name;
    const std::string& newName = issueNewName(checkEmpty);
    addName(newName, _obj);

    return newName;
  }

  /// Add an object to the map
  bool addName(const std::string& _name, const T& _obj)
  {
    if (_name.empty())
    {
      dtwarn << "Empty name is not allowed.\n";
      return false;
    }

    if (hasName(_name))
    {
      dtwarn << "Name [" << _name << "] already exist.\n";
      return false;
    }

    mMap.insert(std::pair<std::string, T>(_name, _obj));

    return true;
  }

  /// Remove an object from the map based on its name
  bool removeName(const std::string& _name)
  {
    typename std::map<std::string, T>::iterator it = mMap.find(_name);

    if (it == mMap.end())
      return false;

    mMap.erase(it);

    return true;
  }

  /// Clear all the objects
  void clear()
  {
    mMap.clear();
  }

  /// Return true if the name is contained
  bool hasName(const std::string& _name) const
  {
    return (mMap.find(_name) != mMap.end());
  }

  /// Get the number of the objects
  size_t getCount() const
  {
    return mMap.size();
  }

  /// Get object by given name
  /// \param[in] _name
  ///   Name of the requested object
  /// \return
  ///   The object if it exists, or NULL if it does not exist
  T getObject(const std::string& _name) const
  {
    typename std::map<std::string, T>::const_iterator result = mMap.find(_name);

    if (result != mMap.end())
      return result->second;
    else
      return NULL;
  }

protected:

  /// Name map
  std::map<std::string, T> mMap;

  bool mNameBeforeNumber;
  std::string mPrefix;
  std::string mInfix;
  std::string mAffix;

};


} // namespace common
} // namespace dart

#endif // DART_COMMON_NAMEMANAGER_H_
