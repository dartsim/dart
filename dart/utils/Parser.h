/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_UTILS_PARSER_H
#define DART_UTILS_PARSER_H

#include <string>
#include <Eigen/Dense>
// TinyXML-2 Library
// http://www.grinninglizard.com/tinyxml2/index.html
#include <tinyxml2.h>

#include "dart/math/MathTypes.h"

namespace dart {
namespace utils {

std::string toString(bool _v);
std::string toString(int _v);
std::string toString(unsigned int _v);
std::string toString(float _v);
std::string toString(double _v);
std::string toString(char _v);
std::string toString(const Eigen::Vector2d& _v);
std::string toString(const Eigen::Vector3d& _v);
std::string toString(const Eigen::Isometry3d& _v);

bool              toBool      (const std::string& _str);
int               toInt       (const std::string& _str);
unsigned int      toUInt      (const std::string& _str);
float             toFloat     (const std::string& _str);
double            toDouble    (const std::string& _str);
char              toChar      (const std::string& _str);
Eigen::Vector2d   toVector2d  (const std::string& _str);
Eigen::Vector3d   toVector3d  (const std::string& _str);
Eigen::Vector3i   toVector3i  (const std::string& _str);
Eigen::Vector6d   toVector6d  (const std::string& _str);
Eigen::Isometry3d toIsometry3d(const std::string& _str);

std::string       getValueString    (tinyxml2::XMLElement* _parentElement, const std::string& _name);
bool              getValueBool      (tinyxml2::XMLElement* _parentElement, const std::string& _name);
int               getValueInt       (tinyxml2::XMLElement* _parentElement, const std::string& _name);
unsigned int      getValueUInt      (tinyxml2::XMLElement* _parentElement, const std::string& _name);
float             getValueFloat     (tinyxml2::XMLElement* _parentElement, const std::string& _name);
double            getValueDouble    (tinyxml2::XMLElement* _parentElement, const std::string& _name);
char              getValueChar      (tinyxml2::XMLElement* _parentElement, const std::string& _name);
Eigen::Vector2d   getValueVector2d  (tinyxml2::XMLElement* _parentElement, const std::string& _name);
Eigen::Vector3d   getValueVector3d  (tinyxml2::XMLElement* _parentElement, const std::string& _name);
Eigen::Vector3i   getValueVector3i  (tinyxml2::XMLElement* _parentElement, const std::string& _name);
Eigen::Vector6d   getValueVector6d  (tinyxml2::XMLElement* _parentElement, const std::string& _name);
Eigen::Isometry3d getValueIsometry3d(tinyxml2::XMLElement* _parentElement, const std::string& _name);

void openXMLFile(tinyxml2::XMLDocument& doc, const char* const filename);
bool hasElement(tinyxml2::XMLElement* _parentElement, const std::string& _name);
tinyxml2::XMLElement* getElement(tinyxml2::XMLElement* _parentElement, const std::string& _name);
std::string getAttribute(tinyxml2::XMLElement* element, const char* const name);
void getAttribute(tinyxml2::XMLElement* element, const char* const name, double* d);

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
/// \brief
class ElementEnumerator
{
public:
    /// \brief
    ElementEnumerator(tinyxml2::XMLElement* _parent, const std::string& _name);

    /// \brief
    ~ElementEnumerator();

    /// \brief
    bool valid() const;

    /// \brief
    bool next();

    /// \brief
    tinyxml2::XMLElement* get() const { return m_current; }

    /// \brief
    tinyxml2::XMLElement* operator->() const { return m_current; }

    /// \brief
    tinyxml2::XMLElement& operator*() const { return *m_current; }

    /// \brief
    bool operator==(const ElementEnumerator& _rhs) const;

    /// \brief
    ElementEnumerator & operator=(const ElementEnumerator& _rhs);

private:
    /// \brief
    std::string m_name;

    /// \brief
    tinyxml2::XMLElement* m_parent;

    /// \brief
    tinyxml2::XMLElement* m_current;
};

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_PARSER_H
