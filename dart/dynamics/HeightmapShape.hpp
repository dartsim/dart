/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_DYNAMICS_HEIGHTMAPSHAPE_HPP_
#define DART_DYNAMICS_HEIGHTMAPSHAPE_HPP_

#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {

/**
 * \brief Shape for a height map.
 */
class HeightmapShape : public Shape
{
public:
  /// \brief Data type used for height map. Could be made template.
  /// At this point, only double and float are supported.
  /// short and char can be added at a later point.
  using HeightType = float;

  using HeightField = Eigen::Matrix<HeightType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  /// \brief Constructor.
  explicit HeightmapShape();

  /// \brief Destructor.
  virtual ~HeightmapShape();

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns shape type for this class
  static const std::string& getStaticType();

  // Documentation inherited.
  // This base class computes the intertia based on the bounding box.
  // Subclasses may choose to provide a more accurate computation of the inertia
  virtual Eigen::Matrix3d computeInertia(double mass) const override;

  /// \brief Set scale of this heightmap.
  /// \param[in] scale scale of the height map.
  void setScale(const Eigen::Vector3d& scale);

  /// \brief Get scale of this heightmap.
  const Eigen::Vector3d& getScale() const;

  /// \brief Set the height field.
  /// The data in \e heights will be copied locally.
  /// It would be nice to have the option to use the values in
  /// \e heights directly instead of copying them locally to a vector in this
  /// class (this would avoid any data being kept twice). However some
  /// collision engine implementations may require to transform the height
  /// values, e.g. bullet needs the y values flipped. Therefore,
  /// a (mutable) copy of the height values passed in \e heights will be kept
  /// in this class. The copied data can be modified via
  /// getHeightFieldModifiable() and with flipY().
  /// 
  /// \param[in] width width of the field (x axis)
  /// \param[in] depth depth of the field (-y axis)
  /// \param[in] heights the height data of size \e width * \e depth.
  //    The heights are interpreted as z values, while \e width goes in x 
  //    direction and \e depth in -y (it goes to -y because traditionally
  //    images are read from top row to bottom row).
  //    In the geometry which is to be generated from this shape, the min/max
  //    height value is also the min/max z value (so if the minimum height 
  //    value is -100, the lowest terrain point will be -100, times the z
  //    scale to be applied).
  void setHeightField(const size_t& width, const size_t& depth,
                      const std::vector<HeightType>& heights);

  /// \brief Get the height field.
  const HeightField& getHeightField() const;
  
  /// \brief Gets the modified height field. See also setHeightField().
  HeightField& getHeightFieldModifiable() const;

  /// \brief Flips the y values in the height field.
  void flipY() const; 

  /// \brief Get the width dimension of the height field
  size_t getWidth() const;
  
  /// \brief Get the height dimension of the height field
  size_t getDepth() const;

  /// \brief Get the minimum height set by setHeightField()
  HeightType getMinHeight() const;
  
  /// \brief Get the maximum height set by setHeightField()
  HeightType getMaxHeight() const;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  /// Documentation inherited.
  /// This base class provides a simple implementation which returns
  /// the volume of the bounding box. Subclasses may opt to provide a more
  /// accurate computation of the volume.
  virtual void updateVolume() const override;
 
  /// \brief Computes the bounding box of the height field.
  /// \param[out] min mininum of box
  /// \param[out] max maxinum of box
  void computeBoundingBox(Eigen::Vector3d& min,
                          Eigen::Vector3d& max) const;

private:
  /// \brief scale of the heightmap
  Eigen::Vector3d mScale;

  /// \brief height field
  mutable HeightField mHeights;

  /// \brief minimum and maximum heights.
  /// Is computed each time the height field is set with setHeightField().
  HeightType mMinHeight, mMaxHeight;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_HEIGHTMAPSHAPE_HPP_
