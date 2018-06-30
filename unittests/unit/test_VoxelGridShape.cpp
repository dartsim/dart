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

#include <iostream>
#include <gtest/gtest.h>
#include <octomap/octomap.h>
#include "TestHelpers.hpp"

#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/VskParser.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

using namespace std;
using namespace octomap;

//==============================================================================
void print_query_info(point3d query, OcTreeNode* node)
{
  if (node)
  {
    cout << "occupancy probability at " << query << ":\t "
         << node->getOccupancy() << endl;
  }
  else
  {
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
  }
}

//==============================================================================
TEST(VoxelGridShape, SimpleExample)
{
  cout << endl;
  cout << "generating example map" << endl;

  OcTree tree(0.1); // create empty tree with resolution 0.1

  // insert some measurements of occupied cells

  for (int x = -20; x < 20; x++)
  {
    for (int y = -20; y < 20; y++)
    {
      for (int z = -20; z < 20; z++)
      {
        point3d endpoint(
            static_cast<float>(x) * 0.05f,
            static_cast<float>(y) * 0.05f,
            static_cast<float>(z) * 0.05f);
        /*auto node = */ tree.updateNode(
            endpoint, true); // integrate 'occupied' measurement
        //        std::cout << "Updated node: (" << endpoint << "), (" <<
        //        node->getOccupancy() << ")" << std::endl;
      }
    }
  }

  // insert some measurements of free cells

  for (int x = -30; x < 30; x++)
  {
    for (int y = -30; y < 30; y++)
    {
      for (int z = -30; z < 30; z++)
      {
        point3d endpoint(
            static_cast<float>(x) * 0.02f - 1.0f,
            static_cast<float>(y) * 0.02f - 1.0f,
            static_cast<float>(z) * 0.02f - 1.0f);
        /*auto node = */ tree.updateNode(
            endpoint, false); // integrate 'free' measurement
        //        std::cout << "Updated node: (" << endpoint << "), (" <<
        //        node->getOccupancy() << ")" << std::endl;
      }
    }
  }

  cout << endl;
  cout << "performing some queries:" << endl;

  point3d query(0., 0., 0.);
  OcTreeNode* result = tree.search(query);
  print_query_info(query, result);

  query = point3d(-1., -1., -1.);
  result = tree.search(query);
  print_query_info(query, result);

  query = point3d(1., 1., 1.);
  result = tree.search(query);
  print_query_info(query, result);

  cout << endl;
  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt"
       << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl << endl;
}

//==============================================================================
TEST(VoxelGridShape, Normals)
{
  cout << endl;
  cout << "generating example map" << endl;

  OcTree tree(0.1); // create empty tree with resolution 0.1

  // insert some measurements of occupied cells

  for (int x = -20; x < 20; x++)
  {
    for (int y = -20; y < 20; y++)
    {
      for (int z = -20; z < 20; z++)
      {
        point3d endpoint(
            static_cast<float>(x) * 0.05f,
            static_cast<float>(y) * 0.05f,
            static_cast<float>(z) * 0.05f);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells

  for (int x = -30; x < 30; x++)
  {
    for (int y = -30; y < 30; y++)
    {
      for (int z = -30; z < 30; z++)
      {
        point3d endpoint(
            static_cast<float>(x) * 0.02f - 1.0f,
            static_cast<float>(y) * 0.02f - 1.0f,
            static_cast<float>(z) * 0.02f - 1.0f);
        tree.updateNode(endpoint, false); // integrate 'free' measurement
      }
    }
  }

  cout << endl;
  cout << "performing some queries around the desired voxel:" << endl;

  point3d query;
  //  OcTreeNode* result = NULL;

  //  for(float z = -0.6f; z < -0.21f; z += 0.1f){
  //    for(float y = -0.6f; y < -0.21f; y += 0.1f){
  //      for(float x = -0.6f; x < -0.21f; x += 0.1f){
  //        query = point3d(x, y, z);
  //        result = tree.search(query);
  //        print_query_info(query, result);
  //      }
  //    }
  //  }

  query = point3d(-0.5f, -0.4f, -0.4f);
  //  result = tree.search(query);

  vector<point3d> normals;
  if (tree.getNormals(query, normals))
  {

    cout << endl;
    string s_norm = (normals.size() > 1) ? " normals " : " normal ";
    cout << "MC algorithm gives " << normals.size() << s_norm << "in voxel at "
         << query << endl;
    for (unsigned i = 0; i < normals.size(); ++i)
      cout << "\t" << normals[i].x() << "; " << normals[i].y() << "; "
           << normals[i].z() << endl;
  }
  else
  {
    cout << "query point unknown (no normals)\n";
  }
}
