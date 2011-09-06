/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

/** \author Jia Pan */

#ifndef COLLISION_CHECKING_COLLISION_GEOM_H
#define COLLISION_CHECKING_COLLISION_GEOM_H

#include <fstream>
#include <vector>
#include <boost/math/constants/constants.hpp>
#include <LinearMath/btTransform.h>
#include "collision.h"

namespace collision_checking
{

inline void saveOBjFile(const std::string& name, const std::vector<Vec3f>& points, const std::vector<Triangle>& tri_indices)
{
  std::ofstream file(name.c_str());
  file << points.size() << " " << tri_indices.size() << std::endl;
  for(unsigned int i = 0; i < points.size(); ++i)
  {
    file << "v " << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;
  }
  for(unsigned int i = 0; i < tri_indices.size(); ++i)
  {
    file << "f " << tri_indices[i][0] + 1 << " " << tri_indices[i][1] + 1 << " " << tri_indices[i][2] + 1 << std::endl;
  }
}

struct CollisionGeom
{
  CollisionGeom()
  {
    ccd = false;
    t1.setIdentity();
    t2.setIdentity();
  }

  virtual ~CollisionGeom()
  {}

  virtual BVH_CollideResult collide(CollisionGeom* other, int num_contacts = 0) { BVH_CollideResult res; return res; }

  virtual void applyTransform(const btTransform& pose, bool refit = true, bool bottomup = true)
  {
    t2 = t1;
    t1 = pose;
  }

  virtual void computeAABB() {}

  bool ccd;
  btTransform t1; // current
  btTransform t2; // previous

  AABB aabb;
};



template<typename BV>
struct CollisionMesh;
template<typename BV>
CollisionMesh<BV>* makeMesh(const std::vector<Vec3f>& points, const std::vector<Triangle>& tri_indices);
template<typename BV>
CollisionMesh<BV>* makeBox(double a, double b, double c);
template<typename BV>
CollisionMesh<BV>* makeCylinder(double r, double h, unsigned int tot = 16);
template<typename BV>
CollisionMesh<BV>* makeSphere(double r, unsigned int seg = 16, unsigned int ring = 16);

template<typename BV>
struct CollisionMesh : public CollisionGeom
{
private:
  CollisionMesh() : CollisionGeom()
  {
  }

public:
  friend CollisionMesh<BV>* makeMesh<>(const std::vector<Vec3f>& points, const std::vector<Triangle>& tri_indices);
  friend CollisionMesh<BV>* makeBox<>(double a, double b, double c);
  friend CollisionMesh<BV>* makeCylinder<>(double r, double h, unsigned int tot);
  friend CollisionMesh<BV>* makeSphere<>(double r, unsigned int seg, unsigned int ring);

  // TODO: get rid of dynamic_cast<> :(
  BVH_CollideResult collide(CollisionGeom* other, int num_max_contacts = 0)
  {
    BVH_CollideResult res;
    res.num_max_contacts = num_max_contacts;
    CollisionMesh<BV>* othermesh = dynamic_cast<CollisionMesh<BV>* >(other);
    if(othermesh)
    {
      collision_checking::collide(model, othermesh->model, &res);
    }

    return res;
  }

  void applyTransform(const btTransform& pose, bool refit = false, bool bottomup = true)
  {
    t2 = t1;
    t1 = pose;

    std::vector<Vec3f> points;
    for(int i = 0; i < model.num_vertices; ++i)
    {
      const Vec3f& p = model.vertices[i];
      btVector3 v(p[0], p[1], p[2]);
      v = pose * v;
      points[i] = Vec3f(v.x(), v.y(), v.z());
    }

    if(ccd)
    {
      model.beginUpdateModel();
      model.updateSubModel(points);
      model.endUpdateModel(refit, bottomup);
    }
    else
    {
      model.beginReplaceModel();
      model.replaceSubModel(points);
      model.endReplaceModel(refit, bottomup);
    }

    computeAABB();
  }

  void computeAABB()
  {
    AABB aabb_;
    if(ccd)
    {
      for(int i = 0; i < model.num_vertices; ++i)
      {
        aabb_ += model.vertices[i];
        aabb_ += model.prev_vertices[i];
      }
    }
    else
    {
      for(int i = 0; i < model.num_vertices; ++i)
      {
        aabb_ += model.vertices[i];
      }
    }

    aabb = aabb_;
  }


  BVHModel<BV> model;
};


/** \brief Specialization for OBB */
template<>
struct CollisionMesh<OBB> : public CollisionGeom
{
private:
  CollisionMesh() : CollisionGeom()
  {
  }

public:
  friend CollisionMesh<OBB>* makeMesh<>(const std::vector<Vec3f>& points, const std::vector<Triangle>& tri_indices);
  friend CollisionMesh<OBB>* makeBox<>(double a, double b, double c);
  friend CollisionMesh<OBB>* makeCylinder<>(double r, double h, unsigned int tot);
  friend CollisionMesh<OBB>* makeSphere<>(double r, unsigned int seg, unsigned int ring);

  // TODO: get rid of dynamic_cast<> :(
  BVH_CollideResult collide(CollisionGeom* other, int num_max_contacts = 0)
  {
    BVH_CollideResult res;
    res.num_max_contacts = num_max_contacts;
    CollisionMesh<OBB>* othermesh = dynamic_cast<CollisionMesh<OBB>* >(other);
    if(othermesh)
    {
      Vec3f R1[3];
      Vec3f R2[3];
      Vec3f T1;
      Vec3f T2;
      btVector3 t = t1.getOrigin();
      T1 = Vec3f(t.x(), t.y(), t.z());
      btMatrix3x3 r = t1.getBasis();
      for(int i = 0; i < 3; ++i)
        R1[i] = Vec3f(r[i].x(), r[i].y(), r[i].z());

      t = othermesh->t1.getOrigin();
      T2 = Vec3f(t.x(), t.y(), t.z());
      r = othermesh->t1.getBasis();
      for(int i = 0; i < 3; ++i)
        R2[i] = Vec3f(r[i].x(), r[i].y(), r[i].z());

      collision_checking::collide(model, R1, T1, othermesh->model, R2, T2, &res);
    }

    return res;
  }

  void applyTransform(const btTransform& pose, bool refit = true, bool bottomup = true)
  {
    t2 = t1;
    t1 = pose;

    computeAABB();
  }

  void computeAABB()
  {
    AABB aabb_;

    /* Compute an exact AABB from vertices, slow :(
    if(ccd)
    {
      for(int i = 0; i < model.num_vertices; ++i)
      {
        Vec3f p = model.vertices[i];
        btVector3 v(p[0], p[1], p[2]);
        btVector3 v1 = t1 * v;
        btVector3 v2 = t2 * v;

        aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
        aabb_ += Vec3f(v2.x(), v2.y(), v2.z());
      }
    }
    else
    {
      for(int i = 0; i < model.num_vertices; ++i)
      {
        Vec3f p = model.vertices[i];
        btVector3 v(p[0], p[1], p[2]);
        btVector3 v1 = t1 * v;
        aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
      }
    }
    */

    /* So we only compute a rough AABB from rotated OBB */

    if(ccd)
    {
      BVNode<OBB>* obb = model.bvs;
      Vec3f p;
      btVector3 v;
      btVector3 v1;

      Vec3f& axis0 = obb->bv.axis[0];
      Vec3f& axis1 = obb->bv.axis[1];
      Vec3f& axis2 = obb->bv.axis[2];
      Vec3f& To = obb->bv.To;
      Vec3f& extent = obb->bv.extent;

      p = To + axis0 * extent[0] + axis1 * extent[1] + axis2 * extent[2];
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
      v1 = t2 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * extent[0] + axis1 * extent[1] + axis2 * (-extent[2]);
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
      v1 = t2 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * extent[0] + axis1 * (-extent[1]) + axis2 * extent[2];
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
      v1 = t2 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * extent[0] + axis1 * (-extent[1]) + axis2 * (-extent[2]);
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
      v1 = t2 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * (-extent[0]) + axis1 * extent[1] + axis2 * extent[2];
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
      v1 = t2 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * (-extent[0]) + axis1 * extent[1] + axis2 * (-extent[2]);
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
      v1 = t2 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * (-extent[0]) + axis1 * (-extent[1]) + axis2 * extent[2];
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
      v1 = t2 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * (-extent[0]) + axis1 * (-extent[1]) + axis2 * (-extent[2]);
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
      v1 = t2 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
    }
    else
    {
      BVNode<OBB>* obb = model.bvs;
      Vec3f p;
      btVector3 v;
      btVector3 v1;

      Vec3f& axis0 = obb->bv.axis[0];
      Vec3f& axis1 = obb->bv.axis[1];
      Vec3f& axis2 = obb->bv.axis[2];
      Vec3f& To = obb->bv.To;
      Vec3f& extent = obb->bv.extent;

      p = To + axis0 * extent[0] + axis1 * extent[1] + axis2 * extent[2];
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * extent[0] + axis1 * extent[1] + axis2 * (-extent[2]);
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * extent[0] + axis1 * (-extent[1]) + axis2 * extent[2];
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * extent[0] + axis1 * (-extent[1]) + axis2 * (-extent[2]);
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * (-extent[0]) + axis1 * extent[1] + axis2 * extent[2];
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * (-extent[0]) + axis1 * extent[1] + axis2 * (-extent[2]);
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * (-extent[0]) + axis1 * (-extent[1]) + axis2 * extent[2];
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());

      p = To + axis0 * (-extent[0]) + axis1 * (-extent[1]) + axis2 * (-extent[2]);
      v = btVector3(p[0], p[1], p[2]);
      v1 = t1 * v;
      aabb_ += Vec3f(v1.x(), v1.y(), v1.z());
    }

    aabb = aabb_;
  }

  BVHModel<OBB> model;
};


template<typename BV>
inline CollisionMesh<BV>* makeMesh(const std::vector<Vec3f>& points, const std::vector<Triangle>& tri_indices)
{
  CollisionMesh<BV>* m = new CollisionMesh<BV>;
  m->model.beginModel();
  m->model.addSubModel(points, tri_indices);
  m->model.endModel();
  m->computeAABB();

  return m;
}

template<typename BV>
inline CollisionMesh<BV>* makeBox(double a, double b, double c)
{
  std::vector<Vec3f> points(8);
  std::vector<Triangle> tri_indices(12);
  points[0] = Vec3f(0.5 * a, -0.5 * b, 0.5 * c);
  points[1] = Vec3f(0.5 * a, 0.5 * b, 0.5 * c);
  points[2] = Vec3f(-0.5 * a, 0.5 * b, 0.5 * c);
  points[3] = Vec3f(-0.5 * a, -0.5 * b, 0.5 * c);
  points[4] = Vec3f(0.5 * a, -0.5 * b, -0.5 * c);
  points[5] = Vec3f(0.5 * a, 0.5 * b, -0.5 * c);
  points[6] = Vec3f(-0.5 * a, 0.5 * b, -0.5 * c);
  points[7] = Vec3f(-0.5 * a, -0.5 * b, -0.5 * c);

  tri_indices[0] = Triangle(0, 4, 1);
  tri_indices[1] = Triangle(1, 4, 5);
  tri_indices[2] = Triangle(2, 5, 3);
  tri_indices[3] = Triangle(3, 6, 7);
  tri_indices[4] = Triangle(3, 0, 2);
  tri_indices[5] = Triangle(2, 0, 1);
  tri_indices[6] = Triangle(6, 5, 7);
  tri_indices[7] = Triangle(7, 5, 4);
  tri_indices[8] = Triangle(1, 5, 2);
  tri_indices[9] = Triangle(2, 5, 6);
  tri_indices[10] = Triangle(3, 7, 0);
  tri_indices[11] = Triangle(0, 7, 4);


  CollisionMesh<BV>* m = new CollisionMesh<BV>;
  m->model.beginModel();
  m->model.addSubModel(points, tri_indices);
  m->model.endModel();
  m->computeAABB();

  return m;
}

template<typename BV>
inline CollisionMesh<BV>* makeCylinder(double r, double h, unsigned int tot)
{
  std::vector<Vec3f> points;
  std::vector<Triangle> tri_indices;

  double phi, phid;
  const double pi = boost::math::constants::pi<double>();
  phid = pi * 2 / tot;
  phi = 0;

  double circle_edge = phid * r;
  //unsigned int h_num = ceil(h / circle_edge);
  unsigned int h_num = 1;
  double hd = h / h_num;

  for(unsigned int i = 0; i < tot; ++i)
    points.push_back(Vec3f(r * cos(phi + phid * i), r * sin(phi + phid * i), h / 2));

  for(unsigned int i = 0; i < h_num - 1; ++i)
  {
    for(unsigned int j = 0; j < tot; ++j)
    {
      points.push_back(Vec3f(r * cos(phi + phid * j), r * sin(phi + phid * j), h / 2 - (i + 1) * hd));
    }
  }

  for(unsigned int i = 0; i < tot; ++i)
    points.push_back(Vec3f(r * cos(phi + phid * i), r * sin(phi + phid * i), - h / 2));

  points.push_back(Vec3f(0, 0, h / 2));
  points.push_back(Vec3f(0, 0, -h / 2));

  for(unsigned int i = 0; i < tot; ++i)
  {
    Triangle tmp((h_num + 1) * tot, i, ((i == tot - 1) ? 0 : (i + 1)));
    tri_indices.push_back(tmp);
  }

  for(unsigned int i = 0; i < tot; ++i)
  {
    Triangle tmp((h_num + 1) * tot + 1, h_num * tot + i, h_num * tot + ((i == tot - 1) ? 0 : (i + 1)));
    tri_indices.push_back(tmp);
  }

  for(unsigned int i = 0; i < h_num; ++i)
  {
    for(unsigned int j = 0; j < tot; ++j)
    {
      int a, b, c, d;
      a = j;
      b = (j == tot - 1) ? 0 : (j + 1);
      c = j + tot;
      d = (j == tot - 1) ? tot : (j + 1 + tot);

      int start = i * tot;
      tri_indices.push_back(Triangle(start + b, start + a, start + c));
      tri_indices.push_back(Triangle(start + b, start + c, start + d));
    }
  }

  CollisionMesh<BV>* m = new CollisionMesh<BV>;
  m->model.beginModel();
  m->model.addSubModel(points, tri_indices);
  m->model.endModel();
  m->computeAABB();

  return m;
}

template<typename BV>
inline CollisionMesh<BV>* makeSphere(double r, unsigned int seg, unsigned int ring)
{
  std::vector<Vec3f> points;
  std::vector<Triangle> tri_indices;

  double phi, phid;
  const double pi = boost::math::constants::pi<double>();
  phid = pi * 2 / seg;
  phi = 0;

  double theta, thetad;
  thetad = pi / (ring + 1);
  theta = 0;

  for(unsigned int i = 0; i < ring; ++i)
  {
    double theta_ = theta + thetad * (i + 1);
    for(unsigned int j = 0; j < seg; ++j)
    {
      points.push_back(Vec3f(r * sin(theta_) * cos(phi + j * phid), r * sin(theta_) * sin(phi + j * phid), r * cos(theta_)));
    }
  }
  points.push_back(Vec3f(0, 0, r));
  points.push_back(Vec3f(0, 0, -r));

  for(unsigned int i = 0; i < ring - 1; ++i)
  {
    for(unsigned int j = 0; j < seg; ++j)
    {
       unsigned int a, b, c, d;
       a = i * seg + j;
       b = (j == seg - 1) ? (i * seg) : (i * seg + j + 1);
       c = (i + 1) * seg + j;
       d = (j == seg - 1) ? ((i + 1) * seg) : ((i + 1) * seg + j + 1);
       tri_indices.push_back(Triangle(a, c, b));
       tri_indices.push_back(Triangle(b, c, d));
    }
  }

  for(unsigned int j = 0; j < seg; ++j)
  {
    unsigned int a, b;
    a = j;
    b = (j == seg - 1) ? 0 : (j + 1);
    tri_indices.push_back(Triangle(ring * seg, a, b));

    a = (ring - 1) * seg + j;
    b = (j == seg - 1) ? (ring - 1) * seg : ((ring - 1) * seg + j + 1);
    tri_indices.push_back(Triangle(a, ring * seg + 1, b));
  }

  CollisionMesh<BV>* m = new CollisionMesh<BV>;
  m->model.beginModel();
  m->model.addSubModel(points, tri_indices);
  m->model.endModel();
  m->computeAABB();

  return m;
}




}

#endif
