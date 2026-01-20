/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/collision/experimental/types.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(ContactPoint, DefaultConstruction)
{
  ContactPoint cp;

  EXPECT_EQ(cp.position, Eigen::Vector3d::Zero());
  EXPECT_EQ(cp.normal, Eigen::Vector3d::UnitZ());
  EXPECT_EQ(cp.depth, 0.0);
  EXPECT_EQ(cp.object1, nullptr);
  EXPECT_EQ(cp.object2, nullptr);
  EXPECT_EQ(cp.featureIndex1, -1);
  EXPECT_EQ(cp.featureIndex2, -1);
}

TEST(ContactPoint, IsZeroNormal)
{
  EXPECT_TRUE(ContactPoint::isZeroNormal(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(ContactPoint::isZeroNormal(Eigen::Vector3d(1e-7, 0, 0)));
  EXPECT_FALSE(ContactPoint::isZeroNormal(Eigen::Vector3d::UnitX()));
  EXPECT_FALSE(ContactPoint::isZeroNormal(Eigen::Vector3d(0.001, 0, 0)));
}

TEST(ContactPoint, IsNonZeroNormal)
{
  EXPECT_FALSE(ContactPoint::isNonZeroNormal(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(ContactPoint::isNonZeroNormal(Eigen::Vector3d::UnitX()));
}

TEST(ContactManifold, DefaultConstruction)
{
  ContactManifold manifold;

  EXPECT_EQ(manifold.numContacts(), 0u);
  EXPECT_FALSE(manifold.hasContacts());
  EXPECT_EQ(manifold.getType(), ContactType::Unknown);
  EXPECT_EQ(manifold.getObject1(), nullptr);
  EXPECT_EQ(manifold.getObject2(), nullptr);
}

TEST(ContactManifold, AddContact)
{
  ContactManifold manifold;

  ContactPoint cp;
  cp.position = Eigen::Vector3d(1, 2, 3);
  cp.normal = Eigen::Vector3d::UnitX();
  cp.depth = 0.5;

  manifold.addContact(cp);

  EXPECT_EQ(manifold.numContacts(), 1u);
  EXPECT_TRUE(manifold.hasContacts());
  EXPECT_EQ(manifold.getContact(0).position, Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(manifold.getContact(0).normal, Eigen::Vector3d::UnitX());
  EXPECT_EQ(manifold.getContact(0).depth, 0.5);
}

TEST(ContactManifold, Clear)
{
  ContactManifold manifold;
  manifold.addContact(ContactPoint{});
  manifold.addContact(ContactPoint{});
  manifold.setType(ContactType::Face);

  EXPECT_EQ(manifold.numContacts(), 2u);

  manifold.clear();

  EXPECT_EQ(manifold.numContacts(), 0u);
  EXPECT_FALSE(manifold.hasContacts());
  EXPECT_EQ(manifold.getType(), ContactType::Unknown);
}

TEST(ContactManifold, GetContacts)
{
  ContactManifold manifold;

  ContactPoint cp1;
  cp1.position = Eigen::Vector3d(1, 0, 0);

  ContactPoint cp2;
  cp2.position = Eigen::Vector3d(2, 0, 0);

  manifold.addContact(cp1);
  manifold.addContact(cp2);

  auto contacts = manifold.getContacts();
  EXPECT_EQ(contacts.size(), 2u);
  EXPECT_EQ(contacts[0].position, Eigen::Vector3d(1, 0, 0));
  EXPECT_EQ(contacts[1].position, Eigen::Vector3d(2, 0, 0));
}

TEST(ContactManifold, GetSharedNormal_SingleContact)
{
  ContactManifold manifold;

  ContactPoint cp;
  cp.normal = Eigen::Vector3d::UnitY();
  manifold.addContact(cp);

  EXPECT_EQ(manifold.getSharedNormal(), Eigen::Vector3d::UnitY());
}

TEST(ContactManifold, GetSharedNormal_SameNormals)
{
  ContactManifold manifold;

  ContactPoint cp1;
  cp1.normal = Eigen::Vector3d::UnitZ();

  ContactPoint cp2;
  cp2.normal = Eigen::Vector3d::UnitZ();

  manifold.addContact(cp1);
  manifold.addContact(cp2);

  EXPECT_EQ(manifold.getSharedNormal(), Eigen::Vector3d::UnitZ());
}

TEST(ContactManifold, GetSharedNormal_DifferentNormals)
{
  ContactManifold manifold;

  ContactPoint cp1;
  cp1.normal = Eigen::Vector3d::UnitX();

  ContactPoint cp2;
  cp2.normal = Eigen::Vector3d::UnitY();

  manifold.addContact(cp1);
  manifold.addContact(cp2);

  EXPECT_EQ(manifold.getSharedNormal(), Eigen::Vector3d::Zero());
}

TEST(ContactManifold, GetSharedNormal_Empty)
{
  ContactManifold manifold;
  EXPECT_EQ(manifold.getSharedNormal(), Eigen::Vector3d::Zero());
}

TEST(ContactManifold, TypeCompatibility)
{
  ContactPoint cp;
  cp.normal = Eigen::Vector3d::UnitZ();

  ContactManifold pointManifold;
  pointManifold.addContact(cp);
  pointManifold.setType(ContactType::Point);
  EXPECT_TRUE(pointManifold.isTypeCompatible());
  pointManifold.addContact(cp);
  EXPECT_FALSE(pointManifold.isTypeCompatible());

  ContactManifold edgeManifold;
  edgeManifold.addContact(cp);
  edgeManifold.addContact(cp);
  edgeManifold.setType(ContactType::Edge);
  EXPECT_TRUE(edgeManifold.isTypeCompatible());

  ContactManifold faceManifold;
  faceManifold.addContact(cp);
  faceManifold.addContact(cp);
  faceManifold.addContact(cp);
  faceManifold.setType(ContactType::Face);
  EXPECT_TRUE(faceManifold.isTypeCompatible());

  ContactManifold badFace;
  ContactPoint cp2;
  cp2.normal = Eigen::Vector3d::UnitX();
  badFace.addContact(cp);
  badFace.addContact(cp2);
  badFace.addContact(cp);
  badFace.setType(ContactType::Face);
  EXPECT_FALSE(badFace.isTypeCompatible());

  ContactManifold patchManifold;
  patchManifold.addContact(cp);
  patchManifold.addContact(cp);
  patchManifold.setType(ContactType::Patch);
  EXPECT_TRUE(patchManifold.isTypeCompatible());

  ContactManifold unknownManifold;
  unknownManifold.setType(ContactType::Unknown);
  EXPECT_TRUE(unknownManifold.isTypeCompatible());
}

TEST(CollisionResult, DefaultConstruction)
{
  CollisionResult result;

  EXPECT_FALSE(result.isCollision());
  EXPECT_FALSE(static_cast<bool>(result));
  EXPECT_EQ(result.numContacts(), 0u);
  EXPECT_EQ(result.numManifolds(), 0u);
}

TEST(CollisionResult, AddContact)
{
  CollisionResult result;

  ContactPoint cp;
  cp.position = Eigen::Vector3d(1, 2, 3);
  cp.depth = 0.1;

  result.addContact(cp);

  EXPECT_TRUE(result.isCollision());
  EXPECT_TRUE(static_cast<bool>(result));
  EXPECT_EQ(result.numContacts(), 1u);
  EXPECT_EQ(result.numManifolds(), 1u);
  EXPECT_EQ(result.getContact(0).position, Eigen::Vector3d(1, 2, 3));
}

TEST(CollisionResult, AddManifold)
{
  CollisionResult result;

  ContactManifold manifold;
  manifold.addContact(ContactPoint{});
  manifold.addContact(ContactPoint{});
  manifold.setType(ContactType::Face);

  result.addManifold(std::move(manifold));

  EXPECT_TRUE(result.isCollision());
  EXPECT_EQ(result.numContacts(), 2u);
  EXPECT_EQ(result.numManifolds(), 1u);
  EXPECT_EQ(result.getManifold(0).getType(), ContactType::Face);
}

TEST(CollisionResult, Clear)
{
  CollisionResult result;
  result.addContact(ContactPoint{});
  result.addContact(ContactPoint{});

  EXPECT_EQ(result.numContacts(), 2u);

  result.clear();

  EXPECT_FALSE(result.isCollision());
  EXPECT_EQ(result.numContacts(), 0u);
  EXPECT_EQ(result.numManifolds(), 0u);
}

TEST(CollisionResult, GetManifolds)
{
  CollisionResult result;

  ContactManifold m1;
  m1.setType(ContactType::Point);
  result.addManifold(std::move(m1));

  ContactManifold m2;
  m2.setType(ContactType::Edge);
  result.addManifold(std::move(m2));

  auto manifolds = result.getManifolds();
  EXPECT_EQ(manifolds.size(), 2u);
  EXPECT_EQ(manifolds[0].getType(), ContactType::Point);
  EXPECT_EQ(manifolds[1].getType(), ContactType::Edge);
}

TEST(CollisionResult, GetContactFlat)
{
  CollisionResult result;

  ContactManifold m1;
  ContactPoint cp1;
  cp1.position = Eigen::Vector3d(1, 0, 0);
  m1.addContact(cp1);

  ContactManifold m2;
  ContactPoint cp2;
  cp2.position = Eigen::Vector3d(2, 0, 0);
  ContactPoint cp3;
  cp3.position = Eigen::Vector3d(3, 0, 0);
  m2.addContact(cp2);
  m2.addContact(cp3);

  result.addManifold(std::move(m1));
  result.addManifold(std::move(m2));

  EXPECT_EQ(result.numContacts(), 3u);
  EXPECT_EQ(result.getContact(0).position, Eigen::Vector3d(1, 0, 0));
  EXPECT_EQ(result.getContact(1).position, Eigen::Vector3d(2, 0, 0));
  EXPECT_EQ(result.getContact(2).position, Eigen::Vector3d(3, 0, 0));
}

TEST(CollisionOption, Defaults)
{
  CollisionOption opt;

  EXPECT_TRUE(opt.enableContact);
  EXPECT_EQ(opt.maxNumContacts, 1000u);
}

TEST(CollisionOption, BinaryCheck)
{
  auto opt = CollisionOption::binaryCheck();

  EXPECT_FALSE(opt.enableContact);
  EXPECT_EQ(opt.maxNumContacts, 1u);
}

TEST(CollisionOption, FullContacts)
{
  auto opt = CollisionOption::fullContacts(500);

  EXPECT_TRUE(opt.enableContact);
  EXPECT_EQ(opt.maxNumContacts, 500u);
}

TEST(ContactType, EnumValues)
{
  EXPECT_NE(ContactType::Point, ContactType::Edge);
  EXPECT_NE(ContactType::Edge, ContactType::Face);
  EXPECT_NE(ContactType::Face, ContactType::Patch);
  EXPECT_NE(ContactType::Patch, ContactType::Unknown);
}
