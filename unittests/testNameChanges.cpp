
#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"
#include "dart/dynamics/SoftBodyNode.h"

using namespace dart;
using namespace math;
using namespace dynamics;


TEST(NAMECHANGES, BASIC)
{
  //--------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------
  // Bodies
  BodyNode* body1 = new BodyNode;
  BodyNode* body2 = new BodyNode;
  BodyNode* body3 = new BodyNode;

  // TODO: Add some SoftBodyNodes into this test.
  // Grey is not familiar with construction of SoftBodyNodes,
  // so he is leaving that for someone else for the time being.

  // Joints
  RevoluteJoint* joint1 = new RevoluteJoint;
  RevoluteJoint* joint2 = new RevoluteJoint;
  RevoluteJoint* joint3 = new RevoluteJoint;

  // Skeleton
  Skeleton* skel = new Skeleton;

  body1->setParentJoint(joint1);
  body2->setParentJoint(joint2);
  body3->setParentJoint(joint3);

  body1->addChildBodyNode(body2);
  body2->addChildBodyNode(body3);

  skel->addBodyNode(body1);
  skel->addBodyNode(body2);
  skel->addBodyNode(body3);

  skel->init();

  EXPECT_FALSE(body1->getName() == body2->getName());
  EXPECT_FALSE(body2->getName() == body3->getName());
  EXPECT_FALSE(body3->getName() == body1->getName());


  EXPECT_FALSE(joint1->getName() == joint2->getName());
  EXPECT_FALSE(joint2->getName() == joint3->getName());
  EXPECT_FALSE(joint3->getName() == joint1->getName());

  std::string newname1 = body1->setName("same_name");
  std::string newname2 = body2->setName("same_name");
  std::string newname3 = body3->setName("same_name");

  EXPECT_FALSE(body1->getName() == body2->getName());
  EXPECT_FALSE(body2->getName() == body3->getName());
  EXPECT_FALSE(body3->getName() == body1->getName());

  EXPECT_TRUE(body1->getName() == newname1);
  EXPECT_TRUE(body2->getName() == newname2);
  EXPECT_TRUE(body3->getName() == newname3);

  EXPECT_TRUE(skel->getBodyNode(newname1) == body1);
  EXPECT_TRUE(skel->getBodyNode(newname2) == body2);
  EXPECT_TRUE(skel->getBodyNode(newname3) == body3);

  newname1 = joint1->setName("another_name");
  newname2 = joint2->setName("another_name");
  newname3 = joint3->setName("another_name");

  EXPECT_FALSE(joint1->getName() == joint2->getName());
  EXPECT_FALSE(joint2->getName() == joint3->getName());
  EXPECT_FALSE(joint3->getName() == joint1->getName());

  EXPECT_TRUE(joint1->getName() == newname1);
  EXPECT_TRUE(joint2->getName() == newname2);
  EXPECT_TRUE(joint3->getName() == newname3);

  EXPECT_TRUE(skel->getJoint(newname1) == joint1);
  EXPECT_TRUE(skel->getJoint(newname2) == joint2);
  EXPECT_TRUE(skel->getJoint(newname3) == joint3);

  std::string unique_name = body2->setName("a_unique_name");
  EXPECT_TRUE(body2->getName() == "a_unique_name");
  EXPECT_TRUE(skel->getBodyNode("a_unique_name") == body2);

  EXPECT_FALSE(body1->getName() == body2->getName());
  EXPECT_FALSE(body2->getName() == body3->getName());
  EXPECT_FALSE(body3->getName() == body1->getName());

  unique_name = joint3->setName("a_unique_name");
  EXPECT_TRUE(joint3->getName() == "a_unique_name");
  EXPECT_TRUE(skel->getJoint("a_unique_name") == joint3);

  EXPECT_FALSE(joint1->getName() == joint2->getName());
  EXPECT_FALSE(joint2->getName() == joint3->getName());
  EXPECT_FALSE(joint3->getName() == joint1->getName());

  EXPECT_TRUE(skel->getBodyNode("nonexistent_name") == NULL);
  EXPECT_TRUE(skel->getJoint("nonexistent_name") == NULL);
  EXPECT_TRUE(skel->getSoftBodyNode("nonexistent_name") == NULL);

  Joint* oldJoint = body3->getParentJoint();
  std::string oldJointName = oldJoint->getName();
  Joint* newJoint = new RevoluteJoint(Eigen::Vector3d(1,0,0), "a_new_joint");
  body3->setParentJoint(newJoint);
  EXPECT_TRUE(skel->getJoint("a_new_joint") == newJoint);
  EXPECT_FALSE(skel->getJoint(oldJointName) == oldJoint);
}

TEST(NAMECHANGES, SETPATTERN)
{
  dart::common::NameManager<BodyNode> test_mgr;

  BodyNode* bn0 = new BodyNode("name");
  BodyNode* bn1 = new BodyNode("name");
  BodyNode* bn2 = new BodyNode("name");

  test_mgr.setPattern("%s(%d)");

  test_mgr.issueNewNameAndAdd(bn0->getName(), bn0);
  test_mgr.issueNewNameAndAdd(bn1->getName(), bn1);
  test_mgr.issueNewNameAndAdd(bn2->getName(), bn2);

  EXPECT_TRUE( test_mgr.getObject("name") == bn0);
  EXPECT_TRUE( test_mgr.getObject("name(1)") == bn1);
  EXPECT_TRUE( test_mgr.getObject("name(2)") == bn2);

  test_mgr.clear();

  bn0->setName("bodynode");
  bn1->setName("bodynode");
  bn2->setName("bodynode");

  test_mgr.setPattern("(%d)-%s");
  test_mgr.issueNewNameAndAdd(bn0->getName(), bn0);
  test_mgr.issueNewNameAndAdd(bn1->getName(), bn1);
  test_mgr.issueNewNameAndAdd(bn2->getName(), bn2);

  EXPECT_TRUE( test_mgr.getObject("bodynode") == bn0);
  EXPECT_TRUE( test_mgr.getObject("(1)-bodynode") == bn1 );
  EXPECT_TRUE( test_mgr.getObject("(2)-bodynode") == bn2 );

  delete bn0;
  delete bn1;
  delete bn2;

}


int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
