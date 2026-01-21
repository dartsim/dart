// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/HierarchicalIK.hpp>
#include <dart/dynamics/InverseKinematics.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/math/Constants.hpp>
#include <dart/math/optimization/Function.hpp>
#include <dart/math/optimization/GradientDescentSolver.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <vector>

using namespace dart;
using namespace dart::dynamics;

namespace {

struct IkFixture
{
  SkeletonPtr skeleton;
  BodyNode* root{nullptr};
  BodyNode* mid{nullptr};
  BodyNode* end{nullptr};
};

IkFixture makeIkSkeleton()
{
  IkFixture fixture;
  fixture.skeleton = Skeleton::create("ik");

  auto rootPair = fixture.skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  rootPair.first->setAxis(Eigen::Vector3d::UnitZ());
  rootPair.second->setName("root");
  fixture.root = rootPair.second;

  auto midPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  midPair.first->setAxis(Eigen::Vector3d::UnitY());
  midPair.second->setName("mid");

  fixture.mid = midPair.second;
  fixture.end = midPair.second;

  return fixture;
}

class DummyAnalytical final : public InverseKinematics::Analytical
{
public:
  explicit DummyAnalytical(InverseKinematics* ik)
    : Analytical(ik, "DummyAnalytical", Properties())
  {
    const auto dofs = ik->getDofs();
    mDofs.assign(dofs.begin(), dofs.end());
  }

  std::span<const Solution> computeSolutions(
      const Eigen::Isometry3d& desiredTf) override
  {
    (void)desiredTf;
    mSolutions.clear();
    mSolutions.emplace_back(mIK->getPositions(), VALID);
    return mSolutions;
  }

  std::span<const std::size_t> getDofs() const override
  {
    return mDofs;
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIK) const override
  {
    return std::make_unique<DummyAnalytical>(newIK);
  }

private:
  std::vector<std::size_t> mDofs;
};

class PartialAnalytical final : public InverseKinematics::Analytical
{
public:
  explicit PartialAnalytical(InverseKinematics* ik)
    : Analytical(ik, "PartialAnalytical", Properties())
  {
    const auto dofs = ik->getDofs();
    if (!dofs.empty())
      mDofs.push_back(dofs.front());
  }

  std::span<const Solution> computeSolutions(
      const Eigen::Isometry3d& desiredTf) override
  {
    (void)desiredTf;
    mSolutions.clear();
    Eigen::VectorXd config
        = Eigen::VectorXd::Zero(static_cast<int>(mDofs.size()));
    if (!mDofs.empty()) {
      config[0] = mIK->getNode()->getSkeleton()->getDof(mDofs[0])->getPosition();
    }
    mSolutions.emplace_back(config, VALID);
    return mSolutions;
  }

  std::span<const std::size_t> getDofs() const override
  {
    return mDofs;
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIK) const override
  {
    return std::make_unique<PartialAnalytical>(newIK);
  }

private:
  std::vector<std::size_t> mDofs;
};

} // namespace

TEST(InverseKinematics, ConfigurationAndClone)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  ik->setInactive();
  EXPECT_FALSE(ik->isActive());

  ik->setActive(true);
  EXPECT_TRUE(ik->isActive());

  ik->setHierarchyLevel(2);
  EXPECT_EQ(ik->getHierarchyLevel(), 2u);

  ik->useChain();
  ik->useWholeBody();

  ik->setOffset(Eigen::Vector3d(0.1, 0.2, 0.3));
  EXPECT_TRUE(ik->hasOffset());

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target");
  target->setTransform(fixture.end->getTransform());
  ik->setTarget(target);
  EXPECT_EQ(ik->getTarget(), target);

  ik->setErrorMethod<InverseKinematics::TaskSpaceRegion>();
  auto& error = ik->getErrorMethod();
  error.setBounds();
  error.setAngularBounds();
  error.setLinearBounds();
  error.setErrorLengthClamp(0.5);
  error.setErrorWeights(Eigen::Vector6d::Ones());
  error.setAngularErrorWeights(Eigen::Vector3d::Constant(0.5));
  error.setLinearErrorWeights(Eigen::Vector3d::Constant(1.5));
  EXPECT_EQ(error.getMethodName().empty(), false);

  ik->setGradientMethod<InverseKinematics::JacobianTranspose>();
  auto& gradient = ik->getGradientMethod();
  gradient.setComponentWiseClamp(0.25);
  gradient.setComponentWeights(
      Eigen::VectorXd::Constant(static_cast<int>(ik->getDofs().size()), 1.0));
  EXPECT_EQ(gradient.getMethodName().empty(), false);

  auto clone = ik->clone(fixture.root);
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getNode(), fixture.root);
}

TEST(InverseKinematics, FindSolutionPaths)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  ik->setSolver(nullptr);
  Eigen::VectorXd solution;
  EXPECT_FALSE(ik->findSolution(solution));

  auto solver = std::make_shared<math::GradientDescentSolver>();
  solver->setNumMaxIterations(5);
  solver->setTolerance(1e-6);
  ik->setSolver(solver);
  ik->resetProblem();

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target2");
  target->setTransform(fixture.end->getTransform());
  ik->setTarget(target);

  solution.resize(0);
  ik->findSolution(solution);
  EXPECT_EQ(solution.size(), static_cast<int>(ik->getDofs().size()));
}

TEST(InverseKinematics, AnalyticalGradientAndWeights)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target3");
  target->setTransform(fixture.end->getTransform());
  ik->setTarget(target);

  auto& analytical = ik->setGradientMethod<DummyAnalytical>();
  ASSERT_NE(ik->getAnalytical(), nullptr);

  auto* analyticalPtr = ik->getAnalytical();
  analyticalPtr->setExtraDofUtilization(
      InverseKinematics::Analytical::PRE_ANALYTICAL);
  analyticalPtr->setExtraErrorLengthClamp(0.2);
  analyticalPtr->setQualityComparisonFunction(
      [](const Eigen::VectorXd&,
         const Eigen::VectorXd&,
         const InverseKinematics*) { return true; });

  const auto solutions = analyticalPtr->getSolutions();
  ASSERT_EQ(solutions.size(), 1u);
  EXPECT_EQ(
      solutions[0].mConfig.size(),
      static_cast<int>(ik->getDofs().size()));

  Eigen::VectorXd q = ik->getPositions();
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(q.size());
  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), grad.size());
  analytical.evalGradient(q, gradMap);
  EXPECT_EQ(grad.size(), q.size());

  analyticalPtr->resetQualityComparisonFunction();
}

TEST(InverseKinematics, JacobianDlsGradientAndDofs)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto& gradient = ik->setGradientMethod<InverseKinematics::JacobianDLS>();
  gradient.setDampingCoefficient(0.05);
  EXPECT_NEAR(gradient.getDampingCoefficient(), 0.05, 1e-12);

  const auto dofs = ik->getDofs();
  ASSERT_GT(dofs.size(), 0u);
  ik->setDofs(std::span<const std::size_t>(dofs.data(), 1u));
  EXPECT_EQ(ik->getDofs().size(), 1u);

  Eigen::VectorXd q = ik->getPositions();
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(q.size());
  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), grad.size());
  gradient.evalGradient(q, gradMap);
  EXPECT_EQ(grad.size(), q.size());
}

TEST(InverseKinematics, TaskSpaceRegionAccessorsAndError)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto& tsr = ik->setErrorMethod<InverseKinematics::TaskSpaceRegion>();
  InverseKinematics::ErrorMethod::Bounds bounds;
  bounds.first = Eigen::Vector6d::Constant(-0.05);
  bounds.second = Eigen::Vector6d::Constant(0.05);
  bounds.second[0] = std::numeric_limits<double>::infinity();
  bounds.first[1] = -std::numeric_limits<double>::infinity();
  tsr.setBounds(bounds);
  tsr.setComputeFromCenter(true);
  tsr.setErrorLengthClamp(0.01);
  tsr.setErrorWeights(Eigen::Vector6d::Ones());
  tsr.setAngularErrorWeights(Eigen::Vector3d::Constant(0.5));
  tsr.setLinearErrorWeights(Eigen::Vector3d::Constant(1.5));

  auto reference = std::make_shared<SimpleFrame>(Frame::World(), "ik_ref");
  Eigen::Isometry3d refTf = Eigen::Isometry3d::Identity();
  refTf.linear()
      = Eigen::AngleAxisd(0.25 * math::pi, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  reference->setTransform(refTf);
  tsr.setReferenceFrame(reference);
  EXPECT_EQ(tsr.getReferenceFrame(), reference);
  EXPECT_TRUE(tsr.isComputingFromCenter());

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target");
  Eigen::Isometry3d targetTf = fixture.end->getTransform();
  targetTf.translation() += Eigen::Vector3d(0.2, -0.1, 0.15);
  targetTf.linear()
      = (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()))
            .toRotationMatrix();
  target->setTransform(targetTf);
  ik->setTarget(target);

  ik->setOffset(Eigen::Vector3d(0.05, 0.0, 0.0));

  const Eigen::VectorXd q = ik->getPositions();
  const auto& error = tsr.evalError(q);
  EXPECT_TRUE(error.array().isFinite().all());

  const auto desired = tsr.computeDesiredTransform(
      fixture.end->getTransform(), error);
  EXPECT_TRUE(desired.matrix().array().isFinite().all());

  const auto& boundsOut = tsr.getBounds();
  EXPECT_EQ(boundsOut.first.size(), 6);
  EXPECT_EQ(boundsOut.second.size(), 6);
  EXPECT_EQ(tsr.getAngularBounds().first.size(), 3);
  EXPECT_EQ(tsr.getLinearBounds().first.size(), 3);
  EXPECT_DOUBLE_EQ(tsr.getErrorLengthClamp(), 0.01);
  Eigen::Vector6d expectedWeights = Eigen::Vector6d::Zero();
  expectedWeights.head<3>().setConstant(0.5);
  expectedWeights.tail<3>().setConstant(1.5);
  EXPECT_TRUE(tsr.getErrorWeights().isApprox(expectedWeights));
  EXPECT_TRUE(tsr.getAngularErrorWeights().isApprox(
      Eigen::Vector3d::Constant(0.5)));
  EXPECT_TRUE(tsr.getLinearErrorWeights().isApprox(
      Eigen::Vector3d::Constant(1.5)));
}

TEST(InverseKinematics, AnalyticalExtraDofsAndGradientCaching)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target_extra");
  Eigen::Isometry3d targetTf = fixture.end->getTransform();
  targetTf.translation() += Eigen::Vector3d(0.1, 0.0, 0.0);
  target->setTransform(targetTf);
  ik->setTarget(target);

  auto& analytical = ik->setGradientMethod<PartialAnalytical>();
  analytical.setExtraDofUtilization(InverseKinematics::Analytical::PRE_ANALYTICAL);
  analytical.setExtraErrorLengthClamp(0.2);
  analytical.setComponentWiseClamp(0.05);
  analytical.setComponentWeights(Eigen::VectorXd::Constant(
      static_cast<int>(ik->getDofs().size()), 1.0));

  Eigen::VectorXd q = ik->getPositions();
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(q.size());
  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), grad.size());
  analytical.evalGradient(q, gradMap);
  EXPECT_EQ(grad.size(), q.size());

  const Eigen::VectorXd firstGrad = grad;
  grad.setZero();
  analytical.evalGradient(q, gradMap);
  EXPECT_TRUE(grad.isApprox(firstGrad));

  EXPECT_EQ(analytical.getIK(), ik.get());
  EXPECT_EQ(
      analytical.getExtraErrorLengthClamp(), 0.2);
  EXPECT_EQ(
      analytical.getExtraDofUtilization(),
      InverseKinematics::Analytical::PRE_ANALYTICAL);
  EXPECT_DOUBLE_EQ(
      analytical.getComponentWiseClamp(), 0.05);
  EXPECT_EQ(
      analytical.getComponentWeights().size(),
      static_cast<int>(ik->getDofs().size()));
}

TEST(InverseKinematics, ObjectiveAndConstraintEvaluation)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto objective = std::make_shared<math::ModularFunction>("objective");
  objective->setCostFunction(
      [](const Eigen::VectorXd& x) { return x.squaredNorm(); });
  objective->setGradientFunction(
      [](const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) {
        grad = 2.0 * x;
      });

  auto nullspace = std::make_shared<math::ModularFunction>("nullspace");
  nullspace->setCostFunction(
      [](const Eigen::VectorXd& x) { return x.sum(); });
  nullspace->setGradientFunction(
      [](const Eigen::VectorXd&, Eigen::Map<Eigen::VectorXd> grad) {
        grad.setOnes();
      });

  ik->setObjective(objective);
  ik->setNullSpaceObjective(nullspace);
  EXPECT_TRUE(ik->hasNullSpaceObjective());

  auto problem = ik->getProblem();
  ASSERT_NE(problem, nullptr);
  auto objectiveFunc = problem->getObjective();
  ASSERT_NE(objectiveFunc, nullptr);

  Eigen::VectorXd q = ik->getPositions();
  q.setConstant(0.2);

  const double cost = objectiveFunc->eval(q);
  EXPECT_GT(cost, 0.0);

  Eigen::VectorXd grad = Eigen::VectorXd::Zero(q.size());
  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), grad.size());
  objectiveFunc->evalGradient(q, gradMap);
  EXPECT_TRUE(grad.array().isFinite().all());

  auto constraint = problem->getEqConstraint(0u);
  ASSERT_NE(constraint, nullptr);
  EXPECT_GE(constraint->eval(q), 0.0);

  grad.setZero();
  constraint->evalGradient(q, gradMap);
  EXPECT_TRUE(grad.array().isFinite().all());

  const Eigen::VectorXd before = ik->getPositions();
  Eigen::VectorXd wrongSize(1);
  wrongSize[0] = 0.0;
  ik->setPositions(wrongSize);
  EXPECT_TRUE(ik->getPositions().isApprox(before));

  EXPECT_EQ(ik->getDofMap().size(), ik->getDofs().size());
  ik->clearCaches();
}

TEST(InverseKinematics, ResetProblemAndSolveAndApply)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto problem = ik->getProblem();
  ASSERT_NE(problem, nullptr);

  const auto dofs = ik->getDofs();
  Eigen::VectorXd seed = Eigen::VectorXd::Zero(static_cast<int>(dofs.size()));
  problem->addSeed(seed);
  EXPECT_EQ(problem->getSeeds().size(), 1u);

  ik->resetProblem(false);
  EXPECT_EQ(problem->getSeeds().size(), 1u);

  ik->resetProblem(true);
  EXPECT_EQ(problem->getSeeds().size(), 0u);

  ik->setTarget(nullptr);
  ASSERT_NE(ik->getTarget(), nullptr);

  ik->setOffset(Eigen::Vector3d(0.1, 0.0, 0.0));
  const auto jac = ik->computeJacobian();
  EXPECT_EQ(jac.rows(), 6);
  EXPECT_EQ(jac.cols(), static_cast<int>(dofs.size()));

  ik->setOffset(Eigen::Vector3d::Zero());
  auto solver = std::make_shared<math::GradientDescentSolver>();
  solver->setNumMaxIterations(5);
  solver->setTolerance(1e-6);
  ik->setSolver(solver);

  EXPECT_TRUE(ik->solveAndApply(false));

  Eigen::VectorXd positions;
  EXPECT_TRUE(ik->solveAndApply(positions, false));
  EXPECT_EQ(positions.size(), static_cast<int>(dofs.size()));
}

TEST(HierarchicalIK, CompositeAndWholeBody)
{
  auto fixture = makeIkSkeleton();
  auto ikRoot = fixture.root->getOrCreateIK();
  auto ikEnd = fixture.end->getOrCreateIK();
  ASSERT_NE(ikRoot, nullptr);
  ASSERT_NE(ikEnd, nullptr);

  ikRoot->setHierarchyLevel(0);
  ikEnd->setHierarchyLevel(1);

  auto composite = CompositeIK::create(fixture.skeleton);
  ASSERT_NE(composite, nullptr);
  EXPECT_TRUE(composite->addModule(ikRoot));
  EXPECT_TRUE(composite->addModule(ikEnd));
  composite->refreshIKHierarchy();

  Eigen::VectorXd positions;
  composite->findSolution(positions);
  EXPECT_EQ(positions.size(), fixture.skeleton->getNumDofs());

  const auto nullspaces = composite->computeNullSpaces();
  EXPECT_EQ(nullspaces.size(), composite->getIKHierarchy().size());

  auto clonedSkel = fixture.skeleton->cloneSkeleton();
  auto cloned = composite->clone(clonedSkel);
  ASSERT_NE(cloned, nullptr);
  EXPECT_EQ(cloned->getSkeleton()->getNumDofs(), clonedSkel->getNumDofs());

  auto wholeBody = WholeBodyIK::create(fixture.skeleton);
  ASSERT_NE(wholeBody, nullptr);
  wholeBody->refreshIKHierarchy();
  Eigen::VectorXd wbPositions;
  wholeBody->solveAndApply(wbPositions, false);
  EXPECT_EQ(wbPositions.size(), fixture.skeleton->getNumDofs());
}
