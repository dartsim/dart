// Copyright (c) 2011, The DART development contributors

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/hierarchical_ik.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/constants.hpp>
#include <dart/math/optimization/function.hpp>
#include <dart/math/optimization/gradient_descent_solver.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <span>
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
    if (!dofs.empty()) {
      mDofs.push_back(dofs.front());
    }
  }

  std::span<const Solution> computeSolutions(
      const Eigen::Isometry3d& desiredTf) override
  {
    (void)desiredTf;
    mSolutions.clear();
    Eigen::VectorXd config
        = Eigen::VectorXd::Zero(static_cast<int>(mDofs.size()));
    if (!mDofs.empty()) {
      config[0]
          = mIK->getNode()->getSkeleton()->getDof(mDofs[0])->getPosition();
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
      solutions[0].mConfig.size(), static_cast<int>(ik->getDofs().size()));

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
  refTf.linear() = Eigen::AngleAxisd(0.25 * math::pi, Eigen::Vector3d::UnitZ())
                       .toRotationMatrix();
  reference->setTransform(refTf);
  tsr.setReferenceFrame(reference);
  EXPECT_EQ(tsr.getReferenceFrame(), reference);
  EXPECT_TRUE(tsr.isComputingFromCenter());

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target");
  Eigen::Isometry3d targetTf = fixture.end->getTransform();
  targetTf.translation() += Eigen::Vector3d(0.2, -0.1, 0.15);
  targetTf.linear() = (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX())
                       * Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()))
                          .toRotationMatrix();
  target->setTransform(targetTf);
  ik->setTarget(target);

  ik->setOffset(Eigen::Vector3d(0.05, 0.0, 0.0));

  const Eigen::VectorXd q = ik->getPositions();
  const auto& error = tsr.evalError(q);
  EXPECT_TRUE(error.array().isFinite().all());

  const auto desired
      = tsr.computeDesiredTransform(fixture.end->getTransform(), error);
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
  EXPECT_TRUE(
      tsr.getAngularErrorWeights().isApprox(Eigen::Vector3d::Constant(0.5)));
  EXPECT_TRUE(
      tsr.getLinearErrorWeights().isApprox(Eigen::Vector3d::Constant(1.5)));
}

TEST(InverseKinematics, AnalyticalExtraDofsAndGradientCaching)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto target
      = std::make_shared<SimpleFrame>(Frame::World(), "ik_target_extra");
  Eigen::Isometry3d targetTf = fixture.end->getTransform();
  targetTf.translation() += Eigen::Vector3d(0.1, 0.0, 0.0);
  target->setTransform(targetTf);
  ik->setTarget(target);

  auto& analytical = ik->setGradientMethod<PartialAnalytical>();
  analytical.setExtraDofUtilization(
      InverseKinematics::Analytical::PRE_ANALYTICAL);
  analytical.setExtraErrorLengthClamp(0.2);
  analytical.setComponentWiseClamp(0.05);
  analytical.setComponentWeights(
      Eigen::VectorXd::Constant(static_cast<int>(ik->getDofs().size()), 1.0));

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
  EXPECT_EQ(analytical.getExtraErrorLengthClamp(), 0.2);
  EXPECT_EQ(
      analytical.getExtraDofUtilization(),
      InverseKinematics::Analytical::PRE_ANALYTICAL);
  EXPECT_DOUBLE_EQ(analytical.getComponentWiseClamp(), 0.05);
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
  nullspace->setCostFunction([](const Eigen::VectorXd& x) { return x.sum(); });
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

namespace {

struct IkChain
{
  SkeletonPtr skeleton;
  BodyNode* root{nullptr};
  BodyNode* mid{nullptr};
  BodyNode* tip{nullptr};
  EndEffector* endEffector{nullptr};
};

IkChain makeIkChain()
{
  IkChain chain;
  chain.skeleton = Skeleton::create("ik_chain");

  auto rootPair = chain.skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  rootPair.first->setAxis(Eigen::Vector3d::UnitZ());
  rootPair.second->setName("root");
  chain.root = rootPair.second;

  auto midPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  midPair.first->setAxis(Eigen::Vector3d::UnitY());
  midPair.second->setName("mid");
  chain.mid = midPair.second;

  auto tipPair
      = midPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  tipPair.first->setAxis(Eigen::Vector3d::UnitX());
  tipPair.second->setName("tip");
  chain.tip = tipPair.second;

  chain.endEffector = chain.tip->createEndEffector("ee");
  Eigen::Isometry3d eeTf = Eigen::Isometry3d::Identity();
  eeTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.2);
  chain.endEffector->setDefaultRelativeTransform(eeTf, true);

  return chain;
}

class SimpleIkObjective final : public InverseKinematics::Function,
                                public math::Function
{
public:
  explicit SimpleIkObjective(InverseKinematics* ik) : mIk(ik)
  {
    (void)mIk;
  }

  math::FunctionPtr clone(InverseKinematics* newIk) const override
  {
    return std::make_shared<SimpleIkObjective>(newIk);
  }

  double eval(const Eigen::VectorXd& x) override
  {
    return x.squaredNorm();
  }

  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    grad = 2.0 * x;
  }

private:
  InverseKinematics* mIk;
};

class InequalityConstraintFunction final : public InverseKinematics::Function,
                                           public math::Function
{
public:
  explicit InequalityConstraintFunction(InverseKinematics* ik) : mIk(ik) {}

  math::FunctionPtr clone(InverseKinematics* newIk) const override
  {
    return std::make_shared<InequalityConstraintFunction>(newIk);
  }

  double eval(const Eigen::VectorXd& x) override
  {
    return x.sum();
  }

  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    (void)x;
    grad.setOnes();
  }

  const InverseKinematics* getIk() const
  {
    return mIk;
  }

private:
  InverseKinematics* mIk;
};

class SimpleHierarchicalObjective final : public HierarchicalIK::Function,
                                          public math::Function
{
public:
  explicit SimpleHierarchicalObjective(
      const std::shared_ptr<HierarchicalIK>& ik)
    : mIk(ik)
  {
  }

  math::FunctionPtr clone(
      const std::shared_ptr<HierarchicalIK>& newIk) const override
  {
    return std::make_shared<SimpleHierarchicalObjective>(newIk);
  }

  double eval(const Eigen::VectorXd& x) override
  {
    return x.array().abs().sum();
  }

  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    grad = x.array().sign();
  }

private:
  std::weak_ptr<HierarchicalIK> mIk;
};

class CoverageAnalytical final : public InverseKinematics::Analytical
{
public:
  explicit CoverageAnalytical(InverseKinematics* ik)
    : Analytical(ik, "CoverageAnalytical", Properties())
  {
    const auto dofs = ik->getDofs();
    if (!dofs.empty()) {
      mDofs.push_back(dofs.front());
    }
    if (dofs.size() > 1) {
      mDofs.push_back(dofs[1]);
    }
  }

  std::span<const Solution> computeSolutions(
      const Eigen::Isometry3d& desiredTf) override
  {
    (void)desiredTf;
    mSolutions.clear();

    Eigen::VectorXd current = getPositions();
    mSolutions.emplace_back(current, VALID);

    Eigen::VectorXd outOfReach = current;
    outOfReach.array() += 0.25;
    mSolutions.emplace_back(outOfReach, OUT_OF_REACH);

    Eigen::VectorXd limitViolation = current;
    limitViolation.array() += 10.0;
    mSolutions.emplace_back(limitViolation, VALID);

    checkSolutionJointLimits();

    return mSolutions;
  }

  std::span<const std::size_t> getDofs() const override
  {
    return mDofs;
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIk) const override
  {
    return std::make_unique<CoverageAnalytical>(newIk);
  }

private:
  std::vector<std::size_t> mDofs;
};

class NoSolutionAnalytical final : public InverseKinematics::Analytical
{
public:
  explicit NoSolutionAnalytical(InverseKinematics* ik)
    : Analytical(ik, "NoSolutionAnalytical", Properties())
  {
    const auto dofs = ik->getDofs();
    mDofs.assign(dofs.begin(), dofs.end());
  }

  std::span<const Solution> computeSolutions(
      const Eigen::Isometry3d& desiredTf) override
  {
    (void)desiredTf;
    mSolutions.clear();
    return mSolutions;
  }

  std::span<const std::size_t> getDofs() const override
  {
    return mDofs;
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* newIk) const override
  {
    return std::make_unique<NoSolutionAnalytical>(newIk);
  }

private:
  std::vector<std::size_t> mDofs;
};

} // namespace

TEST(InverseKinematics, AccessorsAndConfiguration)
{
  auto chain = makeIkChain();
  auto ik = InverseKinematics::create(chain.endEffector);
  ASSERT_NE(ik, nullptr);

  ik->setInactive();
  EXPECT_FALSE(ik->isActive());
  ik->setActive(true);
  EXPECT_TRUE(ik->isActive());

  ik->setHierarchyLevel(2);
  EXPECT_EQ(ik->getHierarchyLevel(), 2u);

  ik->useChain();
  const auto chainDofs = ik->getDofs();
  EXPECT_FALSE(chainDofs.empty());

  ik->useWholeBody();
  const auto wholeBodyDofs = ik->getDofs();
  EXPECT_GE(wholeBodyDofs.size(), chainDofs.size());

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target");
  target->setTransform(chain.endEffector->getWorldTransform());
  ik->setTarget(target);
  EXPECT_EQ(ik->getTarget(), target);

  ik->setTarget(nullptr);
  EXPECT_NE(ik->getTarget(), nullptr);

  EXPECT_FALSE(ik->hasOffset());
  ik->setOffset(Eigen::Vector3d(0.1, -0.2, 0.3));
  EXPECT_TRUE(ik->hasOffset());
  EXPECT_TRUE(ik->getOffset().isApprox(Eigen::Vector3d(0.1, -0.2, 0.3)));

  const auto jacobianWithOffset = ik->computeJacobian();
  EXPECT_EQ(jacobianWithOffset.rows(), 6);

  ik->setOffset(Eigen::Vector3d::Zero());
  EXPECT_FALSE(ik->hasOffset());

  std::vector<std::size_t> subsetDofs;
  subsetDofs.push_back(wholeBodyDofs.front());
  if (wholeBodyDofs.size() > 1) {
    subsetDofs.push_back(wholeBodyDofs[1]);
  }
  ik->setDofs(
      std::span<const std::size_t>(subsetDofs.data(), subsetDofs.size()));
  EXPECT_EQ(ik->getDofs().size(), subsetDofs.size());

  auto& tsr = ik->setErrorMethod<InverseKinematics::TaskSpaceRegion>();
  tsr.setComputeFromCenter(false);
  tsr.setBounds(
      Eigen::Vector6d::Constant(-0.05), Eigen::Vector6d::Constant(0.05));
  tsr.setAngularBounds(
      Eigen::Vector3d::Constant(-0.01), Eigen::Vector3d::Constant(0.01));
  tsr.setLinearBounds(
      Eigen::Vector3d::Constant(-0.02), Eigen::Vector3d::Constant(0.02));
  tsr.setErrorLengthClamp(0.5);
  tsr.setAngularErrorWeights(Eigen::Vector3d::Constant(0.4));
  tsr.setLinearErrorWeights(Eigen::Vector3d::Constant(1.2));

  auto reference = std::make_shared<SimpleFrame>(Frame::World(), "ik_ref");
  reference->setTransform(Eigen::Isometry3d::Identity());
  tsr.setReferenceFrame(reference);
  EXPECT_EQ(tsr.getReferenceFrame(), reference);

  Eigen::VectorXd q = ik->getPositions();
  const auto& error = tsr.evalError(q);
  EXPECT_TRUE(error.array().isFinite().all());

  const auto desiredTf = tsr.computeDesiredTransform(
      chain.endEffector->getWorldTransform(), error);
  EXPECT_TRUE(desiredTf.matrix().array().isFinite().all());

  auto& transpose
      = ik->setGradientMethod<InverseKinematics::JacobianTranspose>();
  transpose.setComponentWiseClamp(0.1);
  transpose.setComponentWeights(
      Eigen::VectorXd::Constant(static_cast<int>(ik->getDofs().size()), 1.0));

  Eigen::VectorXd grad = Eigen::VectorXd::Zero(q.size());
  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), grad.size());
  transpose.evalGradient(q, gradMap);
  EXPECT_EQ(grad.size(), q.size());

  auto& dls = ik->setGradientMethod<InverseKinematics::JacobianDLS>();
  dls.setDampingCoefficient(0.08);
  EXPECT_NEAR(dls.getDampingCoefficient(), 0.08, 1e-12);

  auto solver = std::make_shared<math::GradientDescentSolver>();
  solver->setNumMaxIterations(2);
  solver->setTolerance(1e-6);
  ik->setSolver(solver);
  EXPECT_EQ(ik->getSolver(), solver);

  Eigen::VectorXd solution;
  EXPECT_TRUE(ik->findSolution(solution) || solution.size() > 0);
  EXPECT_EQ(solution.size(), static_cast<int>(ik->getDofs().size()));
}

TEST(InverseKinematics, AnalyticalConfigurationAndClone)
{
  auto chain = makeIkChain();
  auto ik = InverseKinematics::create(chain.endEffector);
  ASSERT_NE(ik, nullptr);

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target2");
  target->setTransform(chain.endEffector->getWorldTransform());
  ik->setTarget(target);

  auto& analytical = ik->setGradientMethod<CoverageAnalytical>();
  analytical.setExtraDofUtilization(
      InverseKinematics::Analytical::PRE_AND_POST_ANALYTICAL);
  analytical.setExtraErrorLengthClamp(0.2);
  analytical.setQualityComparisonFunction(
      [](const Eigen::VectorXd&,
         const Eigen::VectorXd&,
         const InverseKinematics*) { return true; });

  const auto solutions = analytical.getSolutions();
  ASSERT_GE(solutions.size(), 1u);

  Eigen::VectorXd q = ik->getPositions();
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(q.size());
  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), grad.size());
  analytical.evalGradient(q, gradMap);
  EXPECT_EQ(grad.size(), q.size());

  analytical.resetQualityComparisonFunction();
  EXPECT_EQ(
      analytical.getExtraDofUtilization(),
      InverseKinematics::Analytical::PRE_AND_POST_ANALYTICAL);
  EXPECT_NEAR(analytical.getExtraErrorLengthClamp(), 0.2, 1e-12);

  ik->setObjective(std::make_shared<SimpleIkObjective>(ik.get()));

  auto clonedSkel = chain.skeleton->cloneSkeleton();
  auto* clonedEnd = clonedSkel->getEndEffector("ee");
  ASSERT_NE(clonedEnd, nullptr);

  auto clonedIk = ik->clone(clonedEnd);
  ASSERT_NE(clonedIk, nullptr);
  EXPECT_EQ(clonedIk->getNode()->getName(), chain.endEffector->getName());
  EXPECT_EQ(clonedIk->getDofs().size(), ik->getDofs().size());
  EXPECT_EQ(
      clonedIk->getErrorMethod().getMethodName(),
      ik->getErrorMethod().getMethodName());
}

TEST(InverseKinematics, AnalyticalGetSolutionsSorting)
{
  auto chain = makeIkChain();
  auto ik = InverseKinematics::create(chain.endEffector);
  ASSERT_NE(ik, nullptr);

  auto target
      = std::make_shared<SimpleFrame>(Frame::World(), "ik_target_solutions");
  target->setTransform(chain.endEffector->getWorldTransform());
  ik->setTarget(target);

  auto& analytical = ik->setGradientMethod<CoverageAnalytical>();
  const auto solutions = analytical.getSolutions();
  EXPECT_GE(solutions.size(), 1u);

  const auto solutionsTf = analytical.getSolutions(target->getTransform());
  EXPECT_EQ(solutionsTf.size(), solutions.size());
}

TEST(InverseKinematics, CloneCopiesInequalityConstraints)
{
  auto chain = makeIkChain();
  auto ik = InverseKinematics::create(chain.endEffector);
  ASSERT_NE(ik, nullptr);

  auto ineq = std::make_shared<InequalityConstraintFunction>(ik.get());
  ik->getProblem()->addIneqConstraint(ineq);
  ASSERT_EQ(ik->getProblem()->getNumIneqConstraints(), 1u);

  auto clonedSkel = chain.skeleton->cloneSkeleton();
  auto* clonedEnd = clonedSkel->getEndEffector("ee");
  ASSERT_NE(clonedEnd, nullptr);

  auto clonedIk = ik->clone(clonedEnd);
  ASSERT_NE(clonedIk, nullptr);
  ASSERT_EQ(clonedIk->getProblem()->getNumIneqConstraints(), 1u);

  auto clonedConstraint
      = std::dynamic_pointer_cast<const InequalityConstraintFunction>(
          clonedIk->getProblem()->getIneqConstraint(0u));
  ASSERT_NE(clonedConstraint, nullptr);
  EXPECT_NE(clonedConstraint.get(), ineq.get());
  EXPECT_EQ(clonedConstraint->getIk(), clonedIk.get());
}

TEST(InverseKinematics, AnalyticalGradientHandlesNoSolutions)
{
  auto chain = makeIkChain();
  auto ik = InverseKinematics::create(chain.endEffector);
  ASSERT_NE(ik, nullptr);

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target_none");
  Eigen::Isometry3d targetTf = chain.endEffector->getWorldTransform();
  targetTf.translation() += Eigen::Vector3d(0.1, -0.05, 0.02);
  target->setTransform(targetTf);
  ik->setTarget(target);

  auto& analytical = ik->setGradientMethod<NoSolutionAnalytical>();
  Eigen::VectorXd q = ik->getPositions();
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(q.size());
  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), grad.size());
  analytical.evalGradient(q, gradMap);
  EXPECT_TRUE(grad.isZero(1e-12));
}

TEST(HierarchicalIK, ObjectiveConstraintAndClone)
{
  auto chain = makeIkChain();
  auto endIk = chain.endEffector->getOrCreateIK();
  auto rootIk = chain.root->getOrCreateIK();
  ASSERT_NE(endIk, nullptr);
  ASSERT_NE(rootIk, nullptr);

  endIk->setHierarchyLevel(1);
  rootIk->setHierarchyLevel(0);

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "hik_target");
  target->setTransform(chain.endEffector->getWorldTransform());
  endIk->setTarget(target);

  auto composite = CompositeIK::create(chain.skeleton);
  ASSERT_NE(composite, nullptr);
  EXPECT_TRUE(composite->addModule(endIk));
  EXPECT_TRUE(composite->addModule(rootIk));

  auto objective = std::make_shared<SimpleHierarchicalObjective>(composite);
  composite->setObjective(objective);
  composite->setNullSpaceObjective(objective);
  EXPECT_TRUE(composite->hasNullSpaceObjective());

  composite->refreshIKHierarchy();
  EXPECT_FALSE(composite->getIKHierarchy().empty());

  const auto problem = composite->getProblem();
  ASSERT_NE(problem, nullptr);
  ASSERT_NE(problem->getObjective(), nullptr);
  ASSERT_NE(problem->getEqConstraint(0u), nullptr);

  Eigen::VectorXd q = composite->getSkeleton()->getPositions();
  const double cost = problem->getObjective()->eval(q);
  EXPECT_GE(cost, 0.0);

  Eigen::VectorXd grad = Eigen::VectorXd::Zero(q.size());
  Eigen::Map<Eigen::VectorXd> gradMap(grad.data(), grad.size());
  problem->getObjective()->evalGradient(q, gradMap);
  EXPECT_TRUE(grad.array().isFinite().all());

  const double constraint = problem->getEqConstraint(0u)->eval(q);
  EXPECT_GE(constraint, 0.0);

  grad.setZero();
  problem->getEqConstraint(0u)->evalGradient(q, gradMap);
  EXPECT_TRUE(grad.array().isFinite().all());

  auto solver = std::dynamic_pointer_cast<math::GradientDescentSolver>(
      composite->getSolver());
  ASSERT_NE(solver, nullptr);
  solver->setNumMaxIterations(2);

  Eigen::VectorXd positions;
  composite->solveAndApply(positions, false);
  EXPECT_EQ(positions.size(), composite->getSkeleton()->getNumDofs());

  auto clonedSkel = chain.skeleton->cloneSkeleton();
  auto clonedComposite = composite->cloneCompositeIK(clonedSkel);
  ASSERT_NE(clonedComposite, nullptr);
  EXPECT_EQ(
      clonedComposite->getSkeleton()->getNumDofs(), clonedSkel->getNumDofs());

  auto wholeBody = WholeBodyIK::create(chain.skeleton);
  ASSERT_NE(wholeBody, nullptr);
  wholeBody->refreshIKHierarchy();
  EXPECT_FALSE(wholeBody->getIKHierarchy().empty());
}

TEST(InverseKinematics, FindSolutionWithNullProblem)
{
  auto fixture = makeIkSkeleton();

  class NullProblemIK final : public InverseKinematics
  {
  public:
    explicit NullProblemIK(JacobianNode* node) : InverseKinematics(node) {}

    void clearProblem()
    {
      mProblem.reset();
    }
  };

  auto ik = std::make_shared<NullProblemIK>(fixture.end);
  ik->clearProblem();

  Eigen::VectorXd positions;
  EXPECT_FALSE(ik->findSolution(positions));
}

TEST(InverseKinematics, ErrorMethodSizeMismatchAndCaching)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto& errorMethod = ik->setErrorMethod<InverseKinematics::TaskSpaceRegion>();

  Eigen::VectorXd wrongSize(1);
  wrongSize << 0.0;
  const auto& mismatch = errorMethod.evalError(wrongSize);
  EXPECT_TRUE(mismatch.isZero());

  Eigen::VectorXd empty;
  const auto& emptyError = errorMethod.evalError(empty);
  EXPECT_TRUE(emptyError.isZero());

  auto target
      = std::make_shared<SimpleFrame>(Frame::World(), "ik_error_target");
  target->setTransform(fixture.end->getTransform());
  ik->setTarget(target);

  Eigen::VectorXd q = ik->getPositions();
  const auto& first = errorMethod.evalError(q);
  const auto& second = errorMethod.evalError(q);
  EXPECT_TRUE(first.isApprox(second));

  const auto desired
      = errorMethod.computeDesiredTransform(fixture.end->getTransform(), first);
  EXPECT_TRUE(desired.matrix().array().isFinite().all());
}

TEST(InverseKinematics, TaskSpaceRegionCenterBoundsAndOffset)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto& tsr = ik->setErrorMethod<InverseKinematics::TaskSpaceRegion>();
  InverseKinematics::ErrorMethod::Bounds bounds;
  bounds.first = Eigen::Vector6d::Constant(-0.01);
  bounds.second = Eigen::Vector6d::Constant(0.01);
  bounds.second[1] = std::numeric_limits<double>::infinity();
  bounds.first[3] = -std::numeric_limits<double>::infinity();
  tsr.setBounds(bounds);
  tsr.setComputeFromCenter(true);
  tsr.setErrorLengthClamp(0.005);

  auto reference
      = std::make_shared<SimpleFrame>(Frame::World(), "ik_ref_center");
  Eigen::Isometry3d refTf = Eigen::Isometry3d::Identity();
  refTf.translation() = Eigen::Vector3d(0.2, 0.1, -0.1);
  reference->setTransform(refTf);
  tsr.setReferenceFrame(reference);

  auto target
      = std::make_shared<SimpleFrame>(Frame::World(), "ik_target_center");
  Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();
  targetTf.translation() = Eigen::Vector3d(-0.3, 0.4, 0.2);
  targetTf.linear() = (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX())
                       * Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY()))
                          .toRotationMatrix();
  target->setTransform(targetTf);
  ik->setTarget(target);

  ik->setOffset(Eigen::Vector3d(0.1, 0.0, 0.0));

  const auto& error = tsr.evalError(ik->getPositions());
  EXPECT_TRUE(error.array().isFinite().all());
  EXPECT_LE(error.norm(), tsr.getErrorLengthClamp() + 1e-12);
}

TEST(HierarchicalIK, FindSolutionErrorPaths)
{
  auto skeleton = Skeleton::create("hik_error_paths");
  auto composite = CompositeIK::create(skeleton);
  ASSERT_NE(composite, nullptr);

  Eigen::VectorXd positions;
  composite->setSolver(nullptr);
  EXPECT_FALSE(composite->findSolution(positions));

  class NullProblemCompositeIK final : public CompositeIK
  {
  public:
    explicit NullProblemCompositeIK(const SkeletonPtr& skel) : CompositeIK(skel)
    {
    }

    static std::shared_ptr<NullProblemCompositeIK> create(
        const SkeletonPtr& skel)
    {
      auto ik = std::shared_ptr<NullProblemCompositeIK>(
          new NullProblemCompositeIK(skel));
      ik->initialize(ik);
      return ik;
    }

    void clearProblem()
    {
      mProblem.reset();
    }
  };

  auto nullProblem = NullProblemCompositeIK::create(skeleton);
  nullProblem->clearProblem();
  EXPECT_FALSE(nullProblem->findSolution(positions));

  auto compositeForSkeleton = CompositeIK::create(skeleton);
  skeleton.reset();
  EXPECT_FALSE(compositeForSkeleton->findSolution(positions));
}

TEST(HierarchicalIK, CompositeModuleSetAccessors)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.root->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto composite = CompositeIK::create(fixture.skeleton);
  ASSERT_NE(composite, nullptr);
  EXPECT_TRUE(composite->addModule(ik));

  const auto& modules = composite->getModuleSet();
  EXPECT_EQ(modules.size(), 1u);

  const CompositeIK* constComposite = composite.get();
  auto constModules = constComposite->getModuleSet();
  EXPECT_EQ(constModules.size(), 1u);
}

TEST(HierarchicalIK, CompositeRejectsForeignModule)
{
  auto skeletonA = Skeleton::create("hik_a");
  auto skeletonB = Skeleton::create("hik_b");
  auto pairA = skeletonA->createJointAndBodyNodePair<RevoluteJoint>();
  auto pairB = skeletonB->createJointAndBodyNodePair<RevoluteJoint>();
  pairA.first->setAxis(Eigen::Vector3d::UnitZ());
  pairB.first->setAxis(Eigen::Vector3d::UnitZ());

  auto ik = pairA.second->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  auto composite = CompositeIK::create(skeletonB);
  ASSERT_NE(composite, nullptr);
  EXPECT_FALSE(composite->addModule(ik));
}

TEST(HierarchicalIK, WholeBodyRefreshWithNoModules)
{
  auto skeleton = Skeleton::create("hik_empty");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitY());

  auto wholeBody = WholeBodyIK::create(skeleton);
  ASSERT_NE(wholeBody, nullptr);
  wholeBody->refreshIKHierarchy();
  EXPECT_TRUE(wholeBody->getIKHierarchy().empty());
}

TEST(InverseKinematics, EndEffectorCreateIkSolveGradientDescent)
{
  auto chain = makeIkChain();
  auto ik = chain.endEffector->createIK();
  ASSERT_NE(ik, nullptr);

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target_gd");
  Eigen::Isometry3d targetTf = chain.endEffector->getWorldTransform();
  targetTf.translation() += Eigen::Vector3d(0.05, -0.02, 0.1);
  target->setTransform(targetTf);
  ik->setTarget(target);

  ik->setOffset(Eigen::Vector3d(0.0, 0.0, 0.05));
  auto solver = std::make_shared<math::GradientDescentSolver>();
  solver->setNumMaxIterations(3);
  solver->setTolerance(1e-6);
  ik->setSolver(solver);

  Eigen::VectorXd solution;
  ik->solveAndApply(solution, true);
  EXPECT_EQ(solution.size(), static_cast<int>(ik->getDofs().size()));
}

TEST(InverseKinematics, EndEffectorCreateIkSolveAnalytical)
{
  auto chain = makeIkChain();
  auto ik = chain.endEffector->createIK();
  ASSERT_NE(ik, nullptr);

  auto target = std::make_shared<SimpleFrame>(Frame::World(), "ik_target_an");
  Eigen::Isometry3d targetTf = chain.endEffector->getWorldTransform();
  targetTf.translation() += Eigen::Vector3d(-0.05, 0.03, 0.08);
  target->setTransform(targetTf);
  ik->setTarget(target);

  ik->setGradientMethod<DummyAnalytical>();
  auto solver = std::make_shared<math::GradientDescentSolver>();
  solver->setNumMaxIterations(1);
  solver->setTolerance(1e-6);
  ik->setSolver(solver);

  Eigen::VectorXd positions;
  ik->solveAndApply(positions, true);
  EXPECT_EQ(positions.size(), static_cast<int>(ik->getDofs().size()));
  EXPECT_NE(ik->getAnalytical(), nullptr);
}

TEST(InverseKinematics, TargetNullUsesCurrentTransform)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  ik->setTarget(nullptr);
  auto target = ik->getTarget();
  ASSERT_NE(target, nullptr);
  EXPECT_TRUE(target->getTransform().matrix().isApprox(
      fixture.end->getWorldTransform().matrix()));
}

TEST(InverseKinematics, OffsetToggleAndActiveFlag)
{
  auto fixture = makeIkSkeleton();
  auto ik = fixture.end->getOrCreateIK();
  ASSERT_NE(ik, nullptr);

  ik->setActive(false);
  EXPECT_FALSE(ik->isActive());
  ik->setActive(true);
  EXPECT_TRUE(ik->isActive());

  ik->setOffset(Eigen::Vector3d(0.1, 0.0, 0.0));
  EXPECT_TRUE(ik->hasOffset());
  ik->setOffset(Eigen::Vector3d::Zero());
  EXPECT_FALSE(ik->hasOffset());
}

TEST(HierarchicalIK, CompositeIgnoresInactiveModulesInNullspace)
{
  auto chain = makeIkChain();
  auto rootIk = chain.root->getOrCreateIK();
  auto endIk = chain.endEffector->getOrCreateIK();
  ASSERT_NE(rootIk, nullptr);
  ASSERT_NE(endIk, nullptr);

  rootIk->setHierarchyLevel(0);
  endIk->setHierarchyLevel(1);
  endIk->setActive(false);

  auto composite = CompositeIK::create(chain.skeleton);
  ASSERT_NE(composite, nullptr);
  EXPECT_TRUE(composite->addModule(rootIk));
  EXPECT_TRUE(composite->addModule(endIk));
  composite->refreshIKHierarchy();

  const auto nullspaces = composite->computeNullSpaces();
  EXPECT_EQ(nullspaces.size(), composite->getIKHierarchy().size());
}

TEST(InverseKinematics, TaskSpaceRegionNonWorldReference)
{
  auto chain = makeIkChain();
  auto ik = chain.endEffector->createIK();
  ASSERT_NE(ik, nullptr);

  auto& tsr = ik->setErrorMethod<InverseKinematics::TaskSpaceRegion>();
  tsr.setBounds();
  tsr.setComputeFromCenter(false);
  tsr.setErrorLengthClamp(0.2);
  tsr.setErrorWeights(Eigen::Vector6d::Ones());

  auto localRef = std::make_shared<SimpleFrame>(chain.tip, "ik_local_ref");
  Eigen::Isometry3d localTf = Eigen::Isometry3d::Identity();
  localTf.translation() = Eigen::Vector3d(0.05, -0.02, 0.01);
  localRef->setRelativeTransform(localTf);
  tsr.setReferenceFrame(localRef);

  auto target
      = std::make_shared<SimpleFrame>(Frame::World(), "ik_target_local");
  Eigen::Isometry3d targetTf = chain.endEffector->getWorldTransform();
  targetTf.translation() += Eigen::Vector3d(0.02, -0.03, 0.04);
  target->setTransform(targetTf);
  ik->setTarget(target);

  const auto& error = tsr.evalError(ik->getPositions());
  EXPECT_TRUE(error.array().isFinite().all());
}

TEST(InverseKinematics, AnalyticalPropertiesConstructors)
{
  using Analytical = InverseKinematics::Analytical;

  Analytical::UniqueProperties uniqueProps(
      Analytical::PRE_ANALYTICAL,
      0.15,
      [](const Eigen::VectorXd&,
         const Eigen::VectorXd&,
         const InverseKinematics*) { return true; });

  EXPECT_EQ(uniqueProps.mExtraDofUtilization, Analytical::PRE_ANALYTICAL);
  EXPECT_NEAR(uniqueProps.mExtraErrorLengthClamp, 0.15, 1e-12);
  EXPECT_TRUE(uniqueProps.mQualityComparator(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), nullptr));

  uniqueProps.resetQualityComparisonFunction();

  InverseKinematics::GradientMethod::Properties gradProps;
  gradProps.mComponentWiseClamp = 0.2;

  Analytical::Properties propsA(gradProps, uniqueProps);
  Analytical::Properties propsB(uniqueProps);

  EXPECT_NEAR(propsA.mComponentWiseClamp, 0.2, 1e-12);
  EXPECT_EQ(propsB.mExtraDofUtilization, uniqueProps.mExtraDofUtilization);
}

TEST(InverseKinematics, SolveAndApplyWithWholeBodyIK)
{
  auto chain = makeIkChain();
  auto ik = chain.endEffector->createIK();
  ASSERT_NE(ik, nullptr);

  auto target
      = std::make_shared<SimpleFrame>(Frame::World(), "ik_target_wholebody");
  Eigen::Isometry3d targetTf = chain.endEffector->getWorldTransform();
  targetTf.translation() += Eigen::Vector3d(0.02, 0.01, -0.03);
  target->setTransform(targetTf);
  ik->setTarget(target);

  auto solver = std::make_shared<math::GradientDescentSolver>();
  solver->setNumMaxIterations(2);
  solver->setTolerance(1e-6);
  ik->setSolver(solver);

  ik->solveAndApply(true);

  auto wholeBody = chain.skeleton->getIK(true);
  ASSERT_NE(wholeBody, nullptr);
  wholeBody->refreshIKHierarchy();
  wholeBody->solveAndApply();
  EXPECT_FALSE(wholeBody->getIKHierarchy().empty());
}

TEST(HierarchicalIK, RefreshHierarchyPriorityLevels)
{
  auto chain = makeIkChain();
  auto rootIk = chain.root->getOrCreateIK();
  auto midIk = chain.mid->getOrCreateIK();
  ASSERT_NE(rootIk, nullptr);
  ASSERT_NE(midIk, nullptr);

  rootIk->setHierarchyLevel(0);
  midIk->setHierarchyLevel(2);

  auto composite = CompositeIK::create(chain.skeleton);
  ASSERT_NE(composite, nullptr);
  EXPECT_TRUE(composite->addModule(rootIk));
  EXPECT_TRUE(composite->addModule(midIk));
  composite->refreshIKHierarchy();

  const auto& hierarchy = composite->getIKHierarchy();
  ASSERT_EQ(hierarchy.size(), 3u);
  EXPECT_EQ(hierarchy[0].size(), 1u);
  EXPECT_EQ(hierarchy[2].size(), 1u);
}

TEST(HierarchicalIK, WholeBodyCloneCopiesHierarchy)
{
  auto chain = makeIkChain();
  auto rootIk = chain.root->getOrCreateIK();
  auto endIk = chain.endEffector->getOrCreateIK();
  ASSERT_NE(rootIk, nullptr);
  ASSERT_NE(endIk, nullptr);

  rootIk->setHierarchyLevel(0);
  endIk->setHierarchyLevel(1);

  auto wholeBody = WholeBodyIK::create(chain.skeleton);
  ASSERT_NE(wholeBody, nullptr);
  wholeBody->refreshIKHierarchy();

  auto clonedSkel = chain.skeleton->cloneSkeleton();
  auto cloned = wholeBody->clone(clonedSkel);
  ASSERT_NE(cloned, nullptr);
  cloned->refreshIKHierarchy();
  EXPECT_EQ(cloned->getSkeleton()->getNumDofs(), clonedSkel->getNumDofs());
}

TEST(HierarchicalIK, CloneWholeBodyIkExplicit)
{
  auto chain = makeIkChain();
  auto rootIk = chain.root->getOrCreateIK();
  auto endIk = chain.endEffector->getOrCreateIK();
  ASSERT_NE(rootIk, nullptr);
  ASSERT_NE(endIk, nullptr);

  rootIk->setHierarchyLevel(0);
  endIk->setHierarchyLevel(1);

  auto wholeBody = WholeBodyIK::create(chain.skeleton);
  ASSERT_NE(wholeBody, nullptr);
  wholeBody->refreshIKHierarchy();

  auto clonedSkel = chain.skeleton->cloneSkeleton();
  auto clonedWhole = wholeBody->cloneWholeBodyIK(clonedSkel);
  ASSERT_NE(clonedWhole, nullptr);
  EXPECT_EQ(clonedWhole->getSkeleton()->getNumDofs(), clonedSkel->getNumDofs());
}

TEST(InverseKinematics, CloneCopiesErrorAndGradientMethods)
{
  auto chain = makeIkChain();
  auto ik = InverseKinematics::create(chain.endEffector);
  ASSERT_NE(ik, nullptr);

  auto& tsr = ik->setErrorMethod<InverseKinematics::TaskSpaceRegion>();
  tsr.setBounds(
      Eigen::Vector6d::Constant(-0.1), Eigen::Vector6d::Constant(0.1));
  tsr.setComputeFromCenter(true);
  tsr.setErrorLengthClamp(0.2);

  auto& gradient = ik->setGradientMethod<InverseKinematics::JacobianDLS>();
  gradient.setDampingCoefficient(0.03);
  gradient.setComponentWiseClamp(0.15);

  ik->setObjective(std::make_shared<SimpleIkObjective>(ik.get()));
  ik->setNullSpaceObjective(std::make_shared<SimpleIkObjective>(ik.get()));

  auto clonedSkel = chain.skeleton->cloneSkeleton();
  auto* clonedEnd = clonedSkel->getEndEffector("ee");
  ASSERT_NE(clonedEnd, nullptr);

  auto clonedIk = ik->clone(clonedEnd);
  ASSERT_NE(clonedIk, nullptr);

  auto* clonedTsr = dynamic_cast<InverseKinematics::TaskSpaceRegion*>(
      &clonedIk->getErrorMethod());
  ASSERT_NE(clonedTsr, nullptr);
  EXPECT_TRUE(
      clonedTsr->getBounds().first.isApprox(Eigen::Vector6d::Constant(-0.1)));
  EXPECT_TRUE(clonedTsr->isComputingFromCenter());
  EXPECT_DOUBLE_EQ(clonedTsr->getErrorLengthClamp(), 0.2);

  auto* clonedDls = dynamic_cast<InverseKinematics::JacobianDLS*>(
      &clonedIk->getGradientMethod());
  ASSERT_NE(clonedDls, nullptr);
  EXPECT_NEAR(clonedDls->getDampingCoefficient(), 0.03, 1e-12);
  EXPECT_DOUBLE_EQ(clonedDls->getComponentWiseClamp(), 0.15);

  EXPECT_NE(clonedIk->getObjective(), nullptr);
  EXPECT_TRUE(clonedIk->hasNullSpaceObjective());
}
