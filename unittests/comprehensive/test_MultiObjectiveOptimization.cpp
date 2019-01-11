#include <fstream>
#include <dart/common/Console.hpp>
#include <dart/common/Memory.hpp>
#include <dart/config.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/GenericMultiObjectiveProblem.hpp>
#include <dart/optimizer/MultiObjectiveSolver.hpp>
#include <gtest/gtest.h>
#if HAVE_PAGMO
#include <dart/optimizer/pagmo/pagmo.hpp>
#endif

using namespace dart;
using namespace dart::optimizer;
using dart::optimizer::Function;
using dart::optimizer::FunctionPtr;
using dart::optimizer::UniqueFunctionPtr;

//==============================================================================
static int dimension = 10;
static Eigen::VectorXd lowerLimits = Eigen::VectorXd::Zero(dimension);
static Eigen::VectorXd upperLimits = Eigen::VectorXd::Constant(dimension, 1.0);

class ZDT1 : public MultiObjectiveProblem
{
public:
  ZDT1() : MultiObjectiveProblem(dimension)
  {
    // Do nothing
  }

  std::size_t getObjectiveDimension() const override
  {
    return 2u;
  }

  Eigen::VectorXd evaluateObjectives(const Eigen::VectorXd& x) const override
  {
    Eigen::VectorXd ret(getObjectiveDimension());

    ret[0] = x[0];

    const double g = 1.0 + 9 * (x.sum() - x[0]) / double(dimension - 1);
    ret[1] = g * (1.0 - std::sqrt(x[0] / g));

    return ret;
  }

protected:
};

//==============================================================================
class Func1 : public Function
{
public:
  Func1() = default;

  double eval(const Eigen::VectorXd& x) const override
  {
    return x[0];
  }

  UniqueFunctionPtr clone() const
  {
    return dart::common::make_unique<Func1>(*this);
  }

  std::size_t getParameterDimension() const
  {
    return 1u;
  }
};

//==============================================================================
class Func2 : public Function
{
public:
  Func2() = default;

  double eval(const Eigen::VectorXd& x) const override
  {
    double g = 1.0 + 9 * (x.sum() - x[0]) / double(dimension - 1);
    return g * (1.0 - std::sqrt(x[0] / g));
  }

  UniqueFunctionPtr clone() const
  {
    return dart::common::make_unique<Func2>(*this);
  }

  std::size_t getParameterDimension() const
  {
    return 1u;
  }
};

//==============================================================================
void testZDT1(MultiObjectiveSolver& solver)
{
#ifdef NDEBUG // release mode
  std::size_t numSolutions = 200;
#else
  std::size_t numSolutions = 10;
#endif
#ifdef NDEBUG // release mode
  std::size_t iterationNum = 1000;
#else
  std::size_t iterationNum = 200;
#endif

  auto problem = std::make_shared<ZDT1>();
  problem->setLowerBounds(lowerLimits);
  problem->setUpperBounds(upperLimits);

  solver.setProblem(problem);
  solver.setPopulationSize(numSolutions);
  solver.setNumPopulations(100);
  solver.setNumIterationsPerEvolution(iterationNum);
  solver.solve(100);

  EXPECT_TRUE(solver.getPopulation(0).getSize() == numSolutions);
  EXPECT_TRUE(solver.getPopulation(1).getSize() == numSolutions);
  EXPECT_TRUE(solver.getNumPopulations() == 100);
}

//==============================================================================
void testZDT1Generic(MultiObjectiveSolver& solver)
{
  auto pFunc1 = std::make_shared<Func1>();
  auto pFunc2 = std::make_shared<Func2>();

  std::vector<FunctionPtr> pFuncs;
  pFuncs.push_back(pFunc1);
  pFuncs.push_back(pFunc2);

#ifdef NDEBUG // release mode
  std::size_t numSolutions = 200;
#else
  std::size_t numSolutions = 10;
#endif
#ifdef NDEBUG // release mode
  std::size_t iterationNum = 1000;
#else
  std::size_t iterationNum = 200;
#endif

  auto problem
      = std::make_shared<optimizer::GenericMultiObjectiveProblem>(dimension);
  problem->setObjectiveFunctions(pFuncs);
  problem->setLowerBounds(lowerLimits);
  problem->setUpperBounds(upperLimits);

  solver.setProblem(problem);
  solver.setPopulationSize(numSolutions);
  solver.setNumPopulations(100);
  solver.setNumIterationsPerEvolution(iterationNum);
  solver.solve(100);

  EXPECT_TRUE(solver.getPopulation(0).getSize() == numSolutions);
  EXPECT_TRUE(solver.getPopulation(1).getSize() == numSolutions);
  EXPECT_TRUE(solver.getNumPopulations() == 100);
}

//==============================================================================
TEST(ZDT1, Basic)
{
#if HAVE_PAGMO
  PagmoMultiObjectiveSolver pagmoSolver;
  testZDT1(pagmoSolver);
  testZDT1Generic(pagmoSolver);
#endif
}
