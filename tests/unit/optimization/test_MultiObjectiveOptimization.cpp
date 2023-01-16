#include <dart/config.hpp>

#include <dart/optimization/Function.hpp>
#include <dart/optimization/GenericMultiObjectiveProblem.hpp>
#include <dart/optimization/MultiObjectiveSolver.hpp>
#if DART_HAVE_PAGMO
  #include <dart/optimization/pagmo/PagmoMultiObjectiveSolver.hpp>
#endif

#include <dart/common/Console.hpp>
#include <dart/common/Memory.hpp>

#include <gtest/gtest.h>

#include <fstream>

using namespace dart;
using namespace dart::optimization;
using dart::optimization::Function;
using dart::optimization::FunctionPtr;
using dart::optimization::UniqueFunctionPtr;

//==============================================================================
static int dimension = 10;
static math::VectorXd lowerLimits = math::VectorXd::Zero(dimension);
static math::VectorXd upperLimits = math::VectorXd::Constant(dimension, 1.0);

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

  math::VectorXd evaluateObjectives(const math::VectorXd& x) const override
  {
    math::VectorXd ret(getObjectiveDimension());

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

  double eval(const math::VectorXd& x) const override
  {
    return x[0];
  }

  UniqueFunctionPtr clone() const
  {
    return std::make_unique<Func1>(*this);
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

  double eval(const math::VectorXd& x) const override
  {
    double g = 1.0 + 9 * (x.sum() - x[0]) / double(dimension - 1);
    return g * (1.0 - std::sqrt(x[0] / g));
  }

  UniqueFunctionPtr clone() const
  {
    return std::make_unique<Func2>(*this);
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
  std::size_t numSolutions = 50;
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
  std::size_t numSolutions = 50;
#else
  std::size_t numSolutions = 10;
#endif
#ifdef NDEBUG // release mode
  std::size_t iterationNum = 1000;
#else
  std::size_t iterationNum = 200;
#endif

  auto problem
      = std::make_shared<optimization::GenericMultiObjectiveProblem>(dimension);
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
#if DART_HAVE_PAGMO
  PagmoMultiObjectiveSolver pagmoSolver;
  testZDT1(pagmoSolver);
  testZDT1Generic(pagmoSolver);
#endif
}
