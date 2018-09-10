#include <fstream>
#include <dart/common/Console.hpp>
#include <dart/common/Memory.hpp>
#include <dart/config.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/GenericMultiObjectiveProblem.hpp>
#include <dart/optimizer/McmcMultiObjectiveSolver.hpp>
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

    const double g = 1.0 + 9.0 * (x.sum() - x[0]) / double(dimension - 1);
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

  double eval(const Eigen::VectorXd& x) override
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

  double eval(const Eigen::VectorXd& x) override
  {
    double g = 1.0 + 9.0 * (x.sum() - x[0]) / double(dimension - 1);
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
std::shared_ptr<MultiObjectiveProblem> createZDT1()
{
  auto problem = std::make_shared<ZDT1>();
  problem->setLowerBounds(lowerLimits);
  problem->setUpperBounds(upperLimits);

  return problem;
}

//==============================================================================
std::shared_ptr<MultiObjectiveProblem> createZDT1Generic()
{
  auto pFunc1 = std::make_shared<Func1>();
  auto pFunc2 = std::make_shared<Func2>();

  std::vector<FunctionPtr> pFuncs;
  pFuncs.push_back(pFunc1);
  pFuncs.push_back(pFunc2);

  auto problem
      = std::make_shared<optimizer::GenericMultiObjectiveProblem>(dimension);
  problem->setObjectiveFunctions(pFuncs);
  problem->setLowerBounds(lowerLimits);
  problem->setUpperBounds(upperLimits);

  return problem;
}

//==============================================================================
void testBasics(
    std::shared_ptr<MultiObjectiveProblem> prob, MultiObjectiveSolver& solver)
{
  std::size_t numPopulations = 1;

#ifdef NDEBUG // release mode
  std::size_t numSolutions = 100;
#else
  std::size_t numSolutions = 10;
#endif

#ifdef NDEBUG // release mode
  std::size_t numIterationPerEvolution = 200;
#else
  std::size_t numIterationPerEvolution = 20;
#endif

  solver.setProblem(prob);
  solver.setNumSolutions(numSolutions);
  solver.setNumPopulations(numPopulations);
  solver.setNumIterationsPerEvolution(numIterationPerEvolution);
  solver.solve(1);

  for (std::size_t i = 0u; i < numPopulations; ++i)
    EXPECT_TRUE(solver.getPopulation(i).getNumSolutions() == numSolutions);
  EXPECT_TRUE(solver.getNumPopulations() == numPopulations);

  std::cout << solver.getPopulation(0) << std::endl;
}

//==============================================================================
TEST(ZDT1, Basics)
{
  auto zdt1 = createZDT1();
  auto zdt1Generic = createZDT1Generic();

#if HAVE_PAGMO
  PagmoMultiObjectiveSolver pagmoSolver;
  testBasics(zdt1, pagmoSolver);
//  testBasics(zdt1Generic, pagmoSolver);
#endif

  MomcmcSolver momcmcSolver;
  testBasics(zdt1, momcmcSolver);
  //  testBasics(zdt1Generic, momcmcSolver);
}
