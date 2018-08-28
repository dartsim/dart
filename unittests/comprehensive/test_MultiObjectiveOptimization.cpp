#include <fstream>
//#include <Python.h>
#include <gtest/gtest.h>
#include <dart/config.hpp>
#include <dart/common/Memory.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/MultiObjectiveProblem.hpp>
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
  auto pFunc1 = std::make_shared<Func1>();
  auto pFunc2 = std::make_shared<Func2>();

  std::vector<FunctionPtr> pFuncs;
  pFuncs.push_back(pFunc1);
  pFuncs.push_back(pFunc2);

#ifdef NDEBUG // release mode
  int numSamples = 200;
#else
  int numSamples = 10;
#endif
#ifdef NDEBUG // release mode
  std::size_t iterationNum = 1000;
#else
  std::size_t iterationNum = 200;
#endif

  auto problem = std::make_shared<optimizer::MultiObjectiveProblem>(dimension);
  problem->setObjectiveFunctions(pFuncs);
  problem->setLowerBounds(lowerLimits);
  problem->setUpperBounds(upperLimits);

  solver.setProblem(problem);
  solver.setPopulationSize(numSamples);
  solver.setNumPopulations(100);
  solver.setNumIterationsPerEvolution(iterationNum);
  solver.solve(100);

  std::cout << *problem << "\n";

  const optimizer::Population& pop0 = solver.getPopulation(0);
  std::cout << pop0 << "\n";

  const optimizer::Population& pop1 = solver.getPopulation(1);
  std::cout << pop1 << "\n";

  std::cout << "num populations: " << solver.getNumPopulations() << std::endl;
}

//==============================================================================
TEST(ZDT1, Basic)
{
  optimizer::PagmoMultiObjectiveSolver pagmoSolver;
  testZDT1(pagmoSolver);
}
