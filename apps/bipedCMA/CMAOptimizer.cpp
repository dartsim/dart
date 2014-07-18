/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "CMAOptimizer.h"

#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
// Implementation of the CMA-ES
#include <shark/Algorithms/DirectSearch/CMA.h>
#include <shark/ObjectiveFunctions/AbstractObjectiveFunction.h>
#include <shark/ObjectiveFunctions/Benchmarks/Benchmarks.h>
//
#include "MyWindow.h"
#include "Controller.h"

namespace shark {

struct MyFunc : public SingleObjectiveFunction {
	
  MyFunc(MyWindow* _window)
  : mWindow(_window)
  {
    m_features |= CAN_PROPOSE_STARTING_POINT;
    m_features |= IS_CONSTRAINED_FEATURE;
    // m_features |= CAN_PROVIDE_CLOSEST_FEASIBLE;
  }

  std::string name() const
    { return "MyFunc"; }

  std::size_t numberOfVariables()const{
    return mWindow->getController()->getNumParams();
  }
	
  bool hasScalableDimensionality()const{
    return false;
  }

  void proposeStartingPoint(SearchPointType &x) const {
    x.resize(numberOfVariables());
    x(0) = 10.0;
    x(1) = 10.0;
    x(2) = 10.0;
    // for (unsigned int i = 0; i < x.size(); i++) {
    //   x(i) = Rng::uni(0.0, 500.0);
    //}
  }

  bool isFeasible( const SearchPointType & input) const {
    int n = numberOfVariables();
    SIZE_CHECK(input.size() == n);
    if (input(0) < 0.0 || input(0) > 500.0)
      return false;
    if (input(1) < 0.0 || input(1) > 500.0)
      return false;
    if (input(2) < 0.0 || input(2) > 50)
      return false;

    // for(int i = 0; i < n; ++i) {
    //   if (input(i) < 0.0 || input(i) > 500.0)
    //     return false;
    // }
    return true;
  }

  void closestFeasible( SearchPointType & input ) const {
    int n = numberOfVariables();
    SIZE_CHECK(input.size() == n);
    input(0) = std::max(0.0,std::min(500.0,input(0)));
    input(1) = std::max(0.0,std::min(500.0,input(1)));
    input(2) = std::max(0.0,std::min(50.0,input(2)));

    // for(int i = 0; i < n; ++i) {
    //   input(i) = std::max(0.0,std::min(500.0,input(i)));
    // }
  }

  double eval(const SearchPointType &p) const {
    m_evaluationCounter++;

    // Setup the parameters
    Eigen::VectorXd params( p. size() );
    for (int i = 0; i < p.size(); i++) params(i) = p(i);
    std::cout << "reset..." << params.transpose() << std::endl;


    mWindow->getController()->setParams(params);
    mWindow->reset();
    while (!mWindow->isSimulationEnd()) {
      mWindow->timeStepping();
    }

    double value = mWindow->getController()->evaluate();
    std::cout << "simulation terminated... " << value << std::endl;
    return value;
  }
private:
  MyWindow* mWindow;
};

} // namespace sharkv


CMAOptimizer::CMAOptimizer() {
}

CMAOptimizer::~CMAOptimizer() {
}

void worker(MyWindow* window)
{
  using namespace shark;
  Rng::seed( (unsigned int) time (NULL) );
  // Instantiate both the problem
  MyFunc func( window );
  // sphere.setNumberOfVariables( 2 );
  // Initialize the optimizer for the objective function instance.
  CMA cma;
  RealVector start;
  func.proposeStartingPoint(start);
  // cma.init( func, start, 20, 10, 1.0 );
  cma.init( func, start, 8, 4, 20.0 );

  // // Iterate the optimizer until a solution of sufficient quality is found.
  // do {

  // } while(cma.solution().value > 1E-20 );	



  // for(;;)

  while(cma.solution().value > 0.01)
  {

    cma.step( func );

    // Report information on the optimizer state and the current solution to the console.
    std::cout << func.evaluationCounter() << " "	
              << cma.solution().value << " "
              << cma.solution().point << " "
              << cma.sigma() << std::endl;
  }
  std::cout << "CMAOptimizer:: problem solved" << std::endl;
}

bool CMAOptimizer::solve() {
  using namespace shark;
  boost::thread t(&worker, mWindow);
  return true;
}

void CMAOptimizer::setMyWindow(MyWindow* _window) {
  mWindow = _window;
}


