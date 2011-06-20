#include <iostream>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

// For problem
#include "optimizer/Problem.h"
#include "optimizer/ObjectiveBox.h"
using namespace optimizer;
#include "SampleConstraint.h"

// For solver
#include "optimizer/snopt/SnoptSolver.h"


int main(int argc, char* argv[]) {
    cout << "Sample Optimizer" << endl;

    Problem prob;
    prob.addVariable(0.0, -10.0, 10.0);

    SampleConstraint* c = new SampleConstraint(
        prob.vars(), 0, 3.0);
    prob.createBoxes();
    prob.objBox()->Add(c);

    snopt::SnoptSolver solver(&prob);
    cout << "test = " << prob.objBox()->mdG.size() << endl;
    solver.Recipe();


    delete c;
    cout << "terminated" << endl;
}
