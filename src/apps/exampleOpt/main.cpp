#include <iostream>
#include <iomanip>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

// For problem
#include "optimizer/Var.h"
#include "optimizer/Problem.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/ConstraintBox.h"
using namespace optimizer;
#include "ExampleConstraint.h"

// For solver
#include "optimizer/snopt/SnoptSolver.h"


class ExampleProblem : public Problem {
public:
    virtual void update(double* coefs) {
        cout << "TestProblem::update" << endl;
        for (unsigned int i = 0; i < mVariables.size(); ++i) {
            cout << "Value (" << i << ") = " << mVariables[i]->mVal << endl;
        }
    }
};

int main(int argc, char* argv[]) {
    cout << setprecision(12) << fixed;

    cout << endl;
    cout << "Example Optimizer" << endl;
    cout << endl;


    // Define problem. you can define your own problem as well
    ExampleProblem prob;
    // Problem prob;
    prob.addVariable(4, -1e7, 1e7);
    prob.addVariable(-4, -1e7, 1e7);
    prob.createBoxes();

    // Create constraints and objectives
    ExampleConstraint* c = new ExampleConstraint(prob.vars(), 0, 3.0);
    //prob.objBox()->add(c);
    prob.conBox()->add(c);

    // Create a solver and solve
    snopt::SnoptSolver solver(&prob);
    solver.solve();

    cout << "Solution = " << solver.getState() << endl;


    delete c;

    cout << endl;
    cout << "OK" << endl;
    cout << endl;
}
