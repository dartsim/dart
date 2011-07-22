#include <iostream>
#include <iomanip>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

// For problem
#include "optimizer/Var.h"
#include "optimizer/Problem.h"
#include "optimizer/ObjectiveBox.h"
using namespace optimizer;
#include "SampleConstraint.h"

// For solver
#include "optimizer/snopt/SnoptSolver.h"


class SampleProblem : public Problem {
public:
    virtual void update(double* coefs) {
        cout << "TestProblem::update" << endl;
        for (int i = 0; i < mVariables.size(); ++i) {
            cout << "Value (" << i << ") = " << mVariables[i]->mVal << endl;
        }
    }
};

int main(int argc, char* argv[]) {
    cout << setprecision(12) << fixed;

    cout << endl;
    cout << "Sample Optimizer" << endl;
    cout << endl;


    // Define problem. you can define your own problem as well
    SampleProblem prob;
    // Problem prob;
    prob.addVariable(0.0, -10.0, 10.0);
    prob.createBoxes();

    // Create constraints and objectives
    SampleConstraint* c = new SampleConstraint(
        prob.vars(), 0, 3.0);
    prob.objBox()->Add(c);

    // Create a solever and solve
    snopt::SnoptSolver solver(&prob);
    solver.solve();

    cout << "Solution = " << solver.getState() << endl;


    delete c;

    cout << endl;
    cout << "OK" << endl;
    cout << endl;
}
