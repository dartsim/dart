/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#ifndef DART_OPTIMIZER_SNOPT_SNOPT_H
#define DART_OPTIMIZER_SNOPT_SNOPT_H

#include <vector>

#include <dart/optimization/snopt/SnoptInterface.h>
#include <dart/optimization/Solver.h>

namespace dart {
namespace optimization {

class Problem;
class ConstraintSet;
class ObjectiveSet;
class Variable;

namespace snopt {

class SnoptSolver : public Solver {
public:
    SnoptSolver(Problem *problem);
    virtual ~SnoptSolver();

    virtual bool solve();

    virtual void resetSolver();
    static int iterUpdate(long mask, int compute_gradients, double *coefs, void *update_data);
private:
    SnoptInterface *mSnopt;
    /* std::vector<Dofs> mVariables; */
    /* ConstrBox* mConstrBox; */
    /* ObjBox* mObjBox; */

    ConstraintSet* conBox();
    ObjectiveSet* objBox();

    bool mNoDisplay;
    int mSolverIter;

    int mTotalDofs;
    int mOptCount;
    bool mPrint;
    int mUnit;


};

} // namespace snopt
} // namespace optimization
} // namespace dart

#endif // #ifndef DART_OPTIMIZER_SNOPT_SNOPT_H

