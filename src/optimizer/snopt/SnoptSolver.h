/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#ifndef OPTIMIZER_SNOPT_SNOPT_H
#define OPTIMIZER_SNOPT_SNOPT_H

#include <vector>
#include <Eigen/Dense>
#include "SnoptInterface.h"
#include "optimizer/Solver.h"

namespace optimizer {
    class Problem;
    class ConstraintBox;
    class ObjectiveBox;
    class Var;
    
    namespace snopt {

        class SnoptSolver : public Solver {
        public:
            SnoptSolver(Problem *problem);
            virtual ~SnoptSolver();

            virtual bool solve() { return true; }
            virtual Eigen::VectorXd getState() {
                return Eigen::VectorXd(1); }
            virtual void setState(const Eigen::VectorXd& x) {}

            virtual bool Recipe();
            virtual void ResetSolver();
            virtual SnoptInterface::Return Solve();

            static int IterUpdate(long mask, int compute_gradients, double *coefs, void *update_data);
        private:
            SnoptInterface *mSnopt;
            /* std::vector<Dofs> mVariables; */
            /* ConstrBox* mConstrBox; */
            /* ObjBox* mObjBox; */

            ConstraintBox* conBox();
            ObjectiveBox* objBox();

            bool mNoDisplay;
            int mSolverIter;

            int mTotalDofs;
            int mOptCount;
            bool mPrint;
            int mUnit;


        };

        
    } // namespace snopt
} // namespace optimizer

#endif // #ifndef OPTIMIZER_SNOPT_SNOPT_H

