
/**
 *  @file test_vars_constr_cost.h
 *
 *  @brief Example to generate a solver-independent formulation for the problem, taken
 *  from the IPOPT cpp_example.
 *
 *  The example problem to be solved is given as:
 *
 *      min_x f(x) = -(x1-2)^2
 *      s.t.
 *           0 = x0^2 + x1 - 1
 *           -1 <= x0 <= 1
 *
 * In this simple example we only use one set of variables, constraints and
 * cost. However, most real world problems have multiple different constraints
 * and also different variable sets representing different quantities. This
 * framework allows to define each set of variables or constraints absolutely
 * independently from another and correctly stitches them together to form the
 * final optimization problem.
 *
 * For a helpful graphical overview, see:
 * http://docs.ros.org/api/ifopt/html/group__ProblemFormulation.html
 */

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include "sources/MatrixConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/RTA/RTA_LL.h"
#include "sources/EnergyOptimization/Energy.h"

namespace rt_num_opt
{
    void Eigen2Array(VectorDynamic &x, double res[])
    {
        for (int i = 0; i < x.rows(); i++)
            res[i] = x(i);
    }
    struct GlobalInfoIfopt
    {
        TaskSetNormal tasks;
        VectorDynamic variableGlobalIfoptInitial;
    };
    GlobalInfoIfopt globalInfoIfopt;

    void InitializeGlobalInfoIfopt(TaskSetNormal &tasks)
    {
        globalInfoIfopt.tasks = tasks;
        globalInfoIfopt.variableGlobalIfoptInitial = GetParameterVD<double>(tasks.tasks_, "executionTime");
    }

      template <class TaskSetType, class Schedul_Analysis>
    class ExConstraint : public ifopt::ConstraintSet
    {
    public:
        ExConstraint() : ExConstraint("constraint1") {}

        // This constraint set just contains 1 constraint, however generally
        // each set can contain multiple related constraints.
        ExConstraint(const std::string &name) : ConstraintSet(1, name) {}

        boost::function<gtsam::Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &executionTimeVector)
        {
            TaskSetType taskT = globalInfoIfopt.tasks;
            UpdateTaskSetExecutionTime(taskT.tasks_, executionTimeVector);
            Schedul_Analysis r(taskT);
            if (r.CheckSchedulability())
            {
                return GenerateVectorDynamic1D(0);
            }
            else
            {
                return GenerateVectorDynamic1D(1);
            }
        };

        // The constraint value minus the constant value "1", moved to bounds.
        VectorDynamic GetValues() const override
        {
            VectorDynamic g(GetRows());
            VectorDynamic x = GetVariables()->GetComponent("var_set1")->GetValues();
            return f(x);
        };

        // The only constraint in this set is an equality constraint to 1.
        // Constant values should always be put into GetBounds(), not GetValues().
        // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
        VecBound GetBounds() const override
        {
            VecBound b(GetRows());
            b.at(0) = ifopt::Bounds(0, 0);
            return b;
        }

        // This function provides the first derivative of the constraints.
        // In case this is too difficult to write, you can also tell the solvers to
        // approximate the derivatives by finite differences and not overwrite this
        // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
        // Attention: see the parent class function for important information on sparsity pattern.
        void FillJacobianBlock(std::string var_set, Jacobian &jac_block) const override
        {
            // must fill only that submatrix of the overall Jacobian that relates
            // to this constraint and "var_set1". even if more constraints or variables
            // classes are added, this submatrix will always start at row 0 and column 0,
            // thereby being independent from the overall problem.
            if (var_set == "var_set1")
            {
                VectorDynamic x = GetVariables()->GetComponent("var_set1")->GetValues();

                // jac_block.coeffRef(0, 0) = 2.0 * x(0); // derivative of first constraint w.r.t x0
                // jac_block.coeffRef(0, 1) = 1.0;        // derivative of first constraint w.r.t x1

                MatrixDynamic jj = NumericalDerivativeDynamic(f, x, deltaOptimizer);

                for (uint i = 0; i < x.rows(); i++)
                    jac_block.coeffRef(0, i) = jj.coeff(0, i);
            }
        }
    };

}
