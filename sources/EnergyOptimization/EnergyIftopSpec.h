#pragma once
#include <iostream>
#include <chrono>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include "sources/Utils/helpifopt.h"

#include "sources/Utils/Parameters.h"
#include "sources/EnergyOptimization/Optimize.h"
#include "sources/RTA/RTA_LL.h"

namespace rt_num_opt
{

    class ExVariablesEnergy : public ifopt::VariableSet
    {
    private:
        VectorDynamic var_;
        TaskSet tasks_;
        VectorDynamic lowerBound_;

    public:
        // Every variable set has a name, here "var_set1". this allows the constraints
        // and costs to define values and Jacobians specifically w.r.t this variable set.
        ExVariablesEnergy(TaskSet &tasks) : ExVariablesEnergy(tasks, "var_set1"){};
        ExVariablesEnergy(TaskSet &tasks, const std::string &name) : VariableSet(tasks.size(), name)
        {
            // the initial values where the NLP starts iterating from
            tasks_ = tasks;
            lowerBound_ = GetParameterVD<double>(tasks_, "executionTimeOrg");
            var_ = lowerBound_;
        }

        // Here is where you can transform the Eigen::Vector into whatever
        // internal representation of your variables you have (here two doubles, but
        // can also be complex classes such as splines, etc..
        void SetVariables(const VectorDynamic &x) override
        {
            var_ = x;
        };

        // Here is the reverse transformation from the internal representation to
        // to the Eigen::Vector
        VectorDynamic GetValues() const override
        {
            return var_;
        };

        // Each variable has an upper and lower bound set here
        VecBound GetBounds() const override
        {
            VecBound bounds(GetRows());

            for (int i = 0; i < var_.rows(); i++)
            {
                if (enableMaxComputationTimeRestrict)
                    bounds.at(i) = ifopt::Bounds(lowerBound_(i), lowerBound_(i) * MaxComputationTimeRestrict);
                else
                    bounds.at(i) = ifopt::Bounds(lowerBound_(i), lowerBound_(i) * 10000);
            }
            return bounds;
        }
    };
    class ExCostEnergy : public ifopt::CostTerm
    {
    private:
        VectorDynamic var_;
        TaskSet tasks_;

    public:
        // otherParameters is actually never used, just for convenience of template
        ExCostEnergy(TaskSet &tasks, boost::optional<VectorDynamic> otherParameters = boost::none) : CostTerm("cost_term1"), tasks_(tasks) {}

        boost::function<gtsam::Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &executionTimeVector)
        {
            TaskSet taskT = tasks_;
            UpdateTaskSetExecutionTime(taskT, executionTimeVector);
            double energy = EstimateEnergyTaskSet(taskT).sum();
            return GenerateVectorDynamic1D(energy);
        };

        double GetCost() const override
        {
            VectorDynamic x = GetVariables()->GetComponent("var_set1")->GetValues();

            return f(x)(0);
        };

        void FillJacobianBlock(std::string var_set, Jacobian &jac) const override
        {
            if (var_set == "var_set1")
            {
                VectorDynamic x = GetVariables()->GetComponent("var_set1")->GetValues();

                // jac.coeffRef(0, 0) = 0.0;                 // derivative of cost w.r.t x0
                // jac.coeffRef(0, 1) = -2.0 * (x(1) - 2.0); // derivative of cost w.r.t x1
                MatrixDynamic jj = NumericalDerivativeDynamic(f, x, deltaOptimizer);
                for (uint i = 0; i < x.rows(); i++)
                {
                    jac.coeffRef(0, i) = jj.coeff(0, i); //
                }
            }
        }
    };

    template <class TaskSetType, class Schedul_Analysis>
    class ExConstraintEnergy : public ifopt::ConstraintSet
    {
    public:
        TaskSetType taskGeneral_;

        ExConstraintEnergy(TaskSetType &tasks) : ExConstraintEnergy(tasks, "constraint1") {}

        // This constraint set just contains 1 constraint, however generally
        // each set can contain multiple related constraints.
        ExConstraintEnergy(TaskSetType &tasks, const std::string &name) : ConstraintSet(1, name), taskGeneral_(tasks) {}

        boost::function<gtsam::Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &executionTimeVector)
        {
            TaskSetType taskT = taskGeneral_;
            UpdateTaskSetExecutionTime(taskT.tasks_, executionTimeVector);
            Schedul_Analysis r(taskT);
            if (r.CheckSchedulability())
            {
                return GenerateVectorDynamic1D(0);
            }
            else
            {
                if (debugMode == 1)
                    std::cout << "Infeasible!" << std::endl;
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
            if (var_set == "var_set1")
            {
                VectorDynamic x = GetVariables()->GetComponent("var_set1")->GetValues();
                MatrixDynamic jj = NumericalDerivativeDynamic(f, x, deltaOptimizer);

                for (uint i = 0; i < x.rows(); i++)
                    jac_block.coeffRef(0, i) = jj.coeff(0, i);
            }
        }
    };

    template <class TaskSetType, class Schedul_Analysis>
    VectorDynamic ClampEnergyResultBasedOnFeasibility(TaskSetType &tasks, VectorDynamic &x)
    {

        UpdateTaskSetExecutionTime(tasks, x);
        Schedul_Analysis r(tasks);
        if (!r.CheckSchedulability())
        {
            return GetParameterVD<double>(tasks, "executionTimeOrg");
        }
        else
            return x;
    }

    template <class TaskSetType, class Schedul_Analysis>
    double OptimizeEnergyIfopt(TaskSetType &tasksN)
    {
        VectorDynamic x = OptimizeIfopt<TaskSetType, ExVariablesEnergy, ExConstraintEnergy<TaskSetType, Schedul_Analysis>, ExCostEnergy>(tasksN);
        VectorDynamic correctedX = ClampEnergyResultBasedOnFeasibility<TaskSetType, Schedul_Analysis>(tasksN, x);

        UpdateTaskSetExecutionTime(tasksN, correctedX);
        double energyAfterOpt = EstimateEnergyTaskSet(tasksN.tasks_).sum();

        if (runMode == "compare")
            return energyAfterOpt / weightEnergy;
        else if (runMode == "normal")
        {
            UpdateTaskSetExecutionTime(tasksN, GetParameterVD<double>(tasksN, "executionTimeOrg"));
            double initialEnergyCost = EstimateEnergyTaskSet(tasksN.tasks_).sum();
            return energyAfterOpt / initialEnergyCost;
        }
        else
        {
            CoutError("Unrecognized runMode!!");
            return 0;
        }
    }
} // namespace rt_num_opt