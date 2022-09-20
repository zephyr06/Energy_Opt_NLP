#pragma once
#include <iostream>
#include <chrono>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include "sources/Utils/helpifopt.h"

#include "sources/Utils/Parameters.h"
#include "sources/RTA/RTA_LL.h"
#include "sources/ControlOptimization/FactorGraphInManifold.h"

namespace rt_num_opt
{

    class ExVariablesControl : public ifopt::VariableSet
    {
    private:
        VectorDynamic var_;
        TaskSet tasks_;
        VectorDynamic upperBound_;

    public:
        ExVariablesControl(TaskSet &tasks) : ExVariablesControl(tasks, "var_set1"){};
        ExVariablesControl(TaskSet &tasks, const std::string &name) : VariableSet(tasks.size(), name)
        {
            // the initial values where the NLP starts iterating from
            tasks_ = tasks;
            upperBound_ = GetParameterVD<double>(tasks_, "period");
            if (debugMode == 1)
                std::cout << "UB: " << upperBound_ << std::endl;
            var_ = upperBound_;
        }

        void SetVariables(const VectorDynamic &x) override
        {
            var_ = x;
        };

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
                bounds.at(i) = ifopt::Bounds(tasks_[i].executionTime, upperBound_(i));
                // bounds.at(i) = ifopt::Bounds(upperBound_(i) - 100, upperBound_(i));
            }
            return bounds;
        }
    };

    double ControlObj(TaskSet &tasks, VectorDynamic &coeff, VectorDynamic periodVector)
    {
        UpdateTaskSetPeriod(tasks, periodVector);
        return FactorGraphInManifold::RealObj(tasks, coeff);
    }

    class ExCostControl : public ifopt::CostTerm
    {
    private:
        VectorDynamic var_;
        TaskSet tasks_;
        VectorDynamic coeff_;

    public:
        ExCostControl(TaskSet &tasks) : CostTerm("cost_term1"), tasks_(tasks)
        {
            CoutError("Never call this constructor without providing coeff vector!");
        }

        ExCostControl(TaskSet &tasks, VectorDynamic coeff) : CostTerm("cost_term1"), tasks_(tasks), coeff_(coeff) {}

        boost::function<gtsam::Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &periodVector)
        {
            double err = ControlObj(tasks_, coeff_, periodVector);
            return GenerateVectorDynamic1D(err);
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
                if (debugMode == 1)
                    std::cout << "Jacobian matrix: " << jac << std::endl;
            }
        }
    };

    template <class TaskSetType, class Schedul_Analysis>
    class ExConstraintControl : public ifopt::ConstraintSet
    {
    public:
        TaskSetType taskGeneral_;

        ExConstraintControl(TaskSetType &tasks) : ExConstraintControl(tasks, "constraint1") {}

        ExConstraintControl(TaskSetType &tasks, const std::string &name) : ConstraintSet(1, name), taskGeneral_(tasks) {}

        boost::function<gtsam::Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &periodVector)
        {
            TaskSetType taskT = taskGeneral_;
            UpdateTaskSetPeriod(taskT.tasks_, periodVector);
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

        VectorDynamic GetValues() const override
        {
            VectorDynamic g(GetRows());
            VectorDynamic x = GetVariables()->GetComponent("var_set1")->GetValues();
            return f(x);
        };

        VecBound GetBounds() const override
        {
            VecBound b(GetRows());
            b.at(0) = ifopt::Bounds(0, 0);
            return b;
        }

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
    VectorDynamic ClampControlResultBasedOnFeasibility(TaskSetType &tasks, VectorDynamic &x)
    {
        UpdateTaskSetPeriod(tasks, x);
        Schedul_Analysis r(tasks);
        if (!r.CheckSchedulability())
        {
            return GetParameterVD<double>(tasks, "periodOrg");
        }
        else
            return x;
    }

    template <class TaskSetType, class Schedul_Analysis>
    std::pair<VectorDynamic, double> OptimizeControlIfopt(TaskSetType &tasksN, VectorDynamic &coeff)
    {
        VectorDynamic x = OptimizeIfopt<TaskSetType, ExVariablesControl, ExConstraintControl<TaskSetType, Schedul_Analysis>, ExCostControl>(tasksN, coeff);

        VectorDynamic correctedX = ClampControlResultBasedOnFeasibility<TaskSetType, Schedul_Analysis>(tasksN, x);

        return std::make_pair(correctedX, ControlObj(tasksN.tasks_, coeff, correctedX));
    }
} // namespace rt_num_opt