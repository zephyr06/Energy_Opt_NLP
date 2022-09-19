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
    public:
        // Every variable set has a name, here "var_set1". this allows the constraints
        // and costs to define values and Jacobians specifically w.r.t this variable set.
        ExVariablesEnergy() : ExVariablesEnergy("var_set1"){};
        ExVariablesEnergy(const std::string &name) : VariableSet(globalInfoIfopt.tasks.N, name)
        {
            // the initial values where the NLP starts iterating from
            var_ = globalInfoIfopt.variableGlobalIfoptInitial;
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
            // bounds.at(0) = ifopt::Bounds(-1.0, 1.0);
            // bounds.at(1) = ifopt::NoBound;

            for (int i = 0; i < var_.rows(); i++)
            {
                if (enableMaxComputationTimeRestrict)
                    bounds.at(i) = ifopt::Bounds(globalInfoIfopt.variableGlobalIfoptInitial(i), globalInfoIfopt.variableGlobalIfoptInitial(i) * MaxComputationTimeRestrict);
                else
                    bounds.at(i) = ifopt::Bounds(globalInfoIfopt.variableGlobalIfoptInitial(i), globalInfoIfopt.variableGlobalIfoptInitial(i) * 10000);
            }
            return bounds;
        }

    private:
        // double x0_, x1_;
        VectorDynamic var_;
    };
    class ExCostEnergy : public ifopt::CostTerm
    {
    public:
        ExCostEnergy() : ExCostEnergy("cost_term1") {}
        ExCostEnergy(const std::string &name) : CostTerm(name) {}

        boost::function<gtsam::Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &executionTimeVector)
        {
            TaskSet taskT = globalInfoIfopt.tasks.tasks_;
            UpdateTaskSetExecutionTime(taskT, executionTimeVector);
            double energy = EstimateEnergyTaskSet(taskT).sum();
            // std::cout << energy << std::endl;
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

    template <class ExVariablesEnergy, class ExConstraint, class ExCostEnergy>
    Eigen::VectorXd OptimizeIfopt()
    {
        // 1. define the problem
        ifopt::Problem nlp;
        nlp.AddVariableSet(std::make_shared<ExVariablesEnergy>());
        nlp.AddConstraintSet(std::make_shared<ExConstraint>());
        nlp.AddCostSet(std::make_shared<ExCostEnergy>());
        if (debugMode == 1)
            nlp.PrintCurrent();

        // 2. choose solver and options
        ifopt::IpoptSolver ipopt;
        ipopt.SetOption("linear_solver", "mumps");
        ipopt.SetOption("jacobian_approximation", "exact");

        // 3 . solve
        ipopt.Solve(nlp);
        Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
        // double xArray[2] = {3, 1};
        double y[x.rows()];
        Eigen2Array(x, y);
        std::cout << "Obj: " << nlp.EvaluateCostFunction(y) << std::endl;
        std::cout << "Optimal variable found: " << x << std::endl;
        return x;
    }

    template <class TaskSetType, class Schedul_Analysis>
    VectorDynamic ClampResultBasedOnFeasibility(TaskSetType &tasks, VectorDynamic &x)
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
        InitializeGlobalInfoIfopt(tasksN);
        VectorDynamic x = OptimizeIfopt<ExVariablesEnergy, ExConstraint<TaskSetType, Schedul_Analysis>, ExCostEnergy>();
        VectorDynamic correctedX = ClampResultBasedOnFeasibility<TaskSetType, Schedul_Analysis>(tasksN, x);

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