#pragma once
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include <boost/optional/optional.hpp>
#include <boost/optional/optional_io.hpp>

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

    template <class TaskSetType, class ExVariablesT, class ExConstraintT, class ExCostT>
    Eigen::VectorXd OptimizeIfopt(TaskSetType &tasks, boost::optional<VectorDynamic> otherParameters = boost::none)
    {
        // 1. define the problem
        ifopt::Problem nlp;
        nlp.AddVariableSet(std::make_shared<ExVariablesT>(tasks.tasks_));
        nlp.AddConstraintSet(std::make_shared<ExConstraintT>(tasks));
        if (otherParameters != boost::none) // this 'if' statement distinguishes control and energy cases
            nlp.AddCostSet(std::make_shared<ExCostT>(tasks.tasks_, otherParameters.get()));
        else
            nlp.AddCostSet(std::make_shared<ExCostT>(tasks.tasks_));
        if (debugMode == 1)
            nlp.PrintCurrent();

        // 2. choose solver and options
        ifopt::IpoptSolver ipopt;
        ipopt.SetOption("linear_solver", "mumps");
        ipopt.SetOption("jacobian_approximation", "exact");
        ipopt.SetOption("max_cpu_time", 600.0); // time-out after 600 seconds

        // 3 . solve
        ipopt.Solve(nlp);
        Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();

        double y[x.rows()];
        Eigen2Array(x, y);
        if (debugMode == 1)
        {
            std::cout << "Obj: " << nlp.EvaluateCostFunction(y) << std::endl;
            std::cout << "Optimal variable found: " << x << std::endl;
        }
        return x;
    }

}
