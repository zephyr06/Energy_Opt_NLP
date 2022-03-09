
#pragma once
#include <chrono>
#include <string>
#include <utility>
#include <numeric>
#include <CppUnitLite/TestHarness.h>
#include "../sources/Parameters.h"
#include "../sources/Optimize.h"
#include "../sources/ReadControlCases.h"
#include "../sources/CoeffFactor.h"
#include "../sources/RTAFactor.h"
#include "../sources/InequalifyFactor.h"

#include "ControlFactorGraphUtils.h"

NonlinearFactorGraph BuildControlGraph(std::vector<bool> maskForElimination, TaskSet tasks, VectorDynamic &coeff)
{
    NonlinearFactorGraph graph;
    double periodMax = GetParameterVD<double>(tasks, "executionTime").sum() * 5;
    auto model1 = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    for (uint i = 0; i < tasks.size(); i++)
    {
        if (maskForElimination[i])
            continue;
        // add CoeffFactor
        graph.emplace_shared<CoeffFactor>(GenerateControlKey(i, "period"), GenerateVectorDynamic1D(coeff(2 * i, 0)), model1);
        graph.emplace_shared<CoeffFactor>(GenerateControlKey(i, "response"), GenerateVectorDynamic1D(coeff(2 * i + 1, 0)), model1);

        // add RTAFactor
        graph.add(GenerateTaskRTAFactor(maskForElimination, tasks, i));

        // add period min/max limits
        graph.emplace_shared<LargerThanFactor1D>(GenerateControlKey(i, "period"), 0, model1);
        graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "period"), periodMax, model1);

        // schedulability
        graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "response"), min(tasks[i].period, tasks[i].deadline), model1);
    }
    return graph;
}

Values GenerateInitialFG(TaskSet tasks, std::vector<bool> &maskForElimination)
{
    RTA_LL r(tasks);
    auto rta = r.ResponseTimeOfTaskSet();
    Values initialEstimateFG;
    for (uint i = 0; i < tasks.size(); i++)
    {
        if (maskForElimination[i])
            continue;
        initialEstimateFG.insert(GenerateControlKey(i, "period"), GenerateVectorDynamic1D(tasks[i].period));
        initialEstimateFG.insert(GenerateControlKey(i, "response"), GenerateVectorDynamic1D(rta(i, 0)));
    }
    return initialEstimateFG;
}

double realObj(TaskSet &tasks, VectorDynamic coeff)
{
    double res = 0;
    RTA_LL r(tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    for (uint i = 0; i < tasks.size(); i++)
    {
        res += coeff(i * 2, 0) * tasks[i].period;
        res += coeff(i * 2 + 1, 0) * rta(i, 0);
    }
    return res;
}