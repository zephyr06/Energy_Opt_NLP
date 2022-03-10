
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
    auto modelNormal = noiseModel::Isotropic::Sigma(1, noiseModelSigma);                            //
    auto modelPunishment = noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint); // smaller sigma means larger error
    for (uint i = 0; i < tasks.size(); i++)
    {

        // schedulability
        graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "response"),
                                                  min(tasks[i].period, tasks[i].deadline), modelNormal);
        // add CoeffFactor
        graph.emplace_shared<CoeffFactor>(GenerateControlKey(i, "response"),
                                          GenerateVectorDynamic1D(coeff(2 * i + 1, 0)), modelNormal);
        // add RTAFactor
        graph.add(GenerateTaskRTAFactor(maskForElimination, tasks, i));

        if (!maskForElimination[i])
        {
            // add CoeffFactor
            graph.emplace_shared<CoeffFactor>(GenerateControlKey(i, "period"),
                                              GenerateVectorDynamic1D(coeff(2 * i, 0)), modelPunishment);
            // add period min/max limits
            graph.emplace_shared<LargerThanFactor1D>(GenerateControlKey(i, "period"), 0, modelPunishment);
            graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "period"), periodMax, modelPunishment);
        }
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
        if (!maskForElimination[i])
        {
            initialEstimateFG.insert(GenerateControlKey(i, "period"),
                                     GenerateVectorDynamic1D(tasks[i].period));
        }

        initialEstimateFG.insert(GenerateControlKey(i, "response"),
                                 GenerateVectorDynamic1D(rta(i, 0)));
    }
    return initialEstimateFG;
}

double RealObj(TaskSet &tasks, VectorDynamic coeff)
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

pair<VectorDynamic, double> UnitOptimizationPeriod(TaskSet &tasks, VectorDynamic coeff,
                                                   std::vector<bool> &maskForElimination)
{
    NonlinearFactorGraph graph = BuildControlGraph(maskForElimination, tasks, coeff);

    // VectorDynamic initialEstimate = GenerateVectorDynamic(N).array() + tasks[0].period;
    // initialEstimate << 68.000000, 321, 400, 131, 308;
    Values initialEstimateFG = GenerateInitialFG(tasks, maskForElimination);

    Values result;
    if (optimizerType == 1)
    {
        DoglegParams params;
        // if (debugMode == 1)
        //     params.setVerbosityDL("VERBOSE");
        params.setDeltaInitial(deltaInitialDogleg);
        params.setRelativeErrorTol(relativeErrorTolerance);
        DoglegOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    else if (optimizerType == 2)
    {
        LevenbergMarquardtParams params;
        params.setlambdaInitial(initialLambda);
        // if (debugMode > 1 && debugMode < 5)
        params.setVerbosityLM("SUMMARY");
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }

    VectorDynamic optComp, rtaFromOpt;
    std::tie(optComp, rtaFromOpt) = ExtractResults(result, tasks);
    cout << endl;
    cout << Color::blue;
    cout << "After optimization, the period vector is " << endl
         << optComp << endl;
    cout << "After optimization, the rta vector is " << endl
         << rtaFromOpt << endl;
    cout << Color::def;
    cout << endl;
    cout << Color::blue;
    UpdateTaskSetPeriod(tasks, ExtractResults(initialEstimateFG, tasks).first);
    cout << "Before optimization, the total error is " << RealObj(tasks, coeff) << endl;
    UpdateTaskSetPeriod(tasks, optComp);
    cout << "The objective function is " << RealObj(tasks, coeff) << endl;
    cout << Color::def;

    double eeee = graph.error(result);

    return make_pair(optComp, RealObj(tasks, coeff));
}
void FindEliminatedVariables(TaskSet &tasks, std::vector<bool> &maskForElimination, double disturb = 1e0)
{
    RTA_LL r(tasks);
    VectorDynamic rtaBase = r.ResponseTimeOfTaskSet();
    for (uint i = 0; i < tasks.size(); i++)
    {
        tasks[i].period -= disturb;
        RTA_LL r1(tasks);
        VectorDynamic rtaCurr = r1.ResponseTimeOfTaskSet();
        if ((rtaBase - rtaCurr).array().abs().maxCoeff() >= disturb)
        // TODO: more analytic way
        {
            maskForElimination[i] = true;
        }
        tasks[i].period += disturb;
    }
    for (auto a : maskForElimination)
        cout << a << ", ";
    cout << endl;
}

pair<VectorDynamic, double> OptimizeTaskSetIterativeWeight(TaskSet &tasks, VectorDynamic coeff,
                                                           std::vector<bool> &maskForElimination)
{
    RTA_LL rr(tasks);
    if (!rr.CheckSchedulability())
    {
        cout << "The task set is not schedulable!" << endl;
        return make_pair(GetParameterVD<double>(tasks, "period"), 1e30);
    }
    VectorDynamic periodRes;
    double err;
    for (double weight = weightSchedulabilityMax; weight >= weightSchedulabilityMin;
         weight /= weightSchedulabilityStep)
    {
        weightSchedulability = weight;
        std::tie(periodRes, err) = UnitOptimizationPeriod(tasks, coeff, maskForElimination);
        VectorDynamic periodPrev = GetParameterVD<double>(tasks, "period");
        UpdateTaskSetPeriod(tasks, periodRes);
        RTA_LL r(tasks);
        if (!r.CheckSchedulability())
        {
            UpdateTaskSetPeriod(tasks, periodPrev);
            return make_pair(periodPrev, err);
        }
        else
        {
            std::lock_guard<std::mutex> lock(mtx);
            cout << Color::blue << "After one iterate on updating weight parameter,\
             the periods remain schedulable and are"
                 << endl
                 << periodRes << endl;
        }
    }

    return make_pair(periodRes, err);
}

VectorDynamic OptimizeTaskSetIterative(TaskSet &tasks, VectorDynamic coeff,
                                       std::vector<bool> &maskForElimination)
{
    VectorDynamic periodResCurr, periodResPrev;
    double errPrev = 1e30;
    double errCurr = RealObj(tasks, coeff);
    while (errCurr < errPrev)
    {
        // store prev result
        errPrev = errCurr;
        periodResPrev = GetParameterVD<double>(tasks, "period");

        FindEliminatedVariables(tasks, maskForElimination);
        double err;
        std::tie(periodResCurr, err) = OptimizeTaskSetIterativeWeight(tasks, coeff, maskForElimination);
        UpdateTaskSetPeriod(tasks, periodResCurr);
        errCurr = RealObj(tasks, coeff);
    }

    return periodResPrev;
}