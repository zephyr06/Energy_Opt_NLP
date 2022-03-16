
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
#include "FactorGraphForceManifold.h"
#include "FactorGraphInManifold.h"

template <typename FactorGraphType>
pair<VectorDynamic, double> UnitOptimizationPeriod(TaskSet &tasks, VectorDynamic coeff,
                                                   std::vector<bool> &maskForElimination)
{
    NonlinearFactorGraph graph = FactorGraphType::BuildControlGraph(maskForElimination, tasks, coeff);

    // VectorDynamic initialEstimate = GenerateVectorDynamic(N).array() + tasks[0].period;
    // initialEstimate << 68.000000, 321, 400, 131, 308;
    Values initialEstimateFG = FactorGraphType::GenerateInitialFG(tasks, maskForElimination);
    if (debugMode == 1)
    {
        cout << Color::green;
        // std::lock_guard<std::mutex> lock(mtx);
        auto sth = graph.linearize(initialEstimateFG)->jacobian();
        MatrixDynamic jacobianCurr = sth.first;
        std::cout << "Current Jacobian matrix:" << endl;
        std::cout << jacobianCurr << endl;
        std::cout << "Current b vector: " << endl;
        std::cout << sth.second << endl;
        cout << Color::def << endl;
    }

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
        params.setVerbosityLM(verbosityLM);
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }

    VectorDynamic optComp, rtaFromOpt; // rtaFromOpt can only be used for 'cout'
    std::tie(optComp, rtaFromOpt) = FactorGraphType::ExtractResults(result, tasks);
    if (debugMode == 1)
    {
        cout << endl;
        cout << Color::blue;
        cout << "After optimization, the period vector is " << endl
             << optComp << endl;
        cout << "After optimization, the rta vector is " << endl
             << rtaFromOpt << endl;
        cout << "The graph error is " << graph.error(result) << endl;
        cout << Color::def;
        cout << endl;
        cout << Color::blue;
        UpdateTaskSetPeriod(tasks, FactorGraphType::ExtractResults(initialEstimateFG, tasks).first);
        cout << "Before optimization, the total error is " << RealObj(tasks, coeff) << endl;
        UpdateTaskSetPeriod(tasks, optComp);
        cout << "The objective function is " << RealObj(tasks, coeff) << endl;
        cout << Color::def;
    }

    UpdateTaskSetPeriod(tasks, optComp);
    return make_pair(optComp, RealObj(tasks, coeff));
}

template <typename FactorGraphType>
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
    for (double weight = weightSchedulabilityMin; weight <= weightSchedulabilityMax;
         weight *= weightSchedulabilityStep)
    {
        weightSchedulability = weight;
        std::tie(periodRes, err) = UnitOptimizationPeriod<FactorGraphType>(tasks, coeff, maskForElimination);
        VectorDynamic periodPrev = GetParameterVD<double>(tasks, "period");
        UpdateTaskSetPeriod(tasks, periodRes);
        RTA_LL r(tasks);
        if (!r.CheckSchedulability())
        {
            UpdateTaskSetPeriod(tasks, periodPrev);
            if (debugMode == 1)
            {
                std::lock_guard<std::mutex> lock(mtx);
                cout << Color::blue << "After one iterate on updating weight parameter,\
             the periods become unschedulable and are"
                     << endl
                     << periodRes << endl;
                cout << Color::def;
            }

            return make_pair(periodPrev, err);
        }
        else
        {
            if (debugMode == 1)
            {
                std::lock_guard<std::mutex> lock(mtx);
                cout << Color::blue << "After one iterate on updating weight parameter,\
             the periods remain schedulable and are"
                     << endl
                     << periodRes << endl;
                cout << Color::def;
            }
        }
    }

    return make_pair(periodRes, err);
}
template <typename FactorGraphType>
pair<VectorDynamic, double> OptimizeTaskSetIterative(TaskSet &tasks, VectorDynamic coeff,
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

        FactorGraphType::FindEliminatedVariables(tasks, maskForElimination);
        double err;
        std::tie(periodResCurr, err) = OptimizeTaskSetIterativeWeight<FactorGraphType>(tasks, coeff, maskForElimination);
        UpdateTaskSetPeriod(tasks, periodResCurr);
        errCurr = RealObj(tasks, coeff);
    }
    UpdateTaskSetPeriod(tasks, periodResPrev);
    return make_pair(periodResPrev, errPrev);
}