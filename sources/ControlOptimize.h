
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

struct FactorGraphForceManifold
{
    static pair<VectorDynamic, VectorDynamic> ExtractResults(const Values &result, TaskSet &tasks)
    {
        VectorDynamic periods = GenerateVectorDynamic(tasks.size());
        VectorDynamic rta = GenerateVectorDynamic(tasks.size());
        for (uint i = 0; i < tasks.size(); i++)
        {
            if (result.exists(GenerateControlKey(i, "period")))
            {
                periods(i, 0) = result.at<VectorDynamic>(GenerateControlKey(i, "period"))(0, 0);
            }
            else
            {
                periods(i, 0) = tasks[i].period;
            }
            rta(i, 0) = result.at<VectorDynamic>(GenerateControlKey(i, "response"))(0, 0);
        }
        return make_pair(periods, rta);
    }
    static InequalityFactor2D GenerateSchedulabilityFactor(std::vector<bool> maskForElimination, TaskSet &tasks, int index)
    {
        auto modelPunishmentSoft1 = noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint);
        NormalErrorFunction2D DBF2D =
            [](VectorDynamic x1, VectorDynamic x2)
        {
            // x1 <= x2
            if (x2(0, 0) < x1(0, 0))
                int a = 1;
            return GenerateVectorDynamic1D(HingeLoss((x2 - x1)(0, 0)) * weightHardConstraint);
        };
        // this factor is explained as: r_i <= T_i
        return InequalityFactor2D(GenerateControlKey(index, "response"),
                                  GenerateControlKey(index, "period"), DBF2D, modelPunishmentSoft1);
    }
    static NonlinearFactorGraph BuildControlGraph(std::vector<bool> maskForElimination, TaskSet tasks, VectorDynamic &coeff)
    {
        NonlinearFactorGraph graph;
        double periodMax = GetParameterVD<double>(tasks, "executionTime").sum() * 5;
        auto modelNormal = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
        auto modelPunishmentSoft1 = noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint);

        for (uint i = 0; i < tasks.size(); i++)
        {

            // add RTAFactor
            graph.add(GenerateTaskRTAFactor(maskForElimination, tasks, i));
            graph.emplace_shared<LargerThanFactor1D>(GenerateControlKey(i, "response"), tasks[i].executionTime, modelPunishmentSoft1);
            if (!maskForElimination[i])
            {
                // add CoeffFactor
                graph.emplace_shared<CoeffFactor>(GenerateControlKey(i, "response"),
                                                  GenerateVectorDynamic1D(coeff(2 * i + 1, 0)), modelNormal);
                // add CoeffFactor
                graph.emplace_shared<CoeffFactor>(GenerateControlKey(i, "period"),
                                                  GenerateVectorDynamic1D(coeff(2 * i, 0)), modelNormal);
                // add period min/max limits
                graph.emplace_shared<LargerThanFactor1D>(GenerateControlKey(i, "period"), 0, modelPunishmentSoft1);
                graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "period"), periodMax, modelPunishmentSoft1);
                // schedulability
                graph.add(GenerateSchedulabilityFactor(maskForElimination, tasks, i));
            }
            else
            {
                // schedulability
                graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "response"),
                                                          min(tasks[i].period, tasks[i].deadline), modelPunishmentSoft1);
            }
        }
        return graph;
    }

    static Values GenerateInitialFG(TaskSet tasks, std::vector<bool> &maskForElimination)
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
};

struct FactorGraphInManifold
{
    static pair<VectorDynamic, VectorDynamic> ExtractResults(const Values &result, TaskSet tasks)
    {
        VectorDynamic periods = GetParameterVD<double>(tasks, "period");
        for (uint i = 0; i < tasks.size(); i++)
        {
            if (result.exists(GenerateControlKey(i, "period")))
            {
                periods(i, 0) = result.at<VectorDynamic>(GenerateControlKey(i, "period"))(0, 0);
            }
        }
        UpdateTaskSetPeriod(tasks, periods);
        RTA_LL r(tasks);
        return make_pair(periods, r.ResponseTimeOfTaskSet());
    }

    static MultiKeyFactor GenerateRTARelatedFactor(std::vector<bool> maskForElimination, TaskSet &tasks, int index, VectorDynamic &coeff)
    {
        LambdaMultiKey f = [tasks, index, coeff](const Values &x)
        {
            VectorDynamic error = GenerateVectorDynamic(2);
            TaskSet tasksCurr = tasks;
            UpdateTaskSetPeriod(tasksCurr, FactorGraphInManifold::ExtractResults(x, tasks).first);
            RTA_LL r(tasksCurr);
            double rta = r.RTA_Common_Warm(tasksCurr[index].executionTime, index);
            error(0) = rta * coeff[2 * index + 1];
            error(1) = HingeLoss(tasksCurr[index].period - rta);

            return error;
        };

        std::vector<gtsam::Symbol> keys;
        keys.reserve(index);
        for (int i = 0; i <= index; i++)
        {
            if (!maskForElimination[i])
            {
                keys.push_back(GenerateControlKey(i, "period"));
            }
        }

        VectorDynamic sigma = GenerateVectorDynamic(2);
        sigma << noiseModelSigma, noiseModelSigma / weightSchedulability;
        auto model = noiseModel::Diagonal::Sigmas(sigma);
        return MultiKeyFactor(keys, f, model);
    }

    static NonlinearFactorGraph BuildControlGraph(std::vector<bool> maskForElimination, TaskSet tasks, VectorDynamic &coeff)
    {
        NonlinearFactorGraph graph;
        double periodMax = GetParameterVD<double>(tasks, "executionTime").sum() * 5;
        auto modelNormal = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
        auto modelPunishmentSoft1 = noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint);

        for (uint i = 0; i < tasks.size(); i++)
        {
            if (!maskForElimination[i])
            {
                // add CoeffFactor
                graph.emplace_shared<CoeffFactor>(GenerateControlKey(i, "period"),
                                                  GenerateVectorDynamic1D(coeff(2 * i, 0)), modelNormal);
                // add period min/max limits
                graph.emplace_shared<LargerThanFactor1D>(GenerateControlKey(i, "period"), tasks[i].executionTime, modelPunishmentSoft1);
                graph.emplace_shared<SmallerThanFactor1D>(GenerateControlKey(i, "period"), periodMax, modelPunishmentSoft1);
            }
            auto factor = GenerateRTARelatedFactor(maskForElimination, tasks, i, coeff);

            graph.add(factor);
        }
        return graph;
    }

    static Values GenerateInitialFG(TaskSet tasks, std::vector<bool> &maskForElimination)
    {
        Values initialEstimateFG;
        for (uint i = 0; i < tasks.size(); i++)
        {
            if (!maskForElimination[i])
            {
                initialEstimateFG.insert(GenerateControlKey(i, "period"),
                                         GenerateVectorDynamic1D(tasks[i].period));
            }
        }
        return initialEstimateFG;
    }
};
template <typename FactorGraphType>
pair<VectorDynamic, double> UnitOptimizationPeriod(TaskSet &tasks, VectorDynamic coeff,
                                                   std::vector<bool> &maskForElimination)
{
    NonlinearFactorGraph graph = FactorGraphType::BuildControlGraph(maskForElimination, tasks, coeff);

    // VectorDynamic initialEstimate = GenerateVectorDynamic(N).array() + tasks[0].period;
    // initialEstimate << 68.000000, 321, 400, 131, 308;
    Values initialEstimateFG = FactorGraphType::GenerateInitialFG(tasks, maskForElimination);

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

    VectorDynamic optComp, rtaFromOpt;
    std::tie(optComp, rtaFromOpt) = FactorGraphType::ExtractResults(result, tasks);
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

    // double eeee = graph.error(result);

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
            std::lock_guard<std::mutex> lock(mtx);
            cout << Color::blue << "After one iterate on updating weight parameter,\
             the periods become unschedulable and are"
                 << endl
                 << periodRes << endl;
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

        FindEliminatedVariables(tasks, maskForElimination);
        double err;
        std::tie(periodResCurr, err) = OptimizeTaskSetIterativeWeight<FactorGraphType>(tasks, coeff, maskForElimination);
        UpdateTaskSetPeriod(tasks, periodResCurr);
        errCurr = RealObj(tasks, coeff);
    }
    UpdateTaskSetPeriod(tasks, periodResPrev);
    return make_pair(periodResPrev, errPrev);
}