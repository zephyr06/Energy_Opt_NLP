
#pragma once
#include <chrono>
#include <string>
#include <utility>
#include <numeric>
#include <CppUnitLite/TestHarness.h>
#include "Parameters.h"
#include "Optimize.h"
#include "ReadControlCases.h"
#include "CoeffFactor.h"
#include "RTAFactor.h"
#include "InequalifyFactor.h"
#include "ControlFactorGraphUtils.h"
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

    static void FindEliminatedVariables(TaskSet &tasks, std::vector<bool> &maskForElimination, double disturb = 1e0)
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
        if (debugMode == 1)
        {
            std::lock_guard<std::mutex> lock(mtx);
            for (auto a : maskForElimination)
                cout << a << ", ";
            cout << endl;
        }
    }
};