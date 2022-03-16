
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

bool ContainFalse(std::vector<bool> &maskForElimination)
{
    for (auto x : maskForElimination)
    {
        if (x == false)
            return true;
    }
    return false;
}
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
        return make_pair(periods, RTALLVector(tasks));
    }

    class RTARelatedFactor : public NoiseModelFactor
    {
    public:
        TaskSet tasks;
        int index;
        VectorDynamic coeff;
        VectorDynamic rtaBase;
        int dimension;
        vector<Symbol> keyVec;
        LambdaMultiKey f_with_RTA;

        RTARelatedFactor(vector<Symbol> &keyVec, TaskSet &tasks, int index, VectorDynamic &coeff, VectorDynamic rtaBase,
                         SharedNoiseModel model) : NoiseModelFactor(model, keyVec), tasks(tasks), index(index), coeff(coeff), rtaBase(rtaBase), dimension(keyVec.size()), keyVec(keyVec)
        {
            f_with_RTA = [tasks, index, coeff, rtaBase](const Values &x)
            {
                BeginTimer(__func__);
                VectorDynamic error = GenerateVectorDynamic(2);
                TaskSet tasksCurr = tasks;
                UpdateTaskSetPeriod(tasksCurr, FactorGraphInManifold::ExtractResults(x, tasks).first);
                RTA_LL r(tasksCurr);
                double rta = r.RTA_Common_Warm(rtaBase(index), index);
                error(0) = rta * coeff[2 * index + 1];
                error(1) = HingeLoss(tasksCurr[index].period - rta);
                EndTimer(__func__);
                return error;
            };
        }
        /* no need to optimize if it contains no keys */
        bool active(const Values &c) const override
        {
            return keyVec.size() != 0;
        }
        Vector unwhitenedError(const Values &x,
                               boost::optional<std::vector<Matrix> &> H = boost::none) const override
        {
            BeginTimer(__func__);
            if (H)
            {
                for (int i = 0; i < dimension; i++)
                {
                    if (exactJacobian)
                    {
                        NormalErrorFunction1D f =
                            [x, i, this](const VectorDynamic xi)
                        {
                            Symbol a = keyVec.at(i);
                            Values xx = x;
                            xx.update(a, xi);
                            return f_with_RTA(xx);
                        };
                        (*H)[i] = NumericalDerivativeDynamic(f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer);
                    }

                    else
                        (*H)[i] = GenerateVectorDynamic(2);
                }
                if (debugMode == 1)
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    cout << Color::blue;
                    // PrintControlValues(x);
                    // x.print();
                    cout << Color::def;
                }
            }
            EndTimer(__func__);
            return f_with_RTA(x);
        }
    };

    static RTARelatedFactor
    GenerateRTARelatedFactor(std::vector<bool> maskForElimination, TaskSet &tasks, int index, VectorDynamic &coeff)
    {
        VectorDynamic rtaBase = RTALLVector(tasks);
        LambdaMultiKey f = [tasks, index, coeff, rtaBase](const Values &x)
        {
            BeginTimer(__func__);
            VectorDynamic error = GenerateVectorDynamic(2);
            TaskSet tasksCurr = tasks;
            UpdateTaskSetPeriod(tasksCurr, FactorGraphInManifold::ExtractResults(x, tasks).first);
            RTA_LL r(tasksCurr);
            double rta = r.RTA_Common_Warm(rtaBase(index), index);
            error(0) = rta * coeff[2 * index + 1];
            error(1) = HingeLoss(tasksCurr[index].period - rta);
            EndTimer(__func__);
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
        // return MultiKeyFactor(keys, f, model);
        return RTARelatedFactor(keys, tasks, index, coeff, rtaBase, model);
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
        BeginTimer(__func__);
        if (!ContainFalse(maskForElimination))
        {
            return;
        }
        bool whether_new_eliminate = false;
        while (!whether_new_eliminate && disturb < disturb_max)
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
                    if (!maskForElimination[i])
                        whether_new_eliminate = true;
                    maskForElimination[i] = true;
                }
                tasks[i].period += disturb;
            }
            if (!whether_new_eliminate)
                disturb *= disturb_step;
            if (debugMode == 1)
            {
                std::lock_guard<std::mutex> lock(mtx);
                for (auto a : maskForElimination)
                    cout << a << ", ";
                cout << endl;
            }
        }

        EndTimer(__func__);
    }
};