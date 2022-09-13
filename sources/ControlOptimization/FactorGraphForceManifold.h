
#pragma once
#include <chrono>
#include <string>
#include <utility>
#include <numeric>

#include <CppUnitLite/TestHarness.h>

#include "sources/Utils/Parameters.h"
#include "sources/EnergyOptimization/Optimize.h"
#include "sources/ControlOptimization/ReadControlCases.h"
#include "sources/ControlOptimization/CoeffFactor.h"
#include "sources/Utils/MultiKeyFactor.h"
#include "sources/Utils/InequalifyFactor.h"
#include "sources/Utils/FactorGraphUtils.h"
#include "sources/ControlOptimization/RTAFactor.h"
namespace rt_num_opt
{

    /**
     * @brief this factor graph uses RTA as an equality constraint rather than eliminating it;
     * a major difference in implementation is maskForElimination, which has length of 2 * n rather than n to include the extra n 'r_i' variables
     *
     */
    struct FactorGraphForceManifold
    {
        /**
         * @brief
         *
         * @param result
         * @param tasks
         * @return pair<VectorDynamic, VectorDynamic> periods, rta; rta is raw rta, which could include -1 (eliminated before)
         */
        // Issue in this function! not able to extract rta because of consistency
        static VectorDynamic ExtractResults(const gtsam::Values &result, TaskSet &tasks)
        {
            VectorDynamic periods = GenerateVectorDynamic(tasks.size());
            VectorDynamic rta = GenerateVectorDynamic(tasks.size());
            for (uint i = 0; i < tasks.size(); i++)
            {
                if (result.exists(GenerateKey(i, "period")))
                {
                    periods(i, 0) = result.at<VectorDynamic>(GenerateKey(i, "period"))(0, 0);
                }
                else
                {
                    periods(i, 0) = tasks[i].period;
                }
                if (result.exists(GenerateKey(i, "response")))
                {
                    rta(i, 0) = result.at<VectorDynamic>(GenerateKey(i, "response"))(0, 0);
                }
                else
                {
                    rta(i, 0) = -1;
                }
            }
            return periods;
        }
        static InequalityFactor2D GenerateSchedulabilityFactor(std::vector<bool> maskForElimination, TaskSet &tasks, int index)
        {
            auto modelPunishmentSoft1 = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint);
            NormalErrorFunction2D DBF2D =
                [](VectorDynamic x1, VectorDynamic x2)
            {
                // x1 <= x2
                // if (x2(0, 0) < x1(0, 0))
                //     int a = 1;
                return GenerateVectorDynamic1D(HingeLoss((x2 - x1)(0, 0)) * weightHardConstraint);
            };
            // this factor is explained as: r_i <= T_i
            return InequalityFactor2D(GenerateKey(index, "response"),
                                      GenerateKey(index, "period"), DBF2D, modelPunishmentSoft1);
        }
        static gtsam::NonlinearFactorGraph BuildControlGraph(std::vector<bool> maskForElimination, TaskSet tasks, VectorDynamic &coeff)
        {
            gtsam::NonlinearFactorGraph graph;
            double periodMax = GetParameterVD<double>(tasks, "executionTime").sum() * 5;
            auto modelNormal = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma);
            auto modelPunishmentSoft1 = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightHardConstraint);
            RTA_LL r(tasks);
            VectorDynamic rtaBase = r.ResponseTimeOfTaskSet();
            for (uint i = 0; i < tasks.size(); i++)
            {
                // add RTAFactor
                graph.add(GenerateTaskRTAFactor(maskForElimination, tasks, i, rtaBase));

                if (!maskForElimination[i + 5])
                {
                    graph.emplace_shared<LargerThanFactor1D>(GenerateKey(i, "response"), tasks[i].executionTime, modelPunishmentSoft1);
                    // add CoeffFactor
                    graph.emplace_shared<CoeffFactor>(GenerateKey(i, "response"),
                                                      GenerateVectorDynamic1D(coeff(2 * i + 1, 0)), modelNormal);
                }
                if (!maskForElimination[i])
                {
                    // add CoeffFactor
                    graph.emplace_shared<CoeffFactor>(GenerateKey(i, "period"),
                                                      GenerateVectorDynamic1D(coeff(2 * i, 0)), modelNormal);
                    // add period min/max limits
                    graph.emplace_shared<LargerThanFactor1D>(GenerateKey(i, "period"), 0, modelPunishmentSoft1);
                    graph.emplace_shared<SmallerThanFactor1D>(GenerateKey(i, "period"), periodMax, modelPunishmentSoft1);
                }

                // schedulability factor
                if (!maskForElimination[i]) // T_i is an variable
                {
                    if (!maskForElimination[i + 5]) // r_i is an variable
                        graph.add(GenerateSchedulabilityFactor(maskForElimination, tasks, i));
                    else
                        graph.emplace_shared<LargerThanFactor1D>(GenerateKey(i, "period"),
                                                                 rtaBase(i), modelPunishmentSoft1);
                }
                else // T_i is eliminated
                {
                    if (!maskForElimination[i + 5]) // r_i is an variable
                        graph.emplace_shared<SmallerThanFactor1D>(GenerateKey(i, "response"),
                                                                  min(tasks[i].period, tasks[i].deadline), modelPunishmentSoft1);
                }
            }
            return graph;
        }

        static gtsam::Values GenerateInitialFG(TaskSet tasks, std::vector<bool> &maskForElimination)
        {
            RTA_LL r(tasks);
            auto rta = r.ResponseTimeOfTaskSet();
            gtsam::Values initialEstimateFG;
            for (uint i = 0; i < tasks.size(); i++)
            {
                if (!maskForElimination[i])
                {
                    initialEstimateFG.insert(GenerateKey(i, "period"),
                                             GenerateVectorDynamic1D(tasks[i].period));
                }
                if (!maskForElimination[i + 5])
                {
                    initialEstimateFG.insert(GenerateKey(i, "response"),
                                             GenerateVectorDynamic1D(rta(i)));
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

                for (uint j = 0; j < i; j++)
                {
                    if (QuotientDouble(rtaBase(i), tasks[j].period) < disturb)
                    {
                        maskForElimination[i + 5] = true;
                    }
                }
            }
            if (debugMode == 1)
            {
                std::lock_guard<std::mutex> lock(mtx);
                for (auto a : maskForElimination)
                    std::cout << a << ", ";
                std::cout << std::endl;
            }
        }

        static double RealObj(TaskSet &tasks, VectorDynamic coeff)
        {
            BeginTimer(__func__);
            double res = 0;
            RTA_LL r(tasks);
            VectorDynamic rta = r.ResponseTimeOfTaskSet();
            for (uint i = 0; i < tasks.size(); i++)
            {
                res += coeff(i * 2, 0) * tasks[i].period;
                res += coeff(i * 2 + 1, 0) * rta(i, 0);
            }
            EndTimer(__func__);
            return res;
        }
    };
} // namespace rt_num_opt