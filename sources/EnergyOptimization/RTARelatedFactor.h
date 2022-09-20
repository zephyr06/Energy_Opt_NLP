/**
 * @file RTARelatedFactor.h
 * @author Sen Wang
 * @brief RTA related factor for energy optimization
 * @version 0.1
 * @date 2022-09-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "gtsam/linear/NoiseModel.h"

#include "sources/RTA/RTA_BASE.h"
#include "sources/EnergyOptimization/FGEnergyOptUtils.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/utils.h"

#include "sources/Utils/GlobalVariables.h" // EliminationRecord

namespace rt_num_opt
{

    template <class TaskSetType, class Schedul_Analysis>
    class RTARelatedFactor : public gtsam::NoiseModelFactor
    {
    public:
        TaskSetType tasks;
        std::vector<gtsam::Symbol> keyVec;
        LambdaMultiKey f_with_RTA;

        RTARelatedFactor(std::vector<gtsam::Symbol> &keyVec, TaskSetType &tasks,
                         gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor(model, keyVec), tasks(tasks), keyVec(keyVec)
        {
            f_with_RTA = [tasks](const gtsam::Values &x)
            {
                BeginTimer("f_with_RTA");
                VectorDynamic error = GenerateVectorDynamic(1);
                // TODO: consider removing this copy to run faster
                TaskSetType tasksCurr = tasks;
                VectorDynamic execCurr = EnergyOptUtils::ExtractResults(x, tasks);
                UpdateTaskSetExecutionTime(tasksCurr, execCurr);
                Schedul_Analysis r(tasksCurr);
                if (r.CheckSchedulability(1 == debugMode))
                {
                    error(0) = 0;
                }
                else
                {
                    error(0) = Barrier(-1e9);
                }
                EndTimer("f_with_RTA");
                return error;
            };
        }

        gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                      boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const override
        {
            BeginTimer("RTARelatedFactor_unwhitenedError");
            if (H)
            {
                for (uint i = 0; i < keyVec.size(); i++)
                {
                    if (exactJacobian)
                    {
                        NormalErrorFunction1D f =
                            [x, i, this](const VectorDynamic xi)
                        {
                            gtsam::Symbol a = keyVec.at(i);
                            gtsam::Values xx = x;
                            xx.update(a, xi);
                            return f_with_RTA(xx);
                        };
                        (*H)[i] = NumericalDerivativeDynamic(f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer);
                    }
                    else
                        (*H)[i] = GenerateVectorDynamic(1);
                }
            }

            EndTimer("RTARelatedFactor_unwhitenedError");
            return f_with_RTA(x);
        }
    };

    template <class TaskSetType, class Schedul_Analysis>
    RTARelatedFactor<TaskSetType, Schedul_Analysis> GenerateRTARelatedFactor(TaskSetType &tasks, EliminationRecord &eliminationRecord)
    {

        std::vector<gtsam::Symbol> keys;
        keys.reserve(tasks.N);
        for (int i = 0; i < tasks.N; i++)
        {
            if (eliminationRecord[i].type == EliminationType::Not)
                keys.push_back(GenerateKey(i, "executionTime"));
        }
        auto model = gtsam::noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightSchedulability);
        return RTARelatedFactor<TaskSetType, Schedul_Analysis>(keys, tasks, model);
    }

    // template <class TaskSetType, class Schedul_Analysis>
    // class RTARelatedFactor : public gtsam::NoiseModelFactor
    // {
    // public:
    //     TaskSetType tasks;
    //     int index;
    //     VectorDynamic rtaBase;
    //     int dimension;
    //     std::vector<gtsam::Symbol> keyVec;
    //     LambdaMultiKey f_with_RTA;

    //     RTARelatedFactor(std::vector<gtsam::Symbol> &keyVec, TaskSetType &tasks, int index, VectorDynamic rtaBase,
    //                      gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor(model, keyVec), tasks(tasks), index(index), rtaBase(rtaBase), dimension(keyVec.size()), keyVec(keyVec)
    //     {
    //         f_with_RTA = [tasks, index, rtaBase](const gtsam::Values &x)
    //         {
    //             BeginTimer("f_with_RTA");
    //             VectorDynamic error = GenerateVectorDynamic(1);
    //             // TODO: remove this copy to run faster
    //             TaskSetType tasksCurr = tasks;
    //             UpdateTaskSetExecutionTime(tasksCurr, EnergyOptUtils::ExtractResults(x, tasks));
    //             Schedul_Analysis r(tasksCurr);
    //             double rta = r.RTA_Common_Warm(rtaBase(index), index);
    //             error(0) = Barrier(min(tasksCurr[index].period, tasksCurr[index].deadline) - rta);

    //             eliminationRecordGlobal.AdjustEliminationError(error(0), index, EliminationType::RTA);
    //             EndTimer("f_with_RTA");
    //             return error;
    //         };
    //     }

    //     RTARelatedFactor(std::vector<gtsam::Symbol> &keyVec, TaskSetType &tasks, int index,
    //                      gtsam::SharedNoiseModel model)
    //     {
    //         rtaBase = GetParameterVD<double>(tasks, "executionTime");
    //         RTARelatedFactor(keyVec, tasks, index, rtaBase, model);
    //     }
    //     /* no need to optimize if it contains no keys */
    //     bool active(const gtsam::Values &c) const override
    //     {
    //         return keyVec.size() != 0;
    //     }
    //     gtsam::Vector unwhitenedError(const gtsam::Values &x,
    //                                   boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const override
    //     {
    //         BeginTimer("RTARelatedFactor_unwhitenedError");
    //         if (H)
    //         {
    //             for (int i = 0; i < dimension; i++)
    //             {
    //                 if (exactJacobian)
    //                 {
    //                     NormalErrorFunction1D f =
    //                         [x, i, this](const VectorDynamic xi)
    //                     {
    //                         gtsam::Symbol a = keyVec.at(i);
    //                         gtsam::Values xx = x;
    //                         xx.update(a, xi);
    //                         return f_with_RTA(xx);
    //                     };
    //                     (*H)[i] = NumericalDerivativeDynamic(f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer);
    //                 }
    //                 else
    //                     (*H)[i] = GenerateVectorDynamic(1);
    //             }
    //         }

    //         EndTimer("RTARelatedFactor_unwhitenedError");
    //         return f_with_RTA(x);
    //     }
    // };

    // template <class TaskSetType, class Schedul_Analysis>
    // RTARelatedFactor<TaskSetType, Schedul_Analysis> GenerateRTARelatedFactor(TaskSetType &tasks, int index)
    // {

    //     std::vector<gtsam::Symbol> keys;
    //     keys.reserve(index);
    //     for (int i = 0; i <= index; i++)
    //     {
    //         keys.push_back(GenerateKey(i, "executionTime"));
    //     }

    //     VectorDynamic sigma = GenerateVectorDynamic(1);
    //     sigma << noiseModelSigma / weightSchedulability;
    //     auto model = gtsam::noiseModel::Diagonal::Sigmas(sigma);
    //     return RTARelatedFactor<TaskSetType, Schedul_Analysis>(keys, tasks, index, model);
    // }

} // namespace rt_num_opt