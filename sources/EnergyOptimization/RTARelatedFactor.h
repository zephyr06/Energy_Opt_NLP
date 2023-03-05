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
#include "sources/EnergyOptimization/FGEnergyOptUtils.h"
#include "sources/RTA/RTA_BASE.h"
#include "sources/Utils/GlobalVariables.h"  // EliminationRecord
#include "sources/Utils/MultiKeyFactor.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/utils.h"

namespace rt_num_opt {

template <class TaskSetType, class Schedul_Analysis>
class RTARelatedFactor : public gtsam::NoiseModelFactor {
   public:
    TaskSetType tasks;
    std::vector<gtsam::Symbol> keyVec;
    LambdaMultiKey f_with_RTA;

    RTARelatedFactor(std::vector<gtsam::Symbol> &keyVec, TaskSetType &tasks,
                     gtsam::SharedNoiseModel model)
        : gtsam::NoiseModelFactor(model, keyVec), tasks(tasks), keyVec(keyVec) {
        f_with_RTA = [tasks](const gtsam::Values &x) {
            BeginTimer("f_with_RTA");
            VectorDynamic error = GenerateVectorDynamic(1);
            // TODO: consider removing this copy to run slightly faster
            TaskSetType tasksCurr = tasks;
            VectorDynamic execCurr = EnergyOptUtils::ExtractResults(x, tasks);
            UpdateTaskSetExecutionTime(tasksCurr, execCurr);
            Schedul_Analysis r(tasksCurr);
            if (r.CheckSchedulability(1 == debugMode)) {
                error(0) = 0;
            } else {
                error(0) = Barrier(-1e9);
            }
            EndTimer("f_with_RTA");
            return error;
        };
    }

    gtsam::Vector unwhitenedError(const gtsam::Values &x,
                                  boost::optional<std::vector<gtsam::Matrix> &>
                                      H = boost::none) const override {
        BeginTimer("RTARelatedFactor_unwhitenedError");
        if (H) {
            for (uint i = 0; i < keyVec.size(); i++) {
                if (exactJacobian) {
                    NormalErrorFunction1D f = [x, i,
                                               this](const VectorDynamic xi) {
                        gtsam::Symbol a = keyVec.at(i);
                        gtsam::Values xx = x;
                        xx.update(a, xi);
                        return f_with_RTA(xx);
                    };
                    (*H)[i] = NumericalDerivativeDynamic(
                        f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer);
                } else
                    (*H)[i] = GenerateVectorDynamic(1);
            }
        }

        EndTimer("RTARelatedFactor_unwhitenedError");
        return f_with_RTA(x);
    }
};

template <class TaskSetType, class Schedul_Analysis>
RTARelatedFactor<TaskSetType, Schedul_Analysis> GenerateRTARelatedFactor(
    TaskSetType &tasks, EliminationRecord &eliminationRecord) {
    std::vector<gtsam::Symbol> keys;
    keys.reserve(tasks.N);
    for (int i = 0; i < tasks.N; i++) {
        if (eliminationRecord[i].type == EliminationType::Not)
            keys.push_back(GenerateKey(i, "executionTime"));
    }
    auto model = gtsam::noiseModel::Isotropic::Sigma(
        1, noiseModelSigma / weightSchedulability);
    return RTARelatedFactor<TaskSetType, Schedul_Analysis>(keys, tasks, model);
}

}  // namespace rt_num_opt