
#pragma once

#include "sources/Utils/MultiKeyFactor.h"
#include "sources/MatrixConvenient.h"
#include "sources/Utils/FactorGraphUtils.h"

#include "sources/Utils/Parameters.h"

namespace rt_num_opt
{
    MultiKeyFactor GenerateLockFactor(const TaskSetNormal &tasks,
                                      int index)
    {
        std::vector<gtsam::Symbol> keys;
        keys.push_back(GenerateKey(index, "executionTime"));

        double c = tasks.tasks_.at(index).executionTime;
        LambdaMultiKey f = [c, index](const gtsam::Values &x)
        {
            VectorDynamic error = GenerateVectorDynamic(1);
            VectorDynamic executionTimeVecCurr = x.at<VectorDynamic>(GenerateKey(index, "executionTime"));
            error(0) = executionTimeVecCurr(0, 0) - c;

            return error;
        };
        auto model = gtsam::noiseModel::Constrained::All(1);
        return MultiKeyFactor(keys, f, gtsam::noiseModel::Constrained::All(1));
    }
} // namespace rt_num_opt