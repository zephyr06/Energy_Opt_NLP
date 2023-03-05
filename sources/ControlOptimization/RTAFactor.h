#pragma once

#include "sources/Utils/FactorGraphUtils.h"
#include "sources/Utils/MultiKeyFactor.h"

namespace rt_num_opt {
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
typedef long long int LLint;

/* convenient function only used for GenerateTaskRTAFactor function*/
double AtWithReplace(const gtsam::Values &x, gtsam::Symbol key,
                     double alternative) {
    if (x.exists(key))
        return x.at<VectorDynamic>(key)(0, 0);
    else
        return alternative;
};

MultiKeyFactor GenerateTaskRTAFactor(std::vector<bool> &maskForElimination,
                                     TaskSet &tasks, int index,
                                     VectorDynamic &rtaBase) {
    LambdaMultiKey f = [tasks, index, rtaBase](const gtsam::Values &x) {
        double error = AtWithReplace(x, GenerateKey(index, "response"),
                                     rtaBase(index, 0)) -
                       tasks[index].executionTime;

        for (int i = 0; i < index; i++) {
            error -= ceil(AtWithReplace(x, GenerateKey(index, "response"),
                                        rtaBase(index)) /
                          AtWithReplace(x, GenerateKey(i, "period"),
                                        tasks[i].period)) *
                     tasks[i].executionTime;
        }
        return GenerateVectorDynamic1D(error);
    };
    std::vector<gtsam::Symbol> keysVec;
    for (uint i = 0; i < tasks.size(); i++) {
        if (!maskForElimination[i]) keysVec.push_back(GenerateKey(i, "period"));
        if (!maskForElimination[i + 5])
            keysVec.push_back(GenerateKey(i, "response"));
    }
    auto model = gtsam::noiseModel::Isotropic::Sigma(
        1, noiseModelSigma / weightSchedulability);
    return MultiKeyFactor(keysVec, f, model);
}
}  // namespace rt_num_opt