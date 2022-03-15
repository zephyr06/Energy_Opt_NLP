#pragma once
#include "MultiKeyFactor.h"
#include "ControlFactorGraphUtils.h"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
typedef long long int LLint;

MultiKeyFactor GenerateTaskRTAFactor(std::vector<bool> &maskForElimination, TaskSet &tasks, int index)
{
    LambdaMultiKey f = [tasks, index](const Values &x)
    {
        // if (debugMode == 1)
        // {
        //     std::lock_guard<std::mutex> lock(mtx);
        //     cout << "The values input to GenerateTaskRTAFactor: " << endl;
        //     x.print();
        // }
        double error = x.at<VectorDynamic>(GenerateControlKey(index, "response"))(0, 0) -
                       tasks[index].executionTime;
        for (int i = 0; i < index; i++)
        {
            double tj = 0;
            if (x.exists(GenerateControlKey(i, "period")))
            {
                tj = x.at<VectorDynamic>(GenerateControlKey(i, "period"))(0, 0);
            }
            else
            {
                tj = tasks[i].period;
            }
            error -= ceil(x.at<VectorDynamic>(GenerateControlKey(index, "response"))(0, 0) /
                          tj) *
                     tasks[i].executionTime;
        }

        VectorDynamic res = GenerateVectorDynamic(1);
        res << error;
        return res;
    };
    std::vector<gtsam::Symbol> keysVec;
    for (uint i = 0; i < tasks.size(); i++)
    {
        if (!maskForElimination[i])
        {
            keysVec.push_back(GenerateControlKey(i, "period"));
        }
        keysVec.push_back(GenerateControlKey(i, "response"));
    }
    auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightSchedulability);
    // auto modelPunishmentHard = noiseModel::Constrained::All(1);
    // return MultiKeyFactor(keysVec, f, modelPunishmentHard);
    return MultiKeyFactor(keysVec, f, model);
}
void AddRTAFactor(NonlinearFactorGraph &graph, std::vector<bool> maskForElimination, TaskSet &tasks)
{
    for (int index = 0; index < int(tasks.size()); index++)
    {
        MultiKeyFactor f = GenerateTaskRTAFactor(maskForElimination, tasks, index);
        graph.add(f);
    }
}