#pragma once
#include "MultiKeyFactor.h"
#include "ControlFactorGraphUtils.h"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
typedef long long int LLint;

MultiKeyFactor AddTaskRTAFactor(std::vector<bool> &maskForElimination, TaskSet &tasks, int index)
{
    LambdaMultiKey f = [maskForElimination, tasks, index](const Values &x)
    {
        double error = (x.at<VectorDynamic>(GenerateControlKey(index, "response")) -
                        x.at<VectorDynamic>(GenerateControlKey(index, "period")))(0, 0);
        for (int i = 0; i < index; i++)
        {
            double tj = 0;
            if (maskForElimination[i])
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
        if (maskForElimination[i])
            keysVec.push_back(GenerateControlKey(i, "period"));
    }
    auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    return MultiKeyFactor(keysVec, f, model);
}
void AddRTAFactor(NonlinearFactorGraph &graph, std::vector<bool> maskForElimination, TaskSet &tasks)
{
    for (int index = 0; index < int(tasks.size()); index++)
    {
        LambdaMultiKey f = [tasks, index](const Values &x)
        {
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
            if (not maskForElimination[i])
                keysVec.push_back(GenerateControlKey(i, "period"));
        }
        auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
        graph.emplace_shared<MultiKeyFactor>(keysVec, f, model);
    }
}