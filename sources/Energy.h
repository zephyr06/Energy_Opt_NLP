#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "Tasks.h"
#include "Parameters.h"
#include "Declaration.h"
const double EstimateEnergyTask(const Task &task, double frequency)
/**
 * @brief Estimate energy consumption of a single task;
    all tasks' default frequency is 1, which is also the maximum frequency;
 * @param
    frequency is defined as f = C_0 / C_real
    @return
    scalar, energy consumption of the task if it runs at the given frequency
*/
{
    if (frequency * (task.executionTime - 1e-3) <= task.executionTime + 1e-6 && frequency > 0)
        return task.executionTime * pow(frequency, 2) * weightEnergy;
    else
        return task.executionTime * pow(frequency, 2) * weightEnergy * punishmentFrequency;
}
const double EstimateEnergyTaskBasedComputation(const Task &task, double computationTime)
/**
 * @brief Estimate energy consumption of a single task;
    all tasks' default frequency is 1, which is also the maximum frequency;
 * @param
    frequency is defined as f = C_0 / C_real
    @return
    scalar, energy consumption of the task if it runs at the given frequency
*/
{
    double frequency = computationTime / task.executionTime;
    if (computationTime >= task.executionTime - deltaOptimizer)
        return task.executionTime * pow(frequency, 2) * weightEnergy;
    else
        return task.executionTime * pow(frequency, 2) * weightEnergy * punishmentFrequency;
}

// ErrElement EstimateEnergyTaskSet(const TaskSet &taskSet, const ComputationTimeVector &executionTimeVector)
// {
//     int N = taskSet.size();
//     long long int hp = HyperPeriod(taskSet);
//     ErrElement res;
//     for (int i = 0; i < N; i++)
//     {
//         double frequencyRunTime = taskSet[i].executionTime / executionTimeVector(i, 0);
//         res(i, 0) = hp / taskSet[i].period *
//                     EstimateEnergyTask(taskSet[i], frequencyRunTime);
//     }
//     return res;
// }

VectorDynamic EstimateEnergyTaskSet(const TaskSet &taskSet, const VectorDynamic &executionTimeVector)
{
    int N = taskSet.size();
    int n = executionTimeVector.rows();

    long long int hp = HyperPeriod(taskSet);
    MatrixDynamic res;
    res.resize(n, 1);

    for (int i = 0; i < N; i++)
    {
        double frequencyRunTime = taskSet[i].executionTime / executionTimeVector(i, 0);
        res(i, 0) = hp / taskSet[i].period *
                    EstimateEnergyTask(taskSet[i], frequencyRunTime);
    }
    return res;
}
