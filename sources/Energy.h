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
    if (frequency <= task.executionTime / (task.executionTime - 1e-3))
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

VectorDynamic EstimateEnergyTaskSet(const TaskSet &taskSet, const VectorDynamic &executionTimeVector,
                                    int lastTaskDoNotNeedOptimize = -1)
{
    if (executionTimeVector.sum() != 0)
    {
        int N = taskSet.size();
        int n = executionTimeVector.rows();

        MatrixDynamic res;
        res.resize(n, 1);

        for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
        {
            double frequencyRunTime = taskSet[i].executionTime /
                                      executionTimeVector(i - lastTaskDoNotNeedOptimize - 1, 0);
            res(i - lastTaskDoNotNeedOptimize - 1, 0) = 1.0 / taskSet[i].period *
                                                        EstimateEnergyTask(taskSet[i], frequencyRunTime);
        }
        return res;
    }
    else
    {
        cout << "Input error in EstimateEnergyTaskSet" << endl;
        throw;
    }
}