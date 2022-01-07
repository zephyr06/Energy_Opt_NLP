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
#include "FrequencyModel.h"
const double EstimateEnergyTask(const Task &task)
/**
 * @brief Estimate energy consumption of a single task;
    all tasks' default frequency is 1, which is also the maximum frequency;
 * @param
    frequency is defined as f = C_0 / C_real
    @return
    scalar, energy consumption of the task if it runs at the given frequency
*/
{
    double frequency = GetFrequency(task);
    double energy;
    if (EnergyMode == 1)
        energy = task.executionTime * (pow(frequency, 3));
    else if (EnergyMode == 2)
        energy = task.executionTime * (pow(frequency, 3) + pow(frequency, 2));
    else
        CoutError("Not recognized EnergyMode!");
    energy *= weightEnergy;
    if (frequency > 0 && frequency <= 1)
        return energy;
    else
        return energy * punishmentFrequency;
}

VectorDynamic EstimateEnergyTaskSet(const TaskSet &tasks)
{
    int N = tasks.size();
    MatrixDynamic res = GenerateVectorDynamic(N);

    for (int i = 0; i < N; i++)
    {
        res(i, 0) = 1.0 / tasks[i].period *
                    EstimateEnergyTask(tasks[i]);
    }
    return res;
}