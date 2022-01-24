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
#include "utils.h"
#include "RTA_DAG.h"
/**
 * @brief Estimate energy consumption of a single task;
    all tasks' default frequency is 1, which is also the maximum frequency;
 * @param
    frequency is defined as f = C_0 / C_real
    @return
    scalar, energy consumption of the task if it runs at the given frequency
*/
const double EstimateEnergyTask(const Task &task)
{
    double frequency = GetFrequency(task);
    double energy;
    if (EnergyMode == 1)
        energy = task.executionTime * (pow(frequency, 3));
    else if (EnergyMode == 2)
        energy = task.executionTime * (5 * pow(frequency, 3) + 0.09 * pow(frequency, 2) +
                                       0.44 * pow(frequency, 1) +
                                       +0.47);
    // energy = task.executionTime * (pow(frequency, 3) + 0.09 * pow(frequency, 2));
    else
        CoutError("Not recognized EnergyMode!");
    energy *= weightEnergy;
    if (frequency > 0 && frequency <= 1.1)
        return energy;
    else
        return energy * punishmentFrequency;
}

VectorDynamic EstimateEnergyTaskSet(const TaskSet &tasks)
{
    int N = tasks.size();
    MatrixDynamic res = GenerateVectorDynamic(N);
    VectorDynamic rta;
    // if (EnergyMode == 3)
    // {
    //     RTA_DAG r(tasks);
    //     rta = r.ResponseTimeOfTaskSet();
    // }
    for (int i = 0; i < N; i++)
    {
        if (EnergyMode == 3)
        {
            // res(i, 0) = tasks.weightVec_[i] * tasks.tasks_[i].period + rta(i);
            ;
        }
        if (EnergyMode == 1 || EnergyMode == 2)
        {
            res(i, 0) = 1.0 / tasks[i].period *
                        EstimateEnergyTask(tasks[i]);
        }
    }
    return res;
}

double JacobianInEnergyItem(const TaskSet &tasks, int i)
{
    Task taskRef = tasks[i];
    boost::function<gtsam::Matrix(const VectorDynamic &)> f =
        [taskRef, i](const VectorDynamic &executionTimeVector)
    {
        Task task = taskRef;
        double ref = task.executionTime;
        task.executionTime = executionTimeVector(0);
        double res = EstimateEnergyTask(task);
        task.executionTime = ref;
        MatrixDynamic rrr = GenerateZeroMatrix(1, 1);
        rrr << res;
        return rrr;
    };
    VectorDynamic x = GenerateVectorDynamic(1);
    x << tasks[i].executionTime;
    return NumericalDerivativeDynamic(f, x, deltaOptimizer, 1)(0, 0);
};