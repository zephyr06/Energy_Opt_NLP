#pragma once

#include <gtsam/base/Matrix.h>

#include "sources/TaskModel/Tasks.h"
#include "sources/Utils/Parameters.h"
#include "sources/MatrixConvenient.h"
#include "sources/EnergyOptimization/FrequencyModel.h"
#include "sources/Utils/utils.h"

namespace rt_num_opt
{
    const double EstimateEnergyTask(const Task &task)
    /**
    * @brief Estimate energy consumption of a single task;
    all tasks' default frequency is 1, which is also the maximum frequency;
    second frequency model is adopted from Guo19RTSS
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
            energy = task.executionTime * (1.76 * pow(frequency, 3) + 0.5);
        else if (EnergyMode == 3)
            energy = task.period * 1.0;
        // energy = task.executionTime * (pow(frequency, 3) + 0.09 * pow(frequency, 2));
        else
        {
            CoutError("Not recognized EnergyMode!");
            energy = 0; // just to suppress a warning
        }

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

        for (int i = 0; i < N; i++)
        {
            res(i, 0) = pow(1.0 / tasks[i].period *
                                EstimateEnergyTask(tasks[i]),
                            1);
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
            MatrixDynamic rrr = GenerateMatrixDynamic(1, 1);
            rrr << res;
            return rrr;
        };
        VectorDynamic x = GenerateVectorDynamic(1);
        x << tasks[i].executionTime;
        return NumericalDerivativeDynamic(f, x, deltaOptimizer, 1)(0, 0);
    }

} // namespace rt_num_opt