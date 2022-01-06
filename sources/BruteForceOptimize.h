#pragma once

#include "Optimize.h"

double OptimizeTaskSetBf3(TaskSet &tasks, int granularity = granularityInBF)
{
    int N = tasks.size();

    // this function also checks schedulability
    VectorDynamic responseTimeInitial = ResponseTimeOfTaskSet<RTA_LL>(tasks);
    if (!CheckSchedulabilityDirect(tasks, responseTimeInitial))
        return -2;

    double minEnergy = INT64_MAX;
    VectorDynamic minExecutionTime;
    minExecutionTime.resize(N, 1);
    for (int i = 0; i < N; i++)
        minExecutionTime(i, 0) = 0;

    vector<int> lowerBound, upperBound;
    for (int i = 0; i < N; i++)
    {
        lowerBound.push_back(tasks[i].executionTime);
        upperBound.push_back(min(tasks[i].period, tasks[i].deadline));
    }

    for (int firstTaskExe = lowerBound[0]; firstTaskExe < upperBound[0]; firstTaskExe += granularity)
    {
        for (int secondTaskExe = lowerBound[1]; secondTaskExe < upperBound[1]; secondTaskExe += granularity)
        {
            for (int thirdTaskExe = lowerBound[2]; thirdTaskExe < upperBound[2]; thirdTaskExe += granularity)
            {
                VectorDynamic executionTimeVector;
                executionTimeVector.resize(N, 1);
                executionTimeVector << firstTaskExe, secondTaskExe, thirdTaskExe;
                TaskSet taskSetCurr_ = tasks;
                UpdateTaskSetExecutionTime(taskSetCurr_, executionTimeVector);
                if (CheckSchedulability<RTA_LL>(taskSetCurr_))
                {
                    double energyCurr = EstimateEnergyTaskSet(tasks, executionTimeVector).sum();
                    if (energyCurr < minEnergy)
                    {
                        minEnergy = energyCurr;
                        minExecutionTime = executionTimeVector;
                    }
                }
                else
                    break;
            }
        }
    }

    VectorDynamic initialExecutionTime;
    initialExecutionTime.resize(N, 1);
    for (int i = 0; i < N; i++)
        initialExecutionTime(i, 0) = tasks[i].executionTime;
    double initialEnergyCost = EstimateEnergyTaskSet(tasks, initialExecutionTime).sum();
    cout << "The global optimal computation time vector is " << minExecutionTime << endl;
    TaskSet taskSetCurr_ = tasks;
    UpdateTaskSetExecutionTime(taskSetCurr_, minExecutionTime);
    bool dummy = CheckSchedulability<RTA_LL>(taskSetCurr_, true);
    return minEnergy / initialEnergyCost;
}