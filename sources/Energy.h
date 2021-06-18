#include "Tasks.h"

const float EstimateEnergyTask(const Task &task, float frequency)
/**
 * @brief Estimate energy consumption of a single task;
    all tasks' default frequency is 1, which is also the maximum frequency;
 * @param
    frequency is defined as f = C_0 / C_real
    @return
    scalar, energy consumption of the task if it runs at the given frequency
*/
{
    return task.executionTime * pow(frequency, 2);
}

const vector<float> EstimateEnergyTaskSet(const TaskSet &tasks, vector<float> frequencyList)
{
    int N = tasks.size();
    int hp = HyperPeriod(tasks);
    vector<float> energyAll;
    energyAll.reserve(N);

    for (uint i = 0; i < N; i++)
    {
        energyAll.push_back(hp / tasks[i].period *
                            EstimateEnergyTask(tasks[i], frequencyList[i]));
    }
    return energyAll;
}
