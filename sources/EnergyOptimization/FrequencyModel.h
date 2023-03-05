#pragma once
#include "sources/TaskModel/Tasks.h"
#include "sources/Utils/Parameters.h"
namespace rt_num_opt {

double GetFrequency(const Task &task) {
    if (executionTimeModel == 1) {
        return task.executionTimeOrg / task.executionTime;
    } else if (executionTimeModel == 2) {
        return task.executionTimeOrg * frequencyRatio /
               (task.executionTime -
                task.executionTimeOrg * (1 - frequencyRatio));
    } else
        CoutError("executionTimeModel not recognized! Accept: 1, 2");
    return -1;
}
/**
 * @brief c = c_fix + c_var/f
 *
 * @param exec
 * @param task
 * @return double
 */
double Frequency2Execution(const Task &task) { return task.executionTime; }
void WriteFrequencyModelRatio(std::ofstream &file) {
    file << "Frequency_Ratio: " << RandRange(0.1, 0.9) << std::endl;
}
}  // namespace rt_num_opt