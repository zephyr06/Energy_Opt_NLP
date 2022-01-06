#pragma once
#include "Parameters.h"
#include "Tasks.h"
/**
 * @brief we assume c_fix=0.1 c_org, c_var=0.9 c_org in dynamic execution time model
 * 
 * @param exec 
 * @param task 
 * @return double 
 */

double GetFrequency(const Task &task)
{
    if (executionTimeModel == 1)
    {
        return task.executionTimeOrg / task.executionTime;
    }
    else if (executionTimeModel == 2)
    {
        return task.executionTimeOrg * 0.9 / (task.executionTime - task.executionTimeOrg * 0.1);
    }
    else
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
double Frequency2Execution(const Task &task)
{
    return task.executionTime;
}