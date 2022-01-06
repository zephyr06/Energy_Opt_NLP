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

double Execution2Frequency(double exec, const Task &task)
{
    if (executionTimeModel == 1)
    {
        return task.executionTime / exec;
    }
    else if (executionTimeModel == 2)
    {
        return task.executionTime * 0.9 / (exec - task.executionTime * 0.1);
    }
    else
        CoutError("executionTimeModel not recognized! Accept: 1, 2");
    return -1;
}