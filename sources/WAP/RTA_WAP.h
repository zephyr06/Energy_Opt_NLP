#include "Declare_WAP.h"
#include "../ResponseTimeAnalysis.h"

// TODO: add beginTime version for the following function

/**
 * @brief estimate block time for a given task
 * 
 * @param tasks 
 * @param A 
 * @param P 
 * @param index 
 * @return double the blocking time
 */
double BlockingTime(TaskSet tasks, SquareMatrix A, SquareMatrix P, int index)
{
    int N = tasks.size();
    double blockTime = 0;
    for (int l = index + 1; l < N; l++)
    {
        if (A(index, l) == 0 && P(index, l) == 1)
            blockTime = max(blockTime, double(tasks[l].overhead));
        else if (A(index, l) == 0 && P(index, l) == 0)
            blockTime = max(blockTime, tasks[l].executionTime - 1);
        else
            blockTime = max(blockTime, 0.0);
    }
    return blockTime;
}

/**
 * @brief Update hp's execution time with abortion/preemption cost,
 * so that standard RTA can be applied
 * 
 * @param tasks 
 * @param A 
 * @param P 
 * @param index 
 * @return TaskSet 
 */
TaskSet UpdateHpWap(TaskSet &tasks, SquareMatrix A, SquareMatrix P, int index)
{
    TaskSet tasksUpdate;

    for (int j = 0; j < index; j++)
    {
        double overhead = 0;
        for (int k = j + 1; k < index + 1; k++)
        {
            if (A(j, k) == 1)
            {
                overhead = max(overhead, tasks[k].executionTime - 1);
            }
        }

        // if no tasks perform abortion, preemption will be used
        if (overhead == 0 && P(j, index) == 1)
        {
            overhead = tasks[index].overhead;
        }
        tasksUpdate.push_back(tasks[j]);
        tasksUpdate.back().executionTime += overhead;
    }
    return tasksUpdate;
}

/**
 * @brief Get the Busy Period object
 * 
 * @param tassk 
 * @param A 
 * @param P 
 * @return double 
 */
double GetBusyPeriod(TaskSet tasks, SquareMatrix A, SquareMatrix P, int index)
{
    TaskSet tasksUpdate = UpdateHpWap(tasks, A, P, index);
    TaskSet hpTasks;

    for (int i = 0; i < index; i++)
        hpTasks.push_back(tasksUpdate[i]);
    hpTasks.push_back(tasks[index]);

    Task t;
    t.executionTime = BlockingTime(tasks, A, P, index);
    t.period = INT32_MAX;
    return ResponseTimeAnalysis<double>(t, hpTasks);
}

/**
 * @brief Given block time for one task, estimate its response time using standard RTA
 * 
 * @param tasks; accept a copy rather than reference of the task set!!
 * @param A 
 * @param P 
 * @param index 
 * @param block 
 * @return double 
 */
double ResponseTimeWapGivenBlock(TaskSet tasks, SquareMatrix A, SquareMatrix P, int index, double block)
{

    double busyPeriod = GetBusyPeriod(tasks, A, P, index);

    Task taskCurr = tasks[index];
    taskCurr.executionTime += block;
    TaskSet tasksUpdate = UpdateHpWap(tasks, A, P, index);
    TaskSet hpTasks;
    for (int i = 0; i < index; i++)
        hpTasks.push_back(tasksUpdate[i]);

    double worstRT = ResponseTimeAnalysis<double>(taskCurr, hpTasks);
    for (int i = 1; i < int(ceil(busyPeriod / tasks[index].period)); i++)
    {
        taskCurr.executionTime += tasks[index].executionTime;
        double responseTime_q = ResponseTimeAnalysis<double>(taskCurr, hpTasks);
        worstRT = max(worstRT, responseTime_q - (i * taskCurr.period));
    }

    return worstRT;
}

/**
 * @brief Estimate response time for the WAP model
 * 
 * @param tasks 
 * @param A 
 * @param P 
 * @param index 
 * @return double 
 */
double ResponseTimeWAP(TaskSet tasks, SquareMatrix A, SquareMatrix P, int index)
{
    double block = BlockingTime(tasks, A, P, index);
    return ResponseTimeWapGivenBlock(tasks, A, P, index, block);
}
