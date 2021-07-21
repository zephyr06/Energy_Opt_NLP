#include "Declare_WAP.h"
#include "../ResponseTimeAnalysis.h"

/**
 * @brief estimate block time for a given task
 * 
 * @param tasks 
 * @param A 
 * @param P 
 * @param index 
 * @return double the blocking time
 */
double BlockingTime(TaskSet tasks, TYPE_A A, TYPE_P P, int index)
{
    int N = tasks.size();
    double blockTime = 0;
    for (int l = index + 1; l < N; l++)
    {
        if (A(index, l) == 0 && P(index, l) == 1)
            blockTime = max(blockTime, tasks[l].overhead);
        else if (A(index, l) == 0 && P(index, l) == 0)
            blockTime = max(blockTime, tasks[l].executionTime - 1);
        else
            blockTime = max(blockTime, 0);
    }
    return blockTime;
}
