#include "Tasks.h"

int ResponseTimeAnalysisWarm(const int beginTime, const Task &taskCurr, const TaskSet &tasksHighPriority)
{
    const vector<int> periodHigh = GetParameter(tasksHighPriority, "period");
    const vector<int> executionTimeHigh = GetParameter(tasksHighPriority, "executionTime");
    int N = periodHigh.size();

    if (Utilization(tasksHighPriority) > 1.0)
    {
        // cout << "The given task set is unschedulable\n";
        return INT32_MAX;
    }

    bool stop_flag = false;

    int responseTimeBefore = beginTime;
    while (not stop_flag)
    {
        int responseTime = taskCurr.executionTime;
        for (uint i = 0; i < N; i++)
            responseTime += ceil(responseTimeBefore / float(periodHigh[i])) * executionTimeHigh[i];
        if (responseTime == responseTimeBefore)
        {
            stop_flag = true;
            return responseTime;
        }
        else
        {
            responseTimeBefore = responseTime;
        }
    }
    cout << "RTA analysis stops unexpectedly!\n";
    throw;
}

int ResponseTimeAnalysis(const Task &taskCurr, const TaskSet &tasksHighPriority)
{
    const vector<int> executionTimeHigh = GetParameter(tasksHighPriority, "executionTime");
    int executionTimeAll = taskCurr.executionTime + (tasksHighPriority.begin(),
                                                     tasksHighPriority.end(), 0);

    return ResponseTimeAnalysisWarm(executionTimeAll, taskCurr, tasksHighPriority);
}
