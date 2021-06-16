#include "Tasks.h"

int ResponseTimeAnalysisWarm(const int beginTime, const TaskPeriodic &taskCurr, const TaskSet &tasksHighPriority)
{
    const vector<int> periodHigh = tasksHighPriority.GetParameter("period");
    const vector<int> executionTimeHigh = tasksHighPriority.GetParameter("executionTime");
    int N = periodHigh.size();

    if (tasksHighPriority.utilization() > 1 - 1e-4)
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

int ResponseTimeAnalysis(const TaskPeriodic &taskCurr, const TaskSet &tasksHighPriority)
{
    const vector<int> executionTimeHigh = tasksHighPriority.GetParameter("executionTime");
    int executionTimeAll = taskCurr.executionTime + (tasksHighPriority.tasks.begin(),
                                                     tasksHighPriority.tasks.end(), 0);

    return ResponseTimeAnalysisWarm(executionTimeAll, taskCurr, tasksHighPriority);
}
