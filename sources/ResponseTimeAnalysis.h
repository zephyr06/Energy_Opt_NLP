#pragma once

#include "Tasks.h"
#include "Declaration.h"

template <typename T>
T ResponseTimeAnalysisWarm(const T beginTime, const Task &taskCurr, const TaskSet &tasksHighPriority)
{
    const vector<int> periodHigh = GetParameter<int>(tasksHighPriority, "period");
    const vector<T> executionTimeHigh = GetParameter<T>(tasksHighPriority, "executionTime");
    int N = periodHigh.size();

    if (beginTime < 0)
    {
        cout << red << "During optimization, some variables drop below 0\n"
             << def << endl;
        throw;
    }
    for (int i = 0; i < int(executionTimeHigh.size()); i++)
    {
        if (executionTimeHigh[i] < 0)
        {
            cout << red << "During optimization, some variables drop below 0\n"
                 << def << endl;
            // throw;
            return INT32_MAX;
        }
    }

    if (Utilization(tasksHighPriority) > 1.0)
    {
        // cout << "The given task set is unschedulable\n";
        return INT32_MAX;
    }

    bool stop_flag = false;

    T responseTimeBefore = beginTime;
    while (not stop_flag)
    {
        T responseTime = taskCurr.executionTime;
        for (int i = 0; i < N; i++)
            responseTime += ceil(responseTimeBefore / double(periodHigh[i])) * executionTimeHigh[i];
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
template <typename T>
T ResponseTimeAnalysis(const Task &taskCurr, const TaskSet &tasksHighPriority)
{
    const vector<T> executionTimeHigh = GetParameter<T>(tasksHighPriority, "executionTime");
    T executionTimeAll = taskCurr.executionTime;
    for (auto &task : tasksHighPriority)
        executionTimeAll += task.executionTime;
    return ResponseTimeAnalysisWarm<T>(executionTimeAll, taskCurr, tasksHighPriority);
}

template <typename T>
bool CheckSchedulability(const TaskSet &taskSet)
{
    int N = taskSet.size();
    for (int i = 0; i < N; i++)
    {
        TaskSet::const_iterator first = taskSet.begin();
        vector<Task>::const_iterator last = taskSet.begin() + i;
        TaskSet hpTasks(first, last);
        T rta = ResponseTimeAnalysis<T>(taskSet[i], hpTasks);
        cout << "response time for task " << i << " is " << rta << endl;
        if (rta > min(taskSet[i].deadline, taskSet[i].period))
            return false;
    }
    return true;
}

VectorDynamic ResponseTimeOfTaskSetHard(TaskSet &tasks)
{
    int N = tasks.size();
    VectorDynamic res;
    res.resize(N, 1);

    vector<Task> hpTasks;
    for (int i = 0; i < N; i++)
    {
        res(i, 0) = ResponseTimeAnalysis<double>(tasks[i], hpTasks);
        if (res(i, 0) > tasks[i].deadline)
        {
            cout << "The given task set is not schedulable!\n";
            res(0, 0) = -1;
            return res;
        }
        hpTasks.push_back(tasks[i]);
    }
    return res;
}