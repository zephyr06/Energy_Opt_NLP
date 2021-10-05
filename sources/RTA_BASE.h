#pragma once

#include "Tasks.h"
#include "Declaration.h"

/**
 * @brief RTA_BASE encapsulate all the interafaces required
 *  by Energy_optimization.
 * All kinds of RTA should inherit 
 * from RTA_BASE and implement its virtual function
 * 
 */

class RTA_BASE
{
public:
    double ResponseTimeAnalysisWarm(const double beginTime, const Task &taskCurr, const TaskSet &tasksHighPriority);

    double ResponseTimeAnalysis(const Task &taskCurr, const TaskSet &tasksHighPriority);

    VectorDynamic ResponseTimeOfTaskSetHard(TaskSet &tasks);

    VectorDynamic ResponseTimeOfTaskSetHardWarm(TaskSet tasks, VectorDynamic comp);
};

template <class Schedul_Analysis>
bool CheckSchedulability(const TaskSet &taskSet, bool whetherPrint = false)
{
    int N = taskSet.size();
    for (int i = 0; i < N; i++)
    {
        TaskSet::const_iterator first = taskSet.begin();
        vector<Task>::const_iterator last = taskSet.begin() + i;
        TaskSet hpTasks(first, last);
        double rta = Schedul_Analysis::ResponseTimeAnalysis(taskSet[i], hpTasks);
        if (whetherPrint)
            cout << "response time for task " << i << " is " << rta << " and deadline is " << taskSet[i].deadline << endl;
        if (rta > min(taskSet[i].deadline, taskSet[i].period))
            return false;
    }
    return true;
}

template <class Schedul_Analysis>
bool CheckSchedulability(const TaskSet &taskSet, VectorDynamic warmStart, bool whetherPrint = false)
{
    int N = taskSet.size();
    for (int i = 0; i < N; i++)
    {
        TaskSet::const_iterator first = taskSet.begin();
        vector<Task>::const_iterator last = taskSet.begin() + i;
        TaskSet hpTasks(first, last);
        double rta = Schedul_Analysis::ResponseTimeAnalysisWarm(warmStart(i, 0), taskSet[i], hpTasks);
        if (whetherPrint)
            cout << "response time for task " << i << " is " << rta << " and deadline is " << taskSet[i].deadline << endl;
        if (rta > min(taskSet[i].deadline, taskSet[i].period))
            return false;
    }
    return true;
}