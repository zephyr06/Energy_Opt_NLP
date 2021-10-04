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
    template <typename T>
    static T ResponseTimeAnalysisWarm(const T beginTime, const Task &taskCurr, const TaskSet &tasksHighPriority)
    {
        CoutWarning("Call a virtual function!");
        T a;
        return a;
    }

    template <typename T>
    static T ResponseTimeAnalysis(const Task &taskCurr, const TaskSet &tasksHighPriority)
    {
        CoutWarning("Call a virtual function!");
        T a;
        return a;
    }

    VectorDynamic ResponseTimeOfTaskSetHard(TaskSet &tasks);

    VectorDynamic ResponseTimeOfTaskSetHard(TaskSet tasks, VectorDynamic comp);

    template <typename T>
    static bool CheckSchedulability(const TaskSet &taskSet, bool whetherPrint = false)
    {
        int N = taskSet.size();
        for (int i = 0; i < N; i++)
        {
            TaskSet::const_iterator first = taskSet.begin();
            vector<Task>::const_iterator last = taskSet.begin() + i;
            TaskSet hpTasks(first, last);
            T rta = ResponseTimeAnalysis<T>(taskSet[i], hpTasks);
            if (whetherPrint)
                cout << "response time for task " << i << " is " << rta << " and deadline is " << taskSet[i].deadline << endl;
            if (rta > min(taskSet[i].deadline, taskSet[i].period))
                return false;
        }
        return true;
    }

    template <typename T>
    static bool CheckSchedulability(const TaskSet &taskSet, VectorDynamic warmStart, bool whetherPrint = false)
    {
        int N = taskSet.size();
        for (int i = 0; i < N; i++)
        {
            TaskSet::const_iterator first = taskSet.begin();
            vector<Task>::const_iterator last = taskSet.begin() + i;
            TaskSet hpTasks(first, last);
            T rta = ResponseTimeAnalysisWarm<T>(warmStart(i, 0), taskSet[i], hpTasks);
            if (whetherPrint)
                cout << "response time for task " << i << " is " << rta << " and deadline is " << taskSet[i].deadline << endl;
            if (rta > min(taskSet[i].deadline, taskSet[i].period))
                return false;
        }
        return true;
    }
};