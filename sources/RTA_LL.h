#pragma once

#include "Tasks.h"
#include "Declaration.h"
#include "RTA_BASE.h"

class RTA_LL : public RTA_BASE
{
public:
    static double ResponseTimeAnalysisWarm(const double beginTime, const Task &taskCurr, const TaskSet &tasksHighPriority)
    {
        // if(tasksHighPriority[0].deadline==760830 )
        const vector<int> periodHigh = GetParameter<int>(tasksHighPriority, "period");
        const vector<double> executionTimeHigh = GetParameter<double>(tasksHighPriority, "executionTime");
        int N = periodHigh.size();

        if (beginTime < 0)
        {
            cout << red << "During optimization, some variables drop below 0\n"
                 << def << endl;
            throw;
        }
        else if (isnan(taskCurr.executionTime))
        {
            cout << red << "Nan executionTime detected" << def << endl;
            throw "Nan";
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

        if (Utilization(tasksHighPriority) + taskCurr.utilization() >= 1.0)
        {
            // cout << "The given task set is unschedulable\n";
            return INT32_MAX;
        }

        bool stop_flag = false;

        double responseTimeBefore = beginTime;
        while (not stop_flag)
        {
            double responseTime = taskCurr.executionTime;
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

    static double ResponseTimeAnalysis(const Task &taskCurr, const TaskSet &tasksHighPriority)
    {
        const vector<double> executionTimeHigh = GetParameter<double>(tasksHighPriority, "executionTime");
        double executionTimeAll = taskCurr.executionTime;
        for (auto &task : tasksHighPriority)
            executionTimeAll += task.executionTime;
        return ResponseTimeAnalysisWarm(executionTimeAll, taskCurr, tasksHighPriority);
    }

    static VectorDynamic ResponseTimeOfTaskSetHard(TaskSet &tasks)
    {
        int N = tasks.size();
        VectorDynamic res;
        res.resize(N, 1);

        vector<Task> hpTasks;
        if (debugMode == 1)
            cout << "RTA analysis (responseTime, deadline)" << endl;
        for (int i = 0; i < N; i++)
        {
            res(i, 0) = ResponseTimeAnalysis(tasks[i], hpTasks);
            if (debugMode == 1)
                cout << res(i, 0) << ", " << tasks[i].deadline << endl;
            if (res(i, 0) > min(tasks[i].deadline, tasks[i].period))
            {
                if (debugMode == 1)
                    cout << "The current task set is not schedulable!\n";
                res(0, 0) = -1;
                return res;
            }
            hpTasks.push_back(tasks[i]);
        }
        return res;
    }

    static VectorDynamic ResponseTimeOfTaskSetHardWarm(TaskSet tasks, VectorDynamic comp)
    {
        if (comp.rows() != int(tasks.size()))
        {
            cout << "Size mismatch error!" << endl;
            throw;
        }
        UpdateTaskSetExecutionTime(tasks, comp);
        return ResponseTimeOfTaskSetHard(tasks);
    }
};

// template <typename T>
// bool CheckSchedulability(const TaskSet &taskSet, bool whetherPrint = false)
// {
//     int N = taskSet.size();
//     for (int i = 0; i < N; i++)
//     {
//         TaskSet::const_iterator first = taskSet.begin();
//         vector<Task>::const_iterator last = taskSet.begin() + i;
//         TaskSet hpTasks(first, last);
//         T rta = ResponseTimeAnalysis(taskSet[i], hpTasks);
//         if (whetherPrint)
//             cout << "response time for task " << i << " is " << rta << " and deadline is " << taskSet[i].deadline << endl;
//         if (rta > min(taskSet[i].deadline, taskSet[i].period))
//             return false;
//     }
//     return true;
// }

// template <typename T>
// bool CheckSchedulability(const TaskSet &taskSet, VectorDynamic warmStart, bool whetherPrint = false)
// {
//     int N = taskSet.size();
//     for (int i = 0; i < N; i++)
//     {
//         TaskSet::const_iterator first = taskSet.begin();
//         vector<Task>::const_iterator last = taskSet.begin() + i;
//         TaskSet hpTasks(first, last);
//         T rta = ResponseTimeAnalysisWarm(warmStart(i, 0), taskSet[i], hpTasks);
//         if (whetherPrint)
//             cout << "response time for task " << i << " is " << rta << " and deadline is " << taskSet[i].deadline << endl;
//         if (rta > min(taskSet[i].deadline, taskSet[i].period))
//             return false;
//     }
//     return true;
// }
