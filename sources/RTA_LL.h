#pragma once

#include "Tasks.h"
#include "Declaration.h"
#include "RTA_BASE.h"
template <class TaskType>
class RTA_LL : public RTA_BASE<TaskType>
{

public:
    typedef std::vector<TaskType> TaskSetType;
    static string type()
    {
        return "LL";
    }
    static double RTA_Common(const TaskSetType &tasks, int index)
    {
        TaskSetType tasksHp;
        for (int i = 0; i < index; i++)
        {
            tasksHp.push_back(tasks.at(i));
        }
        return ResponseTimeAnalysis(tasks.at(index), tasksHp);
    }
    static double RTA_Common_Warm(double beginTime, const TaskSetType &tasks, int index)
    {
        TaskSetType tasksHp;
        for (int i = 0; i < index; i++)
        {
            tasksHp.push_back(tasks.at(i));
        }
        return ResponseTimeAnalysisWarm(beginTime, tasks.at(index), tasksHp);
    }

    static double ResponseTimeAnalysisWarm_util_nece(double beginTime, const TaskType &taskCurr,
                                                     const TaskSetType &tasksHighPriority)
    {
        // if(tasksHighPriority[0].deadline==760830 )
        const vector<int> periodHigh = GetParameter<int>(tasksHighPriority, "period");
        const vector<double> executionTimeHigh = GetParameter<double>(tasksHighPriority, "executionTime");
        int N = periodHigh.size();

        if (beginTime < 0)
        {
            if (debugMode == 1)
            {
                CoutWarning("During optimization, some variables drop below 0\n");
            }
            beginTime = 0;
        }
        else if (isnan(taskCurr.executionTime) || isnan(beginTime))
        {
            cout << red << "Nan executionTime detected" << def << endl;
            throw "Nan";
        }
        else if (taskCurr.executionTime < 0)
        {
            return INT32_MAX;
        }
        for (int i = 0; i < int(executionTimeHigh.size()); i++)
        {
            if (executionTimeHigh[i] < 0)
            {
                if (debugMode)
                {
                    CoutWarning("During optimization, some variables drop below 0\n");
                }
                return INT32_MAX;
            }
        }
        double utilAll = Utilization(tasksHighPriority);
        if (utilAll >= 1.0 - utilTol)
        {
            // cout << "The given task set is unschedulable\n";
            return INT32_MAX;
        }

        bool stop_flag = false;

        double responseTimeBefore = beginTime;
        int loopCount = 0;
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
            loopCount++;
            if (loopCount > 10000)
            {
                CoutError("LoopCount error in RTA_LL");
            }
        }
        cout << "RTA analysis stops unexpectedly!\n";
        throw;
    }

    static double ResponseTimeAnalysisWarm(const double beginTime, const TaskType &taskCurr,
                                           const std::vector<TaskType> &tasksHighPriority)
    {
        if (Utilization(tasksHighPriority) + taskCurr.utilization() >= 1.0)
        {
            // cout << "The given task set is unschedulable\n";
            return INT32_MAX;
        }
        return ResponseTimeAnalysisWarm_util_nece(beginTime, taskCurr, tasksHighPriority);
    }

    static double ResponseTimeAnalysis(const TaskType &taskCurr, const std::vector<TaskType> &tasksHighPriority)
    {
        const vector<double> executionTimeHigh = GetParameter<double>(tasksHighPriority, "executionTime");
        double executionTimeAll = taskCurr.executionTime;
        for (auto &task : tasksHighPriority)
            executionTimeAll += task.executionTime;
        return ResponseTimeAnalysisWarm(executionTimeAll, taskCurr, tasksHighPriority);
    }
};
