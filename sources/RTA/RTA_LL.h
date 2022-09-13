#pragma once

#include "sources/TaskModel/Tasks.h"
#include "sources/MatrixConvenient.h"
#include "sources/RTA/RTA_BASE.h"
namespace rt_num_opt
{
    class RTA_LL : public RTA_BASE<TaskSetNormal>
    {

    public:
        RTA_LL(const TaskSet &tasksI)
        {
            TaskSetNormal tasksN(tasksI);
            tasks = tasksN;
        }
        RTA_LL(const TaskSetNormal &tasks) : RTA_BASE(tasks) {}

        double RTA_Common_Warm(double beginTime, int index) override
        {
            IncrementRTAControl();
            TaskSet tasksHp;
            for (int i = 0; i < index; i++)
            {
                tasksHp.push_back(tasks.tasks_.at(i));
            }
            return ResponseTimeAnalysisWarm(beginTime, tasks.tasks_.at(index), tasksHp);
        }

        // the rest are helper functions
        static std::string type()
        {
            return "LL";
        }

        double ResponseTimeAnalysisWarm_util_nece(double beginTime, const Task &taskCurr,
                                                  const TaskSet &tasksHighPriority)
        {
            const std::vector<int> periodHigh = GetParameter<int>(tasksHighPriority, "period");
            const std::vector<double> executionTimeHigh = GetParameter<double>(tasksHighPriority, "executionTime");
            int N = periodHigh.size();

            if (beginTime < 0)
            {
                if (debugMode == 1)
                {
                    CoutWarning("During optimization, some variables drop below 0\n");
                }
                beginTime = 0;
            }
            if (isnan(taskCurr.executionTime) || isnan(beginTime))
            {
                std::cout << Color::red << "Nan executionTime detected" << def << std::endl;
                throw "Nan";
            }
            if (taskCurr.executionTime < 0)
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
            for (uint i = 0; i < tasksHighPriority.size(); i++)
            {
                if (tasksHighPriority[i].period < 0)
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
                    if (responseTime > taskCurr.period)
                    {
                        return INT32_MAX;
                    }
                    return responseTime;
                }
                else
                {
                    responseTimeBefore = responseTime;
                }
                loopCount++;
                if (loopCount > 1500)
                {
                    CoutWarning("LoopCount error in RTA_LL");
                    return INT32_MAX;
                }
            }
            std::cout << "RTA analysis stops unexpectedly!\n";
            throw;
        }

        double ResponseTimeAnalysisWarm(const double beginTime, const Task &taskCurr,
                                        const TaskSet &tasksHighPriority)
        {
            if (Utilization(tasksHighPriority) + taskCurr.utilization() > 1.0 + 1e-6)
            {
                // double u1 = Utilization(tasksHighPriority);
                // double u2 = taskCurr.utilization();
                return INT32_MAX;
            }
            return ResponseTimeAnalysisWarm_util_nece(beginTime, taskCurr, tasksHighPriority);
        }

        double ResponseTimeAnalysis(const Task &taskCurr, const TaskSet &tasksHighPriority)
        {
            const std::vector<double> executionTimeHigh = GetParameter<double>(tasksHighPriority, "executionTime");
            double executionTimeAll = taskCurr.executionTime;
            for (auto &task : tasksHighPriority)
                executionTimeAll += task.executionTime;
            return ResponseTimeAnalysisWarm(executionTimeAll, taskCurr, tasksHighPriority);
        }
    };

    VectorDynamic RTAVector(const TaskSet &tasks)
    {
        BeginTimer(__func__);
        RTA_LL r(tasks);
        auto res = r.ResponseTimeOfTaskSet();
        EndTimer(__func__);
        return res;
    }

} // namespace rt_num_opt