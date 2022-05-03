#pragma once

#include "dagSched/tests.h"

#include "sources/TaskModel/DAG_Task.h"

namespace rt_num_opt
{
    struct DAG_Narsi19 : public TaskSetNormal
    {
        DAG_Model tasksVecNarsi_; // it mainly stores the graphical structure of DAGs
        std::string dagCsv;

        DAG_Narsi19() {}
        DAG_Narsi19(DAG_Model &dagTasks)
        {
            tasks_ = dagTasks.tasks_;
            N = tasks_.size();

            dagCsv = convertDAGsToCsv(dagTasks);
        }

        static inline std::string Type() { return "Nasri"; }

        void SyncTaskSet()
        {
            index = 0;
            for (uint i = 0; i < dagTasks.size(); i++)
            {
                for (uint j = 0; j < dagTasks[i].size(); j++)
                {
                    tasksVecNarsi_[i][j] = tasks_[index++];
                }
            }
        }

        static std::string LineInTaskCsv(int taskId, int jobId, int releaseMin, int releaseMax, int costMin, int costMax, int deadline, int priority)
        {

            return std::to_string(taskId) + ", " + std::to_string(jobId) + ", " + std::to_string(releaseMin) + ", " + std::to_string(releaseMax) + ", " + std::to_string(costMin) + ", " + std::to_string(costMax) + ", " + std::to_string(deadline) + ", " + std::to_string(priority);
        }
        static std::string ConvertTasksetToCsv(const TaskSet &tasks)
        {
            auto hyperPeriod = HyperPeriod(tasks);
            std::string taskSetStr = "   Task ID,     Job ID,          Arrival min,          Arrival max,             Cost min,             Cost max,             Deadline,             Priority\n";
            for (uint i = 0; i < tasks_.size(); i++)
            {
                for (int j = 0; j < hyperPeriod / tasks[i].period; j++)
                {
                    taskSetStr += LineInTaskCsv(tasks_[i].id, j, tasks[i].offset + j * tasks[i].period, tasks[i].offset + j * tasks[i].period, tasks[i].executionTime, tasks[i].executionTime, tasks[i].deadline, tasks[i].deadline);
                }
            }
            return taskSetStr;
        }

        static std::string convertDAGsToCsv(const DAG_Model &dagModel)
        {
            auto hyperPeriod = HyperPeriod(tasks);
            std::string dependStr = "Predecessor TID,	Predecessor JID,	Successor TID, Successor JID\n";
            for (uint i = 0; i < tasks_.size(); i++)
            {
                for (int j = 0; j < hyperPeriod / tasks[i].period; j++)
                {
                    dependStr += LineInTaskCsv(tasks_[i].id, j, j * tasks[i].period, j * tasks[i].period, tasks[i].executionTime, tasks[i].executionTime, tasks[i].deadline, tasks[i].deadline);
                }
            }
            return dependStr;
        }
    };

    DAG_Narsi19 ReadDAGNarsi19_Tasks(std::string paths) // std::string priorityType = "orig"
    {
        std::vector<rt_num_opt::DAG_Model> dagsNum = TransformTaskSetNumOpt2dagSched(paths);
        return DAG_Narsi19(dagsNum);
    }
}