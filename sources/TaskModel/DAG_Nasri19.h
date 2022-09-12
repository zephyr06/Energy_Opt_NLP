#pragma once

// #include "dagSched/tests.h"

#include "sources/TaskModel/DAG_Task.h"
#include "sources/TaskModel/ReadWriteYaml.h"

namespace rt_num_opt
{
    struct DAG_Nasri19 : public TaskSetNormal
    {
        std::vector<rt_num_opt::DAG_Model> tasksVecNasri_; // it mainly stores the graphical structure of DAGs
        // std::vector<int> nodeSizes_;
        std::string dagCsv;
        long long int hyperPeriod;

        DAG_Nasri19() {}
        /**
         * @brief Construct a new dag Nasri19 object
         *               ...
         * @param dagsNum
         */
        DAG_Nasri19(std::vector<rt_num_opt::DAG_Model> &dagsNum)
        {
            hyperPeriod = GetHyperPeriodNasri(dagsNum);

            tasksVecNasri_ = dagsNum;
            for (size_t taskId = 0; taskId < tasksVecNasri_.size(); taskId++)
            {
                for (size_t jobId = 0; jobId < tasksVecNasri_[taskId].tasks_.size(); jobId++)
                {
                    // for (int instanceId = 0; instanceId < hyperPeriod / tasksVecNasri_[i].tasks_[0].period; instanceId++)
                    // {
                    tasks_.push_back(tasksVecNasri_[taskId].tasks_[jobId]);
                }
            }
            N = tasks_.size();
            dagCsv = convertDAGsToCsv();
        }

        static long long int GetHyperPeriodNasri(std::vector<rt_num_opt::DAG_Model> &dagsNum)
        {
            TaskSet tasks;
            for (size_t taskId = 0; taskId < dagsNum.size(); taskId++)
            {
                tasks.push_back(dagsNum[taskId].tasks_[0]);
            }
            return HyperPeriod(tasks);
        }

        static inline std::string Type() { return "Nasri19"; }

        void SyncTaskSet()
        {
            int index = 0;
            for (size_t taskId = 0; taskId < tasksVecNasri_.size(); taskId++)
            {
                for (size_t jobId = 0; jobId < tasksVecNasri_[taskId].tasks_.size(); jobId++)
                {
                    tasksVecNasri_[taskId].tasks_[jobId] = tasks_[index++];
                }
            }
        }
        /**
         * @brief
         *
         * @param taskId
         * @param jobId
         * @param release assume releaseMin=releaseMax
         * @param cost assume costMin=costMax
         * @param deadline assuem priority=deadline
         * @return std::string
         */
        inline std::string LineInTaskCsv(int taskId, int jobId, int release, int cost, int deadline)
        {

            return std::to_string(taskId) + ", " + std::to_string(jobId) + ", " + std::to_string(release) + ", " + std::to_string(release) + ", " + std::to_string(cost) + ", " + std::to_string(cost) + ", " + std::to_string(deadline) + ", " + std::to_string(deadline) + "\n";
        }

        /**
         * @brief
         * This decides the order to iterate all the jobs, and should be followed in the following implementation to avoid confusion; For example:
         * TaskId | JobId | InstanceId | JobIdGlobal
         *        |       |
         *    0   |   0   |     0      |      0
         *    0   |   0   |     1      |      1
         *    0   |   0   |     2      |      2
         *    0   |   1   |     0      |      3
         *    0   |   1   |     1      |      4
         *    0   |   1   |     2      |      5
         *    1   |   0   |     0      |      6
         *    0   |   0   |     1      |      7
         *
         * @param taskId
         * @param jobId
         * @param taskIndex
         * @return int
         */
        int IdJob2Global(int taskId, int jobId, int taskIndex)
        {
            int globalId = 0;
            for (int i = 0; i < taskId; i++)
            {
                for (size_t j = 0; j < tasksVecNasri_[i].tasks_.size(); j++)
                {
                    globalId += hyperPeriod / tasksVecNasri_[i].tasks_[j].period;
                }
            }

            for (int j = 0; j < jobId; j++)
            {
                globalId += hyperPeriod / tasksVecNasri_[taskId].tasks_[j].period;
            }

            return globalId + taskIndex;
        }

        struct Ids
        {
            size_t taskId;
            size_t jobId;
            size_t instanceId;
        };

        Ids IdsGlobal2Job(size_t globalId)
        {
            size_t taskId, jobId;
            for (taskId = 0; taskId < tasksVecNasri_.size(); taskId++)
            {
                for (jobId = 0; jobId < tasksVecNasri_[taskId].tasks_.size(); jobId++)
                {
                    if (globalId >= hyperPeriod / tasksVecNasri_[taskId].tasks_[jobId].period)
                    {
                        globalId -= hyperPeriod / tasksVecNasri_[taskId].tasks_[jobId].period;
                    }
                    else
                    {
                        return {taskId, jobId, globalId};
                    }
                }
            }
            CoutError("Out-of-range in IdsGlobal2Job");
            return {0, 0, 0};
        }

        /**
         * @brief this one doesn't considering taskIndex, mainly used to obtain job-level WCRT
         *
         * @param taskId
         * @param jobId
         * @return int
         */
        int IdJobTaskLevel(int taskId, int jobId)
        {
            int id = 0;
            for (int i = 0; i < taskId; i++)
            {
                id += tasksVecNasri_[i].tasks_.size();
            }
            return id + jobId;
        }

        std::string ConvertTasksetToCsv(bool saveOnDisk = whetherWriteNasriTaskSet)
        {
            SyncTaskSet();

            std::string taskSetStr = "Task ID,     Job ID,          Arrival min,          Arrival max,             Cost min,             Cost max,             Deadline,             Priority\n";
            for (size_t taskId = 0; taskId < tasksVecNasri_.size(); taskId++)
            {
                DAG_Model &dag = tasksVecNasri_[taskId];
                for (size_t jobId = 0; jobId < dag.tasks_.size(); jobId++)
                {
                    for (size_t taskIndex = 0; taskIndex < hyperPeriod / dag.tasks_[0].period; taskIndex++)
                    {
                        int globalId = IdJob2Global(taskId, jobId, taskIndex);
                        taskSetStr += LineInTaskCsv(taskId, globalId, tasks_[0].offset + taskIndex * dag.tasks_[0].period, dag.tasks_[jobId].executionTime, tasks_[0].offset + taskIndex * dag.tasks_[0].period + dag.tasks_[0].deadline);
                    }
                }
            }
            if (saveOnDisk)
            {
                std::ofstream out("outputTask.csv");
                out << taskSetStr;
                out.close();
            }
            return taskSetStr;
        }

        inline std::string LineInDagCsv(int predTId, int predJId, int SuccTId, int succJId)
        {
            return std::to_string(predTId) + ", " + std::to_string(predJId) + ", " + std::to_string(SuccTId) + ", " + std::to_string(succJId) + "\n";
        }
        std::string convertDAGsToCsv(bool saveOnDisk = whetherWriteNasriTaskSet)
        {
            std::string dependStr = "Predecessor TID,	Predecessor JID,	Successor TID, Successor JID\n";
            for (uint taskId = 0; taskId < tasksVecNasri_.size(); taskId++)
            {
                DAG_Model &dag = tasksVecNasri_[taskId];
                rt_num_opt::edge_iter ei, ei_end;
                auto vertex2index_ = boost::get(boost::vertex_name, dag.graph_);
                for (tie(ei, ei_end) = boost::edges(dag.graph_); ei != ei_end; ++ei)
                {
                    int fromJId = vertex2index_[boost::source(*ei, dag.graph_)];
                    int toJId = vertex2index_[boost::target(*ei, dag.graph_)];
                    for (uint taskIndex = 0; taskIndex < hyperPeriod / dag.tasks_[0].period; taskIndex++)
                    {
                        dependStr += LineInDagCsv(taskId, IdJob2Global(taskId, fromJId, taskIndex), taskId, IdJob2Global(taskId, toJId, taskIndex));
                    }
                }
            }
            if (saveOnDisk)
            {
                std::ofstream out("outputDAG.csv");
                out << dependStr;
                out.close();
            }
            return dependStr;
        }
    };

    DAG_Nasri19 ReadDAGNasri19_Tasks(std::string path) // std::string priorityType = "orig"
    {
        std::vector<rt_num_opt::DAG_Model> dagsNum = ReadDAG_NasriFromYaml(path);

        return DAG_Nasri19(dagsNum);
    }
}