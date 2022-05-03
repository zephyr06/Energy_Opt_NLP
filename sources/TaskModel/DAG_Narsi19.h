#pragma once

#include "dagSched/tests.h"

#include "sources/TaskModel/DAG_Task.h"
#include "sources/TaskModel/ReadWriteYaml.h"

namespace rt_num_opt
{
    struct DAG_Narsi19 : public TaskSetNormal
    {
        std::vector<rt_num_opt::DAG_Model> tasksVecNarsi_; // it mainly stores the graphical structure of DAGs
        std::vector<int> nodeSizes_;
        std::string dagCsv;

        DAG_Narsi19() {}
        DAG_Narsi19(std::vector<rt_num_opt::DAG_Model> &dagsNum)
        {
            tasksVecNarsi_ = dagsNum;
            for (size_t i = 0; i < dagsNum.size(); i++)
            {
                nodeSizes_.push_back(dagsNum[i].tasks_.size());
                N += dagsNum[i].tasks_.size();
                for (size_t j = 0; j < dagsNum[i].tasks_.size(); j++)
                {
                    tasks_.push_back(dagsNum[i].tasks_[j]);
                }
            }
            dagCsv = convertDAGsToCsv();
        }

        static inline std::string Type() { return "Nasri"; }

        void SyncTaskSet()
        {
            int index = 0;
            for (uint i = 0; i < tasksVecNarsi_.size(); i++)
            {
                for (uint j = 0; j < tasksVecNarsi_[i].tasks_.size(); j++)
                {
                    tasksVecNarsi_[i].tasks_[j] = tasks_[index++];
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
        int IdJob2Global(int taskId, int jobId, int taskIndex)
        {
            int globalId = 0;
            auto hyperPeriod = HyperPeriod(tasks_);
            for (int i = 0; i < taskId; i++)
            {
                for (size_t j = 0; j < tasksVecNarsi_[i].tasks_.size(); j++)
                {
                    globalId += hyperPeriod / tasksVecNarsi_[i].tasks_[j].period;
                }
            }

            for (int j = 0; j < jobId; j++)
            {
                globalId += hyperPeriod / tasksVecNarsi_[taskId].tasks_[j].period;
            }

            return globalId + taskIndex;
        }
        std::string ConvertTasksetToCsv(bool saveOnDisk = true)
        {
            SyncTaskSet();

            auto hyperPeriod = HyperPeriod(tasks_);
            std::string taskSetStr = "   Task ID,     Job ID,          Arrival min,          Arrival max,             Cost min,             Cost max,             Deadline,             Priority\n";
            for (size_t taskId = 0; taskId < tasksVecNarsi_.size(); taskId++)
            {
                DAG_Model &dag = tasksVecNarsi_[taskId];
                for (size_t taskIndex = 0; taskIndex < hyperPeriod / dag.tasks_[0].period; taskIndex++)
                {
                    for (size_t jobId = 0; jobId < dag.tasks_.size(); jobId++)
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
        std::string convertDAGsToCsv(bool saveOnDisk = true)
        {
            auto hyperPeriod = HyperPeriod(tasks_);
            std::string dependStr = "Predecessor TID,	Predecessor JID,	Successor TID, Successor JID\n";
            for (uint taskId = 0; taskId < tasksVecNarsi_.size(); taskId++)
            {
                DAG_Model &dag = tasksVecNarsi_[taskId];
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

    // DAG_Narsi19 ReadDAGNarsi19_Tasks(std::string path) // std::string priorityType = "orig"
    // {
    //     std::vector<rt_num_opt::DAG_Model> dagsNum = ReadDAG_NarsiFromYaml(path);
    //     return DAG_Narsi19(dagsNum);
    // }
}