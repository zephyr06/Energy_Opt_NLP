#pragma once
// The task model is from the paper "Response-Time Analysis of
// Limited-Preemptive Parallel DAG Tasks Under Global Scheduling"

#include "sources/TaskModel/DAG_Task.h"
#include "sources/TaskModel/ReadWriteYaml.h"

namespace rt_num_opt {
struct DAG_Nasri19 : public TaskSetNormal {
    DAG_Nasri19() {}

    DAG_Nasri19(const std::vector<rt_num_opt::DAG_Model> &dagsNum) {
        tasksVecNasri_ = dagsNum;
        RoundPeriod();

        for (size_t taskId = 0; taskId < tasksVecNasri_.size(); taskId++) {
            for (size_t nodeId = 0;
                 nodeId < tasksVecNasri_[taskId].tasks_.size(); nodeId++) {
                tasks_.push_back(tasksVecNasri_[taskId].tasks_[nodeId]);
            }
        }
        InitializePriority();
        hyperPeriod = HyperPeriod(tasks_);
        N = tasks_.size();
        dagCsv = convertDAGsToCsv();
    }

    size_t inline SizeDag() const { return tasksVecNasri_.size(); }

    size_t SizeNode() const {
        size_t size = 0;
        for (uint i = 0; i < tasksVecNasri_.size(); i++)
            size += tasksVecNasri_[i].tasks_.size();
        return size;
    }
    inline const DAG_Model &getDag(size_t index) const {
        return tasksVecNasri_[index];
    }

    static inline std::string Type() { return "Nasri19"; }

    // based on rate-monotonic
    void InitializePriority() {
        for (Task &task_curr : tasks_) task_curr.priority = task_curr.period;
        UpdateTasksVecNasri_();
    }
    bool IsHyperPeriodOverflow() const { return hyperPeriod >= INT_MAX; }
    // TODO: fill in!
    void UpdatePriority(const std::vector<int> &task_id_seq) { ; }

    void AdjustPeriod(size_t dag_index, double delta) {
        UpdatePeriod(dag_index, delta + tasksVecNasri_[dag_index].GetPeriod());
    }

    void UpdatePeriod(size_t dag_index, double period) {
        DAG_Model &dag_curr = tasksVecNasri_[dag_index];
        // prevent some unrealistic values
        if (period <= 0)
            period = 1e0;

        for (uint i = 0; i < dag_curr.tasks_.size(); i++) {
            dag_curr.tasks_[i].period = period;
            if (period >=
                dag_curr.tasks_[i]
                    .executionTime)  // otherwise, this is unschedulable, and we
                                     // hope the system simply return it
                dag_curr.tasks_[i].RoundPeriod();
        }
        UpdateTasksFromVecNasri_();
        hyperPeriod = HyperPeriod(tasks_);
    }

    void UpdateTasksFromVecNasri_() {
        uint node_count = 0;
        for (uint i = 0; i < tasksVecNasri_.size(); i++) {
            DAG_Model &dag_curr = tasksVecNasri_[i];
            for (uint j = 0; j < dag_curr.tasks_.size(); j++) {
                tasks_[node_count++].period = dag_curr.tasks_[j].period;
            }
        }
    }

    // round for tasksVecNasri_
    void RoundPeriod() {
        for (uint i = 0; i < tasksVecNasri_.size(); i++) {
            tasksVecNasri_[i].RoundPeriod();
        }
        for (Task &task_curr : tasks_) task_curr.RoundPeriod();
    }

    void UpdateTaskSet(const TaskSet &tasks) {
        tasks_ = tasks;
        UpdateTasksVecNasri_();
        RoundPeriod();
        hyperPeriod = HyperPeriod(tasks_);
    }

    void UpdateTasksVecNasri_() {
        int index = 0;
        for (size_t taskId = 0; taskId < tasksVecNasri_.size(); taskId++) {
            for (size_t jobId = 0; jobId < tasksVecNasri_[taskId].tasks_.size();
                 jobId++) {
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
    inline std::string LineInTaskCsv(int taskId, int jobId, int release,
                                     int cost, int deadline, int priority) {
        return std::to_string(taskId) + ", " + std::to_string(jobId) + ", " +
               std::to_string(release) + ", " + std::to_string(release) + ", " +
               std::to_string(cost) + ", " + std::to_string(cost) + ", " +
               std::to_string(deadline) + ", " + std::to_string(priority) +
               "\n";
    }

    /**
     * @brief
     * This decides the order to iterate all the jobs, and should be followed in
     * the following implementation to avoid confusion; For example: TaskId |
     * JobId | InstanceId | JobIdGlobal |       | 0   |   0   |     0      | 0
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
    int IdJob2Global(int taskId, int jobId, int taskIndex) {
        int globalId = 0;
        for (int i = 0; i < taskId; i++) {
            for (size_t j = 0; j < tasksVecNasri_[i].tasks_.size(); j++) {
                globalId += hyperPeriod / tasksVecNasri_[i].tasks_[j].period;
            }
        }

        for (int j = 0; j < jobId; j++) {
            globalId += hyperPeriod / tasksVecNasri_[taskId].tasks_[j].period;
        }

        return globalId + taskIndex;
    }

    struct Ids {
        size_t taskId;
        size_t jobId;
        size_t instanceId;
    };

    Ids IdsGlobal2Job(size_t globalId) {
        size_t taskId, jobId;
        for (taskId = 0; taskId < tasksVecNasri_.size(); taskId++) {
            for (jobId = 0; jobId < tasksVecNasri_[taskId].tasks_.size();
                 jobId++) {
                if (globalId >=
                    hyperPeriod / tasksVecNasri_[taskId].tasks_[jobId].period) {
                    globalId -= hyperPeriod /
                                tasksVecNasri_[taskId].tasks_[jobId].period;
                } else {
                    return {taskId, jobId, globalId};
                }
            }
        }
        CoutError("Out-of-range in IdsGlobal2Job");
        return {0, 0, 0};
    }

    /**
     * @brief this one doesn't considering taskIndex, mainly used to obtain
     * job-level WCRT
     *
     * @param taskId
     * @param jobId
     * @return int
     */
    int IdJobTaskLevel(int taskId, int jobId) {
        int id = 0;
        for (int i = 0; i < taskId; i++) {
            id += tasksVecNasri_[i].tasks_.size();
        }
        return id + jobId;
    }

    std::string ConvertTasksetToCsv(
        bool saveOnDisk = whetherWriteNasriTaskSet) {
        UpdateTasksVecNasri_();

        std::string taskSetStr =
            "Task ID,     Job ID,          Arrival min,          Arrival max,  "
            "           Cost min,             Cost max,             Deadline,  "
            "           Priority\n";
        for (size_t taskId = 0; taskId < tasksVecNasri_.size(); taskId++) {
            DAG_Model &dag = tasksVecNasri_[taskId];
            for (size_t nodeId = 0; nodeId < dag.tasks_.size(); nodeId++) {
                Task &task_curr = tasksVecNasri_[taskId].tasks_[nodeId];
                int period = task_curr.period;
                // int period = tasksVecNasri_[taskId].tasks_[0].period;
                for (size_t node_job_index = 0;
                     node_job_index < size_t(hyperPeriod / period);
                     node_job_index++) {
                    int globalId = IdJob2Global(taskId, nodeId, node_job_index);
                    int priority = task_curr.priority;
                    if (priority ==
                        -1)  // not set, so switch to absolute deadline
                        priority = task_curr.offset + node_job_index * period +
                                   task_curr.deadline;
                    taskSetStr += LineInTaskCsv(
                        taskId, globalId,
                        task_curr.offset + node_job_index * period,
                        task_curr.executionTime,
                        task_curr.offset + node_job_index * period +
                            task_curr.deadline,
                        priority);
                }
            }
        }
        if (saveOnDisk) {
            std::ofstream out("outputTask.csv");
            out << taskSetStr;
            out.close();
        }
        return taskSetStr;
    }

    inline std::string LineInDagCsv(int predTId, int predJId, int SuccTId,
                                    int succJId) {
        return std::to_string(predTId) + ", " + std::to_string(predJId) + ", " +
               std::to_string(SuccTId) + ", " + std::to_string(succJId) + "\n";
    }

    std::string convertDAGsToCsv(bool saveOnDisk = whetherWriteNasriTaskSet) {
        std::string dependStr =
            "Predecessor TID,	Predecessor JID,	Successor TID, "
            "Successor JID\n";
        for (uint taskId = 0; taskId < tasksVecNasri_.size(); taskId++) {
            DAG_Model &dag = tasksVecNasri_[taskId];
            rt_num_opt::edge_iter ei, ei_end;
            auto vertex2index_ = boost::get(boost::vertex_name, dag.graph_);
            for (tie(ei, ei_end) = boost::edges(dag.graph_); ei != ei_end;
                 ++ei) {
                int fromJId = vertex2index_[boost::source(*ei, dag.graph_)];
                int toJId = vertex2index_[boost::target(*ei, dag.graph_)];
                for (uint taskIndex = 0;
                     taskIndex < hyperPeriod / dag.tasks_[0].period;
                     taskIndex++) {
                    dependStr += LineInDagCsv(
                        taskId, IdJob2Global(taskId, fromJId, taskIndex),
                        taskId, IdJob2Global(taskId, toJId, taskIndex));
                }
            }
        }
        if (saveOnDisk) {
            std::ofstream out("outputDAG.csv");
            out << dependStr;
            out.close();
        }
        return dependStr;
    }

    // data members
    long long int hyperPeriod;
    std::vector<rt_num_opt::DAG_Model>
        tasksVecNasri_;  // it mainly stores the graphical structure of DAGs
    // std::vector<int> nodeSizes_;
    std::string dagCsv;
};

inline DAG_Nasri19 ReadDAGNasri19_Tasks(
    std::string path)  // std::string priorityType = "orig"
{
    std::vector<rt_num_opt::DAG_Model> dagsNum = ReadDAG_NasriFromYaml(path);

    return DAG_Nasri19(dagsNum);
}
}  // namespace rt_num_opt