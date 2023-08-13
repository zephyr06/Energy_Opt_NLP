#pragma once

#include "sources/TaskModel/DAG_Task.h"
#include "sources/TaskModel/Tasks.h"

namespace rt_num_opt {
struct TaskSetNormal {
    TaskSet tasks_;
    int N;
    std::vector<rt_num_opt::DAG_Model>
        tasksVecNasri_;  // added for a compile issue, only used in its child
                         // class

    TaskSetNormal() { ; }
    TaskSetNormal(const TaskSet &tasks) : tasks_(tasks), N(tasks.size()) {}
    static inline std::string Type() { return "normal"; }
    void UpdateTaskSet(TaskSet &tasks) {
        tasks_ = tasks;
        N = tasks.size();
    }
    void UpdateTaskSetParameter(int index, double value,
                                std::string parameter = "executionTime") {
        if (parameter == "executionTime") {
            tasks_[index].executionTime = value;
        } else if (parameter == "period") {
            tasks_[index].period = value;
        } else {
            CoutError("Please provide Update parameter for " + parameter);
        }
    }

    inline void UpdateTaskSet(const TaskSet &tasks) { tasks_ = tasks; }
    Task operator[](size_t i) { return tasks_[i]; }

    size_t size() { return tasks_.size(); }
};

template <typename T>
VectorDynamic GetParameterVD(const TaskSetNormal &taskset,
                             std::string parameterType) {
    return GetParameterVD<T>(taskset.tasks_, parameterType);
}

void UpdateTaskSetExecutionTime(TaskSet &taskSet,
                                const VectorDynamic &executionTimeVec,
                                int lastTaskDoNotNeedOptimize = -1) {
    int N = taskSet.size();

    for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
        taskSet[i].executionTime =
            executionTimeVec.coeff(i - lastTaskDoNotNeedOptimize - 1, 0);
}
void UpdateTaskSetExecutionTime(TaskSetNormal &taskSet,
                                const VectorDynamic &executionTimeVec,
                                int lastTaskDoNotNeedOptimize = -1) {
    return UpdateTaskSetExecutionTime(taskSet.tasks_, executionTimeVec,
                                      lastTaskDoNotNeedOptimize);
}

void UpdateTaskSetPeriod(TaskSetNormal &taskSet,
                         const VectorDynamic &periodVec) {
    return UpdateTaskSetPeriod(taskSet.tasks_, periodVec);
}
}  // namespace rt_num_opt