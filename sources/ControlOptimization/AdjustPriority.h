

#pragma once
#include <string>

#include "sources/ControlOptimization/FactorGraph_Nasri19.h"

namespace rt_num_opt {
// require x>0
double BarrierReorder(double x) {
    if (x > 0)
        return log(x);
    else
        return 1e8 * -1 * x;
}
double GradientBarrierReorder(double x) {
    if (x > 0)
        return 1 / x;
    else
        return -1 * 1e8;
}
// obj_sorted = obj + w * Barrier(D - R)
// gradient = k_obj + w / (D-R)
struct TaskPriority {
    TaskPriority() {}
    TaskPriority(int i, double p) : task_index(i), priority(p) {}
    int task_index;
    double priority;
};
double GetObjGradient(int task_index, const TaskSet &tasks,
                      const VectorDynamic &coeff, const VectorDynamic &rta,
                      double weight) {
    return coeff(2 * task_index + 1) +
           weight / (tasks[task_index].deadline - rta(task_index));
}

void UpdateAllTasksPriority(DAG_Nasri19 &dag_tasks,
                            const std::vector<TaskPriority> &priority_assign) {
    for (uint i = 0; i < priority_assign.size(); i++) {
        dag_tasks.tasks_[priority_assign[i].task_index].priority = i;
    }
}

// TODO: how to describe priority assignments more elegant?
double GetObjAfterAdjustPriority(
    const DAG_Nasri19 &dag_tasks, const VectorDynamic &coeff,
    const std::vector<TaskPriority> &priority_assign) {
    DAG_Nasri19 dag_tasks_curr = dag_tasks;
    UpdateAllTasksPriority(dag_tasks_curr, priority_assign);
    return FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dag_tasks_curr,
                                                               coeff);
}

void IncreasePriority(int task_index, std::vector<TaskPriority> &tasks_w_pri) {
    // if task_index has highest priority, return
    if (tasks_w_pri[0].task_index == task_index)
        return;
    for (uint i = 1; i < tasks_w_pri.size(); i++) {
        if (tasks_w_pri[i].task_index == task_index) {
            auto temp = tasks_w_pri[i - 1];
            tasks_w_pri[i - 1] = tasks_w_pri[i];
            tasks_w_pri[i] = temp;
            return;
        }
    }
    CoutError("Didn't find the given task_index: " +
              std::to_string(task_index));
}
void DecreasePriority(int task_index, std::vector<TaskPriority> &tasks_w_pri) {
    // if task_index has lowest priority, return
    if (tasks_w_pri.back().task_index == task_index)
        return;
    for (int i = static_cast<int>(tasks_w_pri.size()) - 2; i >= 0; i--) {
        if (tasks_w_pri[i].task_index == task_index) {
            auto temp = tasks_w_pri[i + 1];
            tasks_w_pri[i + 1] = tasks_w_pri[i];
            tasks_w_pri[i] = temp;
            return;
        }
    }
    CoutError("Didn't find the given task_index: " +
              std::to_string(task_index));
}

std::vector<TaskPriority> GetPriorityVector(const DAG_Nasri19 &dag_tasks) {
    const TaskSet &tasks = dag_tasks.tasks_;
    std::vector<TaskPriority> tasks_w_pri(tasks.size());
    for (uint i = 0; i < tasks.size(); i++)
        tasks_w_pri[i] = TaskPriority(i, tasks[i].priority);
    sort(tasks_w_pri.begin(), tasks_w_pri.end(),
         [](const TaskPriority &t1, const TaskPriority &t2) {
             if (t1.priority != t2.priority)
                 return t1.priority < t2.priority;
             else
                 return t1.task_index < t2.task_index;
             //  CoutError("Should not happen!");
             //  return true;
         });
    return tasks_w_pri;
}

// NOTE: in this special function, for the convenince of manipulation,
// tasks with the biggest potential appear last, for the convenience of 'pop'
std::vector<TaskPriority> SortPriorityBasedGradient(const TaskSet &tasks,
                                                    const VectorDynamic &coeff,
                                                    const VectorDynamic &rta,
                                                    double weight) {
    std::vector<TaskPriority> tasks_w_gra(tasks.size());
    for (uint i = 0; i < tasks.size(); i++)
        tasks_w_gra[i] =
            TaskPriority(i, GetObjGradient(i, tasks, coeff, rta, weight));
    sort(tasks_w_gra.begin(), tasks_w_gra.end(),
         [](const TaskPriority &t1, const TaskPriority &t2) {
             return t1.priority < t2.priority;
         });
    return tasks_w_gra;
}

// higher priority tasks appear first, lower priority task appear later
std::vector<TaskPriority> ReorderWithGradient(const DAG_Nasri19 &dag_tasks,
                                              const VectorDynamic &coeff,
                                              double weight) {
    const TaskSet &tasks = dag_tasks.tasks_;
    std::vector<TaskPriority> tasks_w_pri = GetPriorityVector(dag_tasks);
    double obj_base =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dag_tasks, coeff);

    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    std::vector<TaskPriority> tasks_w_gra =
        SortPriorityBasedGradient(tasks, coeff, rta, weight);

    // adjust priority iteratively
    while (!tasks_w_gra.empty()) {
        int task_id_curr = tasks_w_gra.back().task_index;
        tasks_w_gra.pop_back();
        double obj_curr;
        while (true) {
            IncreasePriority(task_id_curr, tasks_w_pri);
            obj_curr = GetObjAfterAdjustPriority(dag_tasks, coeff, tasks_w_pri);
            if (obj_curr >= obj_base) {  // if IncreasePriority happens
                if (tasks_w_pri[0].task_index != task_id_curr)
                    DecreasePriority(task_id_curr, tasks_w_pri);
                break;
            } else {
                if (debugMode == 1) {
                    std::cout << "Priority change: task " << task_id_curr
                              << "'s priority increases, and improves the "
                                 "objective function from "
                              << obj_base << " to " << obj_curr << "\n";
                }
                obj_base = obj_curr;
            }
        }
    }
    return tasks_w_pri;
}

}  // namespace rt_num_opt