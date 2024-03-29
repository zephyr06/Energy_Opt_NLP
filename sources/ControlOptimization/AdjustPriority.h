

#pragma once
#include <string>

#include "sources/ControlOptimization/FactorGraph_Nasri19.h"
#include "sources/ControlOptimization/PriorityRecord.h"

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

// alternative: numerical graident based on FactorGraph_Nasri19::LinearObj()
inline double GetObjGradient(int task_index, const TaskSet &tasks,
                             const VectorDynamic &coeff,
                             const VectorDynamic &rta, double weight) {
    return coeff(3 * task_index + 1) -
           weight / (tasks[task_index].deadline - rta(task_index)) +
           Obj_Pow * pow(rta(task_index), Obj_Pow - 1) *
               coeff(3 * task_index + 2) * (Obj_Pow - 1);
}

bool UpdateAllTasksPriority(DAG_Nasri19 &dag_tasks,
                            const std::vector<TaskPriority> &priority_assign) {
    bool if_changed = false;
    for (uint i = 0; i < priority_assign.size(); i++) {
        if (dag_tasks.tasks_[priority_assign[i].task_index].priority != int(i))
            if_changed = true;
        dag_tasks.tasks_[priority_assign[i].task_index].priority = i;
    }
    dag_tasks.UpdateTasksVecNasri_();
    return if_changed;
}

double GetSchedulabilityObj(const DAG_Nasri19 &dag_tasks,
                            const VectorDynamic &rta, double weight,
                            double log_base = exp(1)) {
    double sum = 0;
    for (uint i = 0; i < dag_tasks.tasks_.size(); i++) {
        double diff = dag_tasks.tasks_[i].deadline - rta(i);
        if (diff <= 0)
            // CoutError("Unschedulable GetSchedulabilityObj!");
            sum += -1e50;
        else
            sum += log(diff) / log(log_base);
    }
    return sum * weight;
}

// TODO: how to describe priority assignments more elegant?
double GetObjAfterAdjustPriority(
    const DAG_Nasri19 &dag_tasks, const VectorDynamic &coeff,
    const std::vector<TaskPriority> &priority_assign, double weight) {
    DAG_Nasri19 dag_tasks_curr = dag_tasks;
    UpdateAllTasksPriority(dag_tasks_curr, priority_assign);
    double obj = FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(
        dag_tasks_curr, coeff);

    RTA_Nasri19 r(dag_tasks_curr);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    double schedulability_obj =
        GetSchedulabilityObj(dag_tasks_curr, rta, weight);
    return obj + schedulability_obj;
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
void PrintPriorityAssignment(const std::vector<TaskPriority> &tasks_w_pri) {
    std::cout << "Current priority sequence:\n";
    for (auto x : tasks_w_pri) {
        std::cout << "(" << x.task_index << ") ";  //  << ", " << x.priority
    }
    std::cout << "\n";
}

size_t FindTaskIndexFromPAOrder(int task_id,
                                const std::vector<TaskPriority> &tasks_w_pri) {
    for (uint i = 0; i < tasks_w_pri.size(); i++)
        if (tasks_w_pri[i].task_index == task_id)
            return i;
    CoutError("didn't find task in FindTaskIndexFromPAOrder!");
    return -1;
}
// return true if a task's current priority assignment is worse than those
// indicated by its gradient
// NOTICE: tasks_w_gra is ordered inversely, i.e., higher priority tasks appear
// last
bool WhetherTaskNeedPAChange(int task_id,
                             const std::vector<TaskPriority> &tasks_w_pri,
                             const std::vector<TaskPriority> &tasks_w_gra,
                             int significant_difference) {
    // std::reverse(tasks_w_gra.begin(), tasks_w_gra.end());
    int id_index_priority = FindTaskIndexFromPAOrder(task_id, tasks_w_pri);
    int id_index_gra =
        tasks_w_gra.size() - 1 - FindTaskIndexFromPAOrder(task_id, tasks_w_gra);
    return id_index_priority - id_index_gra > significant_difference;
}

void AssignPriorityBasedOnlyGradient(DAG_Nasri19 &dag_tasks,
                                     const VectorDynamic &coeff) {
    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    std::vector<TaskPriority> tasks_w_gra =
        SortPriorityBasedGradient(dag_tasks.tasks_, coeff, rta, 0);
    std::reverse(tasks_w_gra.begin(), tasks_w_gra.end());
    UpdateAllTasksPriority(dag_tasks, tasks_w_gra);
}

// higher priority tasks appear first, lower priority task appear later
std::vector<TaskPriority> ReorderWithGradient(
    const DAG_Nasri19 &dag_tasks, const VectorDynamic &coeff, double weight,
    PriorityAssignmentRecord &pa_record,
    double pa_change_threshold = Priority_assignment_adjustment_threshold) {
    const TaskSet &tasks = dag_tasks.tasks_;
    std::vector<TaskPriority> tasks_w_pri = GetPriorityVector(dag_tasks);

    if (debugMode == 1) {
        std::cout << "Initial assignment: ";
        PrintPriorityAssignment(tasks_w_pri);
    }
    double obj_base =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dag_tasks, coeff);

    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    std::vector<TaskPriority> tasks_w_gra =
        SortPriorityBasedGradient(tasks, coeff, rta, weight);
    std::vector<TaskPriority> tasks_w_gra_copy = tasks_w_gra;

    int count_skipped_adjust = 0;
    int count_effective_try = 0;
    // adjust priority iteratively
    while (!tasks_w_gra.empty()) {
        int task_id_curr = tasks_w_gra.back().task_index;
        int significant_difference_threshold =
            pa_change_threshold * tasks_w_pri.size();
        if (!WhetherTaskNeedPAChange(task_id_curr, tasks_w_pri,
                                     tasks_w_gra_copy,
                                     significant_difference_threshold) ||
            (!pa_record.EvaluateRecordHistory(task_id_curr, tasks_w_pri))) {
            tasks_w_gra.pop_back();
            count_skipped_adjust++;
            continue;
        } else {
            // std::cout << "Try adjusting PA:\n";
            tasks_w_gra.pop_back();
            double obj_curr;
            while (true) {
                std::vector<TaskPriority> tasks_w_pri_curr = tasks_w_pri;
                IncreasePriority(task_id_curr, tasks_w_pri_curr);
                obj_curr = GetObjAfterAdjustPriority(dag_tasks, coeff,
                                                     tasks_w_pri_curr, weight);
                if (obj_curr >= obj_base) {
                    pa_record.AddFailedRecord(task_id_curr, tasks_w_pri);
                    break;
                } else {
                    if (debugMode == 1) {
                        std::cout << "\nPriority change: task " << task_id_curr
                                  << "'s priority increases, and improves the "
                                     "objective function from "
                                  << obj_base << " to " << obj_curr << "\n";
                        std::cout << "Current RTA:\n";
                        DAG_Nasri19 dag_tasks_curr = dag_tasks;
                        UpdateAllTasksPriority(dag_tasks_curr,
                                               tasks_w_pri_curr);
                        std::cout << GetNasri19RTA(dag_tasks_curr) << "\n";
                        PrintPriorityAssignment(tasks_w_pri_curr);
                    }
                    pa_record.AddSuccessRecord(task_id_curr, tasks_w_pri);
                    obj_base = obj_curr;
                    tasks_w_pri = tasks_w_pri_curr;
                    count_effective_try++;
                }
            }
        }
    }
    if (debugMode == 1) {
        std::cout << "Final assignment: ";
        PrintPriorityAssignment(tasks_w_pri);
    }
    std::cout << "ReorderWithGradient, skipped adjustments: "
              << count_skipped_adjust << ", "
              << "actual adjust: " << count_effective_try << ", "
              << tasks_w_pri.size() << "\n";
    return tasks_w_pri;
}

}  // namespace rt_num_opt
