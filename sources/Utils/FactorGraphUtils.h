#pragma once
#include <gtsam/inference/Symbol.h>

#include "sources/Utils/GlobalVariables.h"
namespace rt_num_opt {
bool ContainFalse(EliminationRecord &eliminationRecord) {
    for (uint i = 0; i < eliminationRecord.record.size(); i++) {
        if (eliminationRecord[i].type == EliminationType::Not) {
            return true;
        }
    }
    return false;
}
/**
 * @brief generate keys for factor graph modeling the control problem
 * keys' attributes can be extracted via key.chr(). key.index()
 *
 * @param idtask
 * @param type
 * @return gtsam::Symbol
 */
gtsam::Symbol GenerateKey(int idtask, std::string type) {
    if (type == "period") {
        return gtsam::Symbol('t', idtask);
    } else if (type == "response") {
        return gtsam::Symbol('r', idtask);
    } else if (type == "executionTime") {
        return gtsam::Symbol('c', idtask);
    } else {
        CoutError("Unrecognized type in GenerateControlKey");
        gtsam::Symbol key('a', idtask);
        return key;
    }
}

int AnalyzeKey(gtsam::Symbol key) { return key.index(); }

/**
 * @brief perform customized quotient operation for two double type numbers
 * return result is min_k |a - k b|, where k is an integer, a & b >0;
 *
 * Brute force solution, find minimum remainder by finding minimum rem=(a-=b)
 * @param a
 * @param b
 * @return double
 */
double QuotientDouble(double a, double b) {
    double remainder = a - int(a / b) * b;
    return min(remainder, abs(remainder - b));
}

template <typename T>
bool Equals(std::vector<T> &v1, std::vector<T> &v2) {
    if (v1.size() != v2.size()) {
        return false;
    }
    for (uint i = 0; i < v1.size(); i++) {
        if (v1[i] != v2[i]) {
            return false;
        }
    }
    return true;
}

/* only used in Reorder function */
struct TaskAugment {
    Task task_;
    double coeff_T;
    double coeff_R;
    double coeff_R2;
    TaskAugment(Task &task, double coeffT, double coeffR, double coeffR2)
        : task_(task), coeff_T(coeffT), coeff_R(coeffR), coeff_R2(coeffR2) {}

    double sort_metric() const {
        return task_.period + control_sort_exec_weight * task_.executionTime +
               control_sort_obj_coeff_weight * (coeff_T + coeff_R);
    }

    static bool Compare(const TaskAugment &t1, const TaskAugment &t2) {
        if (enableReorder < 1) {
            return t1.task_.period < t2.task_.period;
        } else {
            return t1.sort_metric() < t2.sort_metric();
        }
    }
};
/* in-place adjustment of tasks' order; highest order being the first in tasks;
 only used in workshop experiments
 */
void Reorder_EachTaskHas2Coeff(TaskSet &tasks, VectorDynamic &coeff) {
    std::vector<TaskAugment> tasksAug;
    tasksAug.reserve(tasks.size());
    for (uint i = 0; i < tasks.size(); i++) {
        tasksAug.push_back({tasks[i], coeff(i * 2), coeff(i * 2 + 1), 0});
    }
    stable_sort(tasksAug.begin(), tasksAug.end(), TaskAugment::Compare);
    for (uint i = 0; i < tasks.size(); i++) {
        tasks[i] = tasksAug[i].task_;
        coeff(2 * i) = tasksAug[i].coeff_T;
        coeff(2 * i + 1) = tasksAug[i].coeff_R;
    }
}
void Reorder_EachTaskHas3Coeff(TaskSet &tasks, VectorDynamic &coeff) {
    std::vector<TaskAugment> tasksAug;
    tasksAug.reserve(tasks.size());
    for (uint i = 0; i < tasks.size(); i++) {
        tasksAug.push_back(
            {tasks[i], coeff(i * 3), coeff(i * 3 + 1), coeff(i * 3 + 2)});
    }
    stable_sort(tasksAug.begin(), tasksAug.end(), TaskAugment::Compare);
    for (uint i = 0; i < tasks.size(); i++) {
        tasks[i] = tasksAug[i].task_;
        coeff(3 * i) = tasksAug[i].coeff_T;
        coeff(3 * i + 1) = tasksAug[i].coeff_R;
        coeff(3 * i + 2) = tasksAug[i].coeff_R2;
    }
}
void Reorder(TaskSet &tasks, VectorDynamic &coeff) {
    int n_coeff = coeff.rows();
    int n = tasks.size();
    if (n_coeff / n == 2)
        Reorder_EachTaskHas2Coeff(tasks, coeff);
    else if (n_coeff / n == 3)
        Reorder_EachTaskHas3Coeff(tasks, coeff);
    else
        CoutError("Incorrect input dimension in Reorder function!");
}

/* @brief return a std::string with expected precision by adding leading 0 */
std::string to_string_precision(int a, int precision) {
    return std::string(4 - min(4, std::to_string(a).length()), '0') +
           std::to_string(a);
}

template <typename T>
void print(const std::vector<T> &vec) {
    for (auto x : vec) {
        std::cout << x << ", ";
    }
}

}  // namespace rt_num_opt