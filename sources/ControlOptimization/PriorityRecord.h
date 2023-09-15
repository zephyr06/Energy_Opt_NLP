
#pragma once
#include <algorithm>
#include <unordered_map>
#include <vector>

#include "sources/Tools/testMy.h"

// obj_sorted = obj + w * Barrier(D - R)
// gradient = k_obj + w / (D-R)
struct TaskPriority {
    TaskPriority() {}
    TaskPriority(int i, double p) : task_index(i), priority(p) {}
    int task_index;
    double priority;
};
bool operator==(const TaskPriority &p1, const TaskPriority &p2) {
    return p1.task_index == p2.task_index && p1.priority == p2.priority;
}
bool operator!=(const TaskPriority &p1, const TaskPriority &p2) {
    return !(p1 == p2);
}

struct PriorityRecord {
    PriorityRecord() {}
    PriorityRecord(int id, int priority) : task_id(id), priority(priority) {}

    inline bool operator==(const PriorityRecord &other) const {
        return task_id == other.task_id && priority == other.priority;
    }
    int task_id;
    int priority;
};
template <>
struct std::hash<PriorityRecord> {
    size_t operator()(const PriorityRecord &p) const {
        // Combine the hash values of the member variables
        size_t hash1 = std::hash<int>()(p.task_id);
        size_t hash2 = std::hash<int>()(p.priority);

        // Combine the hashes using XOR or another method
        return hash1 ^ (hash2 << 1);
    }
};

int FindPriority(int task_id, const std::vector<TaskPriority> &tasks_w_pri) {
    for (uint i = 0; i < tasks_w_pri.size(); i++) {
        if (tasks_w_pri[i].task_index == task_id)
            return i;
    }
    CoutError("Didn't find task in FindPriority");
    return -1;
}

class PriorityAssignmentRecord {
   public:
    PriorityAssignmentRecord() {}

    inline void AddFailedRecord(int task_id, int priority) {
        failed_pa_adjust[PriorityRecord(task_id, priority)]++;
    }

    inline void AddFailedRecord(int task_id,
                                const std::vector<TaskPriority> &tasks_w_pri) {
        AddFailedRecord(task_id, FindPriority(task_id, tasks_w_pri));
    }

    void AddSuccessRecord(int task_id, int priority) {
        PriorityRecord record(task_id, priority);
        if (failed_pa_adjust.count(record)) {
            failed_pa_adjust[record]--;
            if (failed_pa_adjust[record] == 0)
                failed_pa_adjust.erase(record);
        }
    }
    inline void AddSuccessRecord(int task_id,
                                 const std::vector<TaskPriority> &tasks_w_pri) {
        AddSuccessRecord(task_id, FindPriority(task_id, tasks_w_pri));
    }

    // return false if the task can be skipped, true otherwise
    bool EvaluateRecordHistory(int task_id, int priority, int threshold) {
        PriorityRecord record(task_id, priority);
        if (failed_pa_adjust.count(record)) {
            if (failed_pa_adjust[record] >= threshold)
                return false;
        }
        return true;
    }
    // return false if the task can be skipped, true otherwise
    inline bool EvaluateRecordHistory(
        int task_id, const std::vector<TaskPriority> &tasks_w_pri,
        int threshold = 2) {
        return EvaluateRecordHistory(
            task_id, FindPriority(task_id, tasks_w_pri), threshold);
    }
    // data
    std::unordered_map<PriorityRecord, int> failed_pa_adjust;
};
