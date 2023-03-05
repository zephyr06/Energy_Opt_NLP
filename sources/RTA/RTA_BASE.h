#pragma once

#include "sources/MatrixConvenient.h"
#include "sources/TaskModel/Tasks.h"
#include "sources/Tools/profilier.h"

// @brief All customized TaskSetType must inherit from TaskSetNormal in Tasks.h
namespace rt_num_opt {
size_t rtaCallingTimes;
size_t rtaControl;
inline void ResetCallingTimes() {
    rtaCallingTimes = 0;
    rtaControl = 0;
}
inline void IncrementRTAControl() { rtaControl++; }
inline void IncrementCallingTimes() { rtaCallingTimes++; }
inline size_t ReadCallingTimes() { return rtaCallingTimes; }
inline size_t ReadRTAControl() { return rtaControl; }

/**
 * @brief RTA_BASE encapsulate all the interafaces required
 *  by Energy_optimization.
 * All kinds of RTA should inherit
 * from RTA_BASE and implement its virtual function
 *
 */
template <class TaskSetType>
class RTA_BASE {
   public:
    TaskSetType tasks;
    RTA_BASE() {}
    RTA_BASE(const TaskSetType &tasks) : tasks(tasks) {}

    /**
     * @brief
     *
     * @param tasks are inverse priority ordered
     * @param index
     * @return double
     */
    virtual double RTA_Common_Warm(double beginTime, int index) {
        CoutError("Calling RTA_Common_Warm that is supposed to be overwriten!");
        return 0;
    }

    VectorDynamic ResponseTimeOfTaskSet(const VectorDynamic &warmStart) {
        IncrementCallingTimes();
        int N = tasks.tasks_.size();
        VectorDynamic res = GenerateVectorDynamic(N);
        if (printRTA) {
            std::cout << "Response time analysis of the task set is:"
                      << std::endl;
        }
        for (int i = 0; i < N; i++) {
            res(i, 0) = RTA_Common_Warm(warmStart(i, 0), i);
            if (printRTA) {
                std::cout << "Task " << i << ": " << res(i, 0) << std::endl;
            }
            if (res(i, 0) >= INT32_MAX - 10000) {
                int a = 1;
                a *= a;
            }
        }
        return res;
    }

    VectorDynamic ResponseTimeOfTaskSet() {
        BeginTimer("ResponseTimeOfTaskSet");

        VectorDynamic warmStart =
            GetParameterVD<double>(tasks, "executionTime");
        auto res = ResponseTimeOfTaskSet(warmStart);
        EndTimer("ResponseTimeOfTaskSet");
        return res;
    }

    /**
     * @brief
     *
     * @tparam Schedul_Analysis
     * @param tasks
     * @param warmStart
     * @param whetherPrint
     * @param tol: positive value, makes schedulability check more strict
     * @return true: system is schedulable
     * @return false: system is not schedulable
     */
    bool CheckSchedulability(VectorDynamic warmStart, bool whetherPrint = false,
                             double tol = 0) {
        IncrementCallingTimes();
        int N = tasks.tasks_.size();
        for (int i = 0; i < N; i++) {
            double rta = RTA_Common_Warm(warmStart(i, 0), i);
            if (whetherPrint)
                std::cout << "response time for task " << i << " is " << rta
                          << " and deadline is "
                          << min(tasks.tasks_[i].deadline,
                                 tasks.tasks_[i].period)
                          << std::endl;
            if (rta + tol >
                min(tasks.tasks_[i].deadline, tasks.tasks_[i].period)) {
                if (whetherPrint) {
                    std::cout << "The current task set is not schedulable "
                                 "because of task "
                              << i << " "
                              << "!\n";
                }

                return false;
            }
        }
        if (whetherPrint) std::cout << std::endl;
        return true;
    }

    bool CheckSchedulability(bool whetherPrint = false) {
        VectorDynamic warmStart =
            GetParameterVD<double>(tasks.tasks_, "executionTime");
        return CheckSchedulability(warmStart, whetherPrint);
    }

    bool CheckSchedulabilityDirect(const VectorDynamic &rta) {
        int N = tasks.tasks_.size();
        for (int i = 0; i < N; i++) {
            if (rta(i, 0) >
                min(tasks.tasks_[i].deadline, tasks.tasks_[i].period)) {
                return false;
            }
        }
        return true;
    }
};

VectorDynamic RTAVector(const TaskSetNormal &tasks) {
    BeginTimer(__func__);
    CoutError("You need to re-write this function in corresponding RTA file!");
    EndTimer(__func__);
    return GenerateVectorDynamic1D(0);
}

}  // namespace rt_num_opt