

#include "sources/BatchTestutils.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/Utils/Parameters.h"

using namespace rt_num_opt;
double TIME_LIMIT_FIND_INITIAL = 100;
bool ifTimeout(ProfilerData::TimerType startTime, bool ifPrint = false) {
    auto curr_time = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(curr_time - startTime)
            .count() >= TIME_LIMIT_FIND_INITIAL) {
        if (ifPrint)
            std::cout
                << "\nTime out when running OptimizeOrder. Maximum time is "
                << TIME_LIMIT_FIND_INITIAL << " seconds.\n\n";
        return true;
    }
    return false;
}

bool IterateSpaceHelper(DAG_Nasri19 &tasksN, int taskIndex,
                        ProfilerData::TimerType startTime) {
    if (ifTimeout(startTime)) return false;
    if (taskIndex >= tasksN.N) {
        RTA_Nasri19 r(tasksN);
        // For debug purpose
        // std::cout << GetParameterVD<double>(tasksN,
        // "executionTime").transpose()
        //           << std::endl;
        return r.CheckSchedulability();
    } else {
        for (int i = tasksN.tasks_[taskIndex].executionTimeOrg;
             i <= tasksN.tasks_[taskIndex].deadline; i++) {
            tasksN.tasks_[taskIndex].executionTime = i;
            if (Utilization(tasksN.tasks_) > core_m_dag) {
                tasksN.tasks_[taskIndex].executionTime =
                    tasksN.tasks_[taskIndex].executionTimeOrg;
                return false;  // no need to keep increasing the
                //     executionTime
            }

            bool schedulableTry =
                IterateSpaceHelper(tasksN, taskIndex + 1, startTime);
            if (schedulableTry) return true;
        }
    }
    return false;
}

bool WhetherTaskSetSchedulableInAllSolutionSpace(DAG_Nasri19 tasksN) {
    auto start_time = std::chrono::high_resolution_clock::now();
    return IterateSpaceHelper(tasksN, 0, start_time);
}

bool WhetherTaskSetSchedulableSimple(DAG_Nasri19 tasksN) {
    for (int i = 0; i < tasksN.N; i++) {
        for (int executionTime = tasksN.tasks_[i].executionTimeOrg;
             executionTime <= tasksN.tasks_[i].deadline; executionTime++) {
            tasksN.tasks_[i].executionTime = executionTime;
            RTA_Nasri19 rCurr(tasksN);
            if (rCurr.CheckSchedulability()) {
                return true;
            }
        }
        tasksN.tasks_[i].executionTime = tasksN.tasks_[i].executionTimeOrg;
    }
    return false;
}