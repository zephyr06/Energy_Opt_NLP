#include <CppUnitLite/TestHarness.h>

#include "sources/BatchTestutils.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/Utils/Parameters.h"
using namespace rt_num_opt;

bool SchedulabilityInSolutionSpace(DAG_Nasri19& tasksN) {
    TaskSet& tasks = tasksN.tasks_;
    for (int c0 = tasks[0].executionTimeOrg;
         c0 <= tasks[0].executionTimeOrg * 2 && c0 < tasks[0].deadline; c0++) {
        tasks[0].executionTime = c0;
        for (int c1 = tasks[1].executionTimeOrg;
             c1 <= tasks[1].executionTimeOrg * 2 && c1 < tasks[1].deadline;
             c1++) {
            tasks[1].executionTime = c1;
            for (int c2 = tasks[2].executionTimeOrg;
                 c2 <= tasks[2].executionTimeOrg * 2 && c2 < tasks[2].deadline;
                 c2++) {
                tasks[2].executionTime = c2;
                for (int c3 = tasks[3].executionTimeOrg;
                     c3 <= tasks[3].executionTimeOrg * 2 &&
                     c3 < tasks[3].deadline;
                     c3++) {
                    tasks[3].executionTime = c3;
                    for (int c4 = tasks[4].executionTimeOrg;
                         c4 <= tasks[4].executionTimeOrg * 2 &&
                         c4 < tasks[4].deadline;
                         c4++) {
                        tasks[4].executionTime = c4;
                        for (int c5 = tasks[5].executionTimeOrg;
                             c5 <= tasks[5].executionTimeOrg * 2 &&
                             c5 < tasks[5].deadline;
                             c5++) {
                            tasks[5].executionTime = c5;

                            RTA_Nasri19 rCurr(tasksN);
                            if (rCurr.CheckSchedulability()) {
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }
    return false;
}

TEST(batchfind, v1) {
    const char* pathDataset;
    pathDataset =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/"
        "task_number/";

    std::vector<double> energySaveRatioVec;
    std::vector<double> runTime;
    if (rt_num_opt::debugMode == 1) printf("Directory: %s\n", pathDataset);
    std::vector<std::string> errorFiles;
    std::vector<std::string> pathTaskSet;
    int totalFiles = 0;
    int infeasibleInitial = 0;
    int passOneTime = 0;
    for (const auto& file : rt_num_opt::ReadFilesInDirectory(pathDataset)) {
        if (rt_num_opt::debugMode) std::cout << file << std::endl;

        if (file.find("yaml") != std::string::npos) {
            std::cout << file << "\n";
            std::string path = pathDataset + file;
            pathTaskSet.push_back(path);
            totalFiles++;
            DAG_Nasri19 tasksN = ReadDAGNasri19_Tasks(path);
            RTA_Nasri19 r(tasksN);
            if (r.CheckSchedulability()) {
                passOneTime++;
                continue;
            } else {
                bool foundInfeasible2feasibleCase = false;
                if (SchedulabilityInSolutionSpace(tasksN)) {
                    infeasibleInitial++;
                }
            }
        }
    }

    std::cout << "Total number of files iterated: " << totalFiles << "\n";
    std::cout << "Total number of files that pass the first time: "
              << passOneTime << "\n";
    std::cout << "The number of cases that the given initial solution is "
                 "infeasible but the DAG is actually feasible: "
              << infeasibleInitial << "\n";
    std::cout << "Initial failed ratio: "
              << 100.0 * infeasibleInitial / (passOneTime + infeasibleInitial)
              << "%\n";
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
