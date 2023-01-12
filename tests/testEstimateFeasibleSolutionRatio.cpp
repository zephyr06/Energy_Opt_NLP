#include <CppUnitLite/TestHarness.h>

#include "sources/BatchTestutils.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/Utils/Parameters.h"
using namespace rt_num_opt;
TEST(batchfind, v1) {
    const char *pathDataset;
    pathDataset =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/"
        "test_Dag_feasible_ratio/";

    std::vector<double> energySaveRatioVec;
    std::vector<double> runTime;
    if (rt_num_opt::debugMode == 1) printf("Directory: %s\n", pathDataset);
    std::vector<std::string> errorFiles;
    std::vector<std::string> pathTaskSet;
    int totalFiles = 0;
    int infeasibleInitial = 0;
    for (const auto &file : rt_num_opt::ReadFilesInDirectory(pathDataset)) {
        if (rt_num_opt::debugMode) std::cout << file << std::endl;

        if (file.find("yaml") != std::string::npos) {
            std::string path = pathDataset + file;
            pathTaskSet.push_back(path);
            totalFiles++;
            DAG_Nasri19 tasksN = ReadDAGNasri19_Tasks(path);
            RTA_Nasri19 r(tasksN);
            if (r.CheckSchedulability()) {
                continue;
            } else {
                bool foundInfeasible2feasibleCase = false;
                for (int i = 0; i < tasksN.N && (!foundInfeasible2feasibleCase);
                     i++) {
                    for (int executionTime = tasksN.tasks_[i].executionTimeOrg;
                         executionTime < tasksN.tasks_[i].deadline &&
                         (!foundInfeasible2feasibleCase);
                         executionTime++) {
                        tasksN.tasks_[i].executionTime = executionTime;
                        RTA_Nasri19 rCurr(tasksN);
                        if (rCurr.CheckSchedulability()) {
                            infeasibleInitial++;
                            foundInfeasible2feasibleCase = true;
                        }
                    }
                    tasksN.tasks_[i].executionTime =
                        tasksN.tasks_[i].executionTimeOrg;
                }
            }
        }
    }

    std::cout << "Total number of files iterated: " << totalFiles << "\n";
    std::cout << "The number of cases that the given initial solution is "
                 "infeasible but the DAG is actually feasible: "
              << infeasibleInitial << "\n";
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
