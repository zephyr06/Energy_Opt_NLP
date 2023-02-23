#include <CppUnitLite/TestHarness.h>

#include "sources/BatchTestutils.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/Utils/FeasibleSolutionEstimate.h"
#include "sources/Utils/Parameters.h"

TEST(batchfind, v1) {
    const char *pathDataset;
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
    int initialFeasible = 0;
    for (const auto &file : rt_num_opt::ReadFilesInDirectory(pathDataset)) {
        if (rt_num_opt::debugMode) std::cout << file << std::endl;

        if (file.find("yaml") != std::string::npos) {
            std::string path = pathDataset + file;
            pathTaskSet.push_back(path);
            totalFiles++;
            DAG_Nasri19 tasksN = ReadDAGNasri19_Tasks(path);
            RTA_Nasri19 r(tasksN);
            if (r.CheckSchedulability()) {
                initialFeasible++;
                continue;
            } else {
                auto start_time = std::chrono::high_resolution_clock::now();
                if (WhetherTaskSetSchedulableSimple(tasksN) ||
                    WhetherTaskSetSchedulableInAllSolutionSpace(tasksN)) {
                    std::cout << "Find a case that the initial strategy fails;"
                              << std::endl;
                    infeasibleInitial++;
                } else {
                    if (ifTimeout(start_time)) totalFiles--;
                }
            }
        }
    }

    std::cout << "Total number of files iterated: " << totalFiles << "\n";
    std::cout << "The number of cases that the given initial solution is "
                 "feasible: "
              << initialFeasible << "\n";
    std::cout << "The number of cases that the given initial solution is "
                 "infeasible but the DAG is actually feasible: "
              << infeasibleInitial << "\n";
    std::string pathRes =
        "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/"
        "FeasibleInitialRatio/FeasibleRatio.txt";
    AddEntry(pathRes, 1.0 - float(infeasibleInitial) / totalFiles);
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
