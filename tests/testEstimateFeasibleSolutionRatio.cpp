
#include "sources/BatchTestutils.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/Utils/FeasibleSolutionEstimate.h"
#include "sources/Utils/Parameters.h"
#include "sources/argparse.hpp"

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("program name");
    program.add_argument("-v", "--verbose");  // parameter packing

    program.add_argument("--U")
        .default_value(0.0)
        .help("the utilization folder")
        .scan<'f', double>();
    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error &err) {
        std::cout << err.what() << std::endl;
        std::cout << program;
        exit(0);
    }

    int N = program.get<double>("--U") * 10;
    std::string pathDatasetStr =
        "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/"
        "FeasibleInitialRatio/U" +
        std::to_string(N) + "/";
    const char *pathDataset = pathDatasetStr.c_str();

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
                    std::cout << "Find a case that the initial "
                                 "strategy fails;"
                              << std::endl;
                    infeasibleInitial++;
                } else {
                    if (ifTimeout(start_time, true)) totalFiles--;
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
    AddEntry(pathRes,
             float(infeasibleInitial) / (initialFeasible + infeasibleInitial));
}