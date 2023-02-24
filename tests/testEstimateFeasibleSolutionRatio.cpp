
#include "sources/BatchTestutils.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/Utils/FeasibleSolutionEstimate.h"
#include "sources/Utils/Parameters.h"
#include "sources/argparse.hpp"

enum FEASIBLE_STATUS {
    Initial_feasible,
    Initial_infeasible,
    Time_out,
    Always_infeasible
};

inline std::string GetResFileName(const std::string &pathDataset,
                                  const std::string &file) {
    return pathDataset + file + "_Res.txt";
}
void WriteToResultFile(const std::string &pathDataset, const std::string &file,
                       int res, double timeTaken) {
    std::string resFile = GetResFileName(pathDataset, file);
    std::ofstream outfileWrite;
    outfileWrite.open(resFile, std::ios_base::app);
    outfileWrite << res << std::endl;
    outfileWrite << timeTaken << std::endl;
    outfileWrite.close();
}
std::pair<int, double> ReadFromResultFile(const std::string &pathDataset,
                                          const std::string &file) {
    std::string resFile = GetResFileName(pathDataset, file);
    std::ifstream cResultFile(resFile.data());
    double timeTaken = 0;
    int res = 0;
    cResultFile >> res >> timeTaken;
    cResultFile.close();
    return std::make_pair(res, timeTaken);
}

bool VerifyResFileExist(const std::string &pathDataset,
                        const std::string &file) {
    std::string resFile = GetResFileName(pathDataset, file);
    std::ifstream myfile;
    myfile.open(resFile);
    if (myfile) {
        return true;
    } else {
        return false;
    }
}

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

        if (file.find("yaml") != std::string::npos &&
            file.find("Res") == std::string::npos) {
            totalFiles++;
            if (VerifyResFileExist(pathDataset, file)) {
                auto res = ReadFromResultFile(pathDataset, file);
                if (res.second >= TIME_LIMIT_FIND_INITIAL - 10) continue;
                switch (res.first) {
                    case FEASIBLE_STATUS::Initial_feasible:
                        initialFeasible++;
                        break;
                    case FEASIBLE_STATUS::Initial_infeasible:
                        infeasibleInitial++;
                        break;
                    case FEASIBLE_STATUS::Time_out:
                        totalFiles--;
                        break;
                    case FEASIBLE_STATUS::Always_infeasible:
                        totalFiles--;
                        break;
                    default:
                        CoutError("Wrong Feasiebl_Status!");  // do nothing
                }
                continue;
            }

            std::string path = pathDataset + file;
            pathTaskSet.push_back(path);
            DAG_Nasri19 tasksN = ReadDAGNasri19_Tasks(path);
            RTA_Nasri19 r(tasksN);
            if (r.CheckSchedulability()) {
                initialFeasible++;
                WriteToResultFile(pathDataset, file,
                                  FEASIBLE_STATUS::Initial_feasible, 0);
                continue;
            } else {
                auto start_time = std::chrono::high_resolution_clock::now();
                if (WhetherTaskSetSchedulableSimple(tasksN) ||
                    WhetherTaskSetSchedulableInAllSolutionSpace(tasksN)) {
                    std::cout << "Find a case that the initial "
                                 "strategy fails;"
                              << std::endl;
                    infeasibleInitial++;
                    WriteToResultFile(pathDataset, file,
                                      FEASIBLE_STATUS::Initial_infeasible, 0);
                } else {  // cannot be analyzed to be schedulable
                    if (ifTimeout(start_time, true)) {
                        WriteToResultFile(pathDataset, file,
                                          FEASIBLE_STATUS::Time_out, 0);
                    } else {  // not schedulable
                        WriteToResultFile(pathDataset, file,
                                          FEASIBLE_STATUS::Always_infeasible,
                                          0);
                    }

                    totalFiles--;
                }
            }
        }
    }

    std::cout << "\nTotal number of files iterated: " << totalFiles << "\n";
    std::cout << "The number of cases that the given initial solution is "
                 "feasible: "
              << initialFeasible << "\n";
    std::cout << "The number of cases that the given initial solution is "
                 "infeasible but the DAG is actually feasible: "
              << infeasibleInitial << "\n";
    std::string pathRes =
        "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/"
        "FeasibleInitialRatio/FeasibleRatio.txt";
    AddEntry(pathRes, 1.0 - float(infeasibleInitial) /
                                (initialFeasible + infeasibleInitial));
}