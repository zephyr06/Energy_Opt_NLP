#pragma once
#include "sources/BatchControlOptimize.h"
#include "sources/BatchOptimizeIO.h"
#include "sources/BatchTestutils.h"
#include "sources/ControlOptimization/ControlIfoptSpec.h"
#include "sources/ControlOptimization/ControlOptimize.h"
#include "sources/ControlOptimization/ControlOptimizeNasri19.h"

namespace ControlNasri19 {
/**
 * @brief check whether the file in the directory is what we're looking for;
 * this function assumes the possible input are only one of the following:
 * periodic-dag-Narsi-set-<n>-<m>-syntheticJobs.yaml, *_control_north.txt,
 * *_control_north_plus.txt, all the other files are not processed;
 *
 * @param file
 * @return int
 *  0 means target source file;
 *  1 means baseline result, NORTH
 *  2 means baseline result, NORTH+
 *  3 means unrelated;
 */
int TargetFileType(std::string file) {
    if (file.find(".yaml") == file.length() - 5 &&
        file.find(".yaml") != std::string::npos)
        return 0;
    else if (file.find("_control_north.txt") != std::string::npos)
        return 1;
    else if (file.find("_control_north_plus.txt") != std::string::npos)
        return 2;
    else
        return 3;
}

inline std::string GetResFileName(const std::string &pathDataset,
                                  const std::string &file) {
    if (rt_num_opt::enableReorder == 0) {
        return pathDataset + file + "_control_north.txt";
    } else {  // (rt_num_opt::enableReorder == 1)
        return pathDataset + file + "_control_north_plus.txt";
    }
}
void WriteToResultFile(const std::string &pathDataset, const std::string &file,
                       double res, double timeTaken) {
    std::string resFile = GetResFileName(pathDataset, file);
    rt_num_opt::WriteToResultFile(resFile, res, timeTaken);
}
std::pair<double, double> ReadFromResultFile(const std::string &pathDataset,
                                             const std::string &file) {
    std::string resFile = GetResFileName(pathDataset, file);
    return rt_num_opt::ReadFromResultFile(resFile);
}

bool VerifyResFileExist(const std::string &pathDataset,
                        const std::string &file) {
    std::string resFile = GetResFileName(pathDataset, file);
    return rt_num_opt::VerifyResFileExist(resFile);
}

}  // namespace ControlNasri19

namespace rt_num_opt {
using namespace ControlOptimize;

// only for Nasri19 experiment
void BatchOptimizeNasri19(int Nn = 5) {
    const char *pathDataset;
    std::string str = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/N" +
                      std::to_string(Nn) + "/";
    pathDataset = str.c_str();
    if (debugMode == 1)
        printf("Directory: %s\n", pathDataset);

    std::vector<double> objVec;
    std::vector<double> runTime;
    std::vector<size_t> rtaCallTime;
    std::vector<std::string> failedFiles;
    int N;
    if (debugMode == 1)
        printf("Directory: %s\n", pathDataset);
    std::vector<std::string> errorFiles;
    double worstObjRatio = -100;
    std::string worstFile = "";
    for (const auto &file : ReadFilesInDirectory(pathDataset)) {
        int type = ControlNasri19::TargetFileType(file);
        std::string path = pathDataset + file;
        switch (type) {
            case 0:  // perform optimization
            {
                std::cout << file << std::endl;
                DAG_Nasri19 dag_tasks = ReadDAGNasri19_Tasks(path);
                VectorDynamic coeff = ReadControlCoeff(path);
                std::vector<bool> maskForElimination(dag_tasks.SizeNode(),
                                                     false);
                double obj_res;
                double timeTaken;
                if (VerifyResFileExist(pathDataset, file)) {
                    std::pair<double, double> res_from_read =
                        ControlNasri19::ReadFromResultFile(pathDataset, file);
                    timeTaken = res_from_read.second;
                    obj_res = res_from_read.first;
                } else {
                    auto start = std::chrono::high_resolution_clock::now();
                    std::pair<VectorDynamic, double> res =
                        OptimizeTaskSetIterative<
                            FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>,
                            DAG_Nasri19, RTA_Nasri19>(dag_tasks, coeff,
                                                      maskForElimination);
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration =
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            stop - start);
                    timeTaken = double(duration.count()) / 1e6;
                    obj_res = res.second;
                }

                runTime.push_back(timeTaken);
                objVec.push_back(obj_res);
                std::string pathRes =
                    "/home/zephyr/Programming/Energy_Opt_NLP/"
                    "CompareWithBaseline/" +
                    batchOptimizeFolder + "/EnergySaveRatio/N" +
                    std::to_string(N) + ".txt";
                AddEntry(pathRes, obj_res);
                ControlNasri19::WriteToResultFile(pathDataset, file, obj_res,
                                                  timeTaken);
                break;
            }
            default: {
                continue;
            }
        }
    }
    std::string pathRes =
        "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
        batchOptimizeFolder + "/time_task_number.txt";
    AddEntry(pathRes, Average(runTime));

    std::cout << Color::blue << std::endl;
    std::cout << "Average relative obj vectors " << Average(objVec)
              << std::endl;

    std::cout << "Average time consumed is " << Average(runTime) << std::endl;
    std::cout << Color::def << std::endl;

    // used for ControlPerformance's speed figure
    std::ofstream outfileWrite;
    outfileWrite.open(
        "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
            batchOptimizeFolder + "/Time/N" + std::to_string(Nn) + ".txt",
        std::ios_base::app);
    outfileWrite << Average(runTime) << std::endl;
    outfileWrite.close();
}
}  // namespace rt_num_opt