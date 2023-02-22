
#pragma once
#include <boost/optional.hpp>

#include "sources/BatchTestutils.h"
// #include "sources/EnergyOptimization/FactorGraphEnergyLL.h"
#include "sources/EnergyOptimization/EnergyIftopSpec.h"
#include "sources/EnergyOptimization/EnergyOptimize.h"

namespace rt_num_opt {

void BatchCompare(int N = -1) {
    const char *pathDataset =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number";
    std::vector<double> energySaveRatioVec;
    std::vector<double> runTimeW, runTimeZ;

    std::vector<std::string> errorFiles;
    runMode = "compare";
    std::string worstFile = "";
    double worstValue = 0.0;
    for (const auto &file : ReadFilesInDirectory(pathDataset)) {
        // if (debugMode)
        std::cout << file << std::endl;
        std::string delimiter = "-";

        if (file.substr(0, file.find(delimiter)) == "periodic") {
            std::string path =
                "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/"
                "task_number/" +
                file;
            auto taskSet1 = ReadTaskSet(path, readTaskMode);
            TaskSetNormal tasksN(taskSet1);
            auto start = std::chrono::high_resolution_clock::now();
            double res;
            ResetCallingTimes();
            // next project: consider generalized elimination?
            if (optimizerType == 6)  // IPM-Ifopt
            {
                res = OptimizeEnergyIfopt<TaskSetNormal, RTA_LL>(tasksN);
            } else if (optimizerType <= 4)
                res =
                    Energy_Opt<TaskSetNormal, RTA_LL>::OptimizeTaskSet(tasksN);
            else
                CoutError("Unrecognized optimizer type!");

            size_t rtaCall = ReadCallingTimes();
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::microseconds>(stop -
                                                                      start);
            double timeTaken = double(duration.count()) / 1e6;

            auto baselineResult = ReadBaselineResult(path, taskSet1.size());

            if (res >= 0 && res <= 1) {
                energySaveRatioVec.push_back(res / (baselineResult.second));
                if (energySaveRatioVec.back() > worstValue) {
                    worstValue = energySaveRatioVec.back();
                    worstFile = path;
                }
                runTimeW.push_back(timeTaken);
                runTimeZ.push_back(baselineResult.first);

                if (debugMode == 3)
                    std::cout
                        << "One compare: " << res / (baselineResult.second)
                        << std::endl;
                // std::ofstream outfileWrite;
                // outfileWrite.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/"
                // + batchOptimizeFolder + "/EnergySaveRatio/N" +
                //                       std::to_string(taskSet1.size()) +
                //                       ".txt",
                //                   std::ios_base::app);
                // outfileWrite << energySaveRatioVec.back() << std::endl;
                // outfileWrite.close();
                AddEntry(
                    "/home/zephyr/Programming/Energy_Opt_NLP/"
                    "CompareWithBaseline/" +
                        batchOptimizeFolder + "/EnergySaveRatio/N" +
                        std::to_string(taskSet1.size()) + ".txt",
                    energySaveRatioVec.back());
                AddEntry(
                    "/home/zephyr/Programming/Energy_Opt_NLP/"
                    "CompareWithBaseline/" +
                        batchOptimizeFolder + "/RTACalling/N" +
                        std::to_string(taskSet1.size()) + ".txt",
                    rtaCall);

                N = taskSet1.size();
            } else if (res == -1 || res > 1) {
                errorFiles.push_back(file);
            }
        }
    }

    double avEnergy = -1;
    double aveTime = -1;
    int n = energySaveRatioVec.size();
    if (n != 0) {
        avEnergy = Average(energySaveRatioVec);
        aveTime = Average(runTimeW) / Average(runTimeZ);
    }

    std::cout << Color::blue << std::endl;
    std::cout << "Average energy optimization objective (NLP: SA) ratio is "
              << avEnergy << std::endl;
    std::cout << "The worst value is " << worstValue << std::endl;
    std::cout << "The worst file is " << worstFile << std::endl;
    std::cout << "Average time consumed ratio (NLP: SA) is " << aveTime
              << std::endl;
    std::cout << "The number of tasksets under analyzation is "
              << energySaveRatioVec.size() << std::endl;
    std::cout << Color::def << std::endl;

    std::ofstream outfile2;
    outfile2.open(
        "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
            batchOptimizeFolder + "/time_task_number.txt",
        std::ios_base::app);
    outfile2 << Average(runTimeW) << ", " << Average(runTimeZ) << std::endl;
    if (debugMode == 1) std::cout << std::endl;
    for (auto &file : errorFiles) std::cout << file << std::endl;
    // if (debugMode)
    std::cout << "The total number of optimization failure files is "
              << errorFiles.size() << std::endl;

    std::ofstream outfileWrite;
    outfileWrite.open(
        "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
            batchOptimizeFolder + "/Time/N" + std::to_string(N) + ".txt",
        std::ios_base::app);
    outfileWrite << Average(runTimeW) << std::endl;
    outfileWrite.close();

    return;
}
}  // namespace rt_num_opt