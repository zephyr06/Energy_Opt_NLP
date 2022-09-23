#pragma once
#include "sources/BatchTestutils.h"
#include "sources/EnergyOptimization/Optimize.h"
#include "sources/RTA/RTA_Melani.h"
#include "sources/RTA/RTA_Nasri19.h"

#include "sources/EnergyOptimization/OptimizeSA.h"
#include "sources/EnergyOptimization/EnergyOptimize.h"
#include "sources/EnergyOptimization/EnergyIftopSpec.h"

namespace rt_num_opt
{

    inline std::string GetResFileName(const std::string &pathDataset, const std::string &file)
    {
        if (optimizerType == 5)
        {
            return pathDataset + file + "_SA_Res.txt";
        }
        else if (optimizerType == 6)
        {
            return pathDataset + file + "_IPM_Res.txt";
        }
        else if (elimIte > 0)
        {
            return pathDataset + file + "_elim_approx_Res.txt";
        }
        else
        {
            return pathDataset + file + "_not_elim_Res.txt";
        }
    }
    void WriteToResultFile(const std::string &pathDataset, const std::string &file, double res, double timeTaken)
    {
        std::string resFile = GetResFileName(pathDataset, file);
        std::ofstream outfileWrite;
        outfileWrite.open(resFile,
                          std::ios_base::app);
        outfileWrite << res << std::endl;
        outfileWrite << timeTaken << std::endl;
        outfileWrite.close();
    }
    std::pair<double, double> ReadFromResultFile(const std::string &pathDataset, const std::string &file)
    {
        std::string resFile = GetResFileName(pathDataset, file);
        std::ifstream cResultFile(resFile.data());
        double timeTaken = 0, res = 0;
        cResultFile >> res >> timeTaken;
        cResultFile.close();
        return std::make_pair(res, timeTaken);
    }

    bool VerifyResFileExist(const std::string &pathDataset, const std::string &file)
    {
        std::string resFile = GetResFileName(pathDataset, file);
        std::ifstream myfile;
        myfile.open(resFile);
        if (myfile)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    template <class TaskSetType, class Schedul_Analysis>
    void BatchOptimize(int Nn = -1)
    {
        runMode = "normal";
        const char *pathDataset;
        std::string str = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/N" + std::to_string(Nn) + "/";
        if (Nn == -1)
            pathDataset = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/";
        else
        {
            pathDataset = str.c_str();
            if (debugMode == 1)
                printf("Directory: %s\n", pathDataset);
        }
        std::vector<double> energySaveRatioVec;
        std::vector<double> runTime;
        std::vector<size_t> rtaCallTime;
        int N;
        if (debugMode == 1)
            printf("Directory: %s\n", pathDataset);
        std::vector<std::string> errorFiles;
        for (const auto &file : ReadFilesInDirectory(pathDataset))
        {
            // if (debugMode)
            std::string delimiter = "-";
            if (file.substr(0, file.find(delimiter)) == "periodic" && file.substr(file.length() - 4, 4) != ".txt")
            {
                std::cout << file << std::endl;
                double res;
                double timeTaken;
                if (VerifyResFileExist(pathDataset, file)) // already optimized
                {
                    auto p = ReadFromResultFile(pathDataset, file);
                    res = p.first;
                    timeTaken = p.second;
                }
                else
                {
                    std::string path = pathDataset + file;
                    TaskSetType tasksN;
                    if (TaskSetType::Type() == "normal")
                    {
                        auto tasks = ReadTaskSet(path, readTaskMode);
                        tasksN.UpdateTaskSet(tasks);
                        N = tasks.size();
                        Nn = tasks.size();
                    }
                    // else if (TaskSetType::Type() == "dag")
                    // {
                    //     tasksN = ReadDAG_Tasks(path, readTaskMode);
                    //     N = tasksN.tasks_.size();
                    // }
                    else if (TaskSetType::Type() == "Nasri19")
                    {
                        tasksN = ReadDAGNasri19_Tasks(path);
                        N = Nn;
                        if (batchOptimizeFolder == "DAGPerformanceUtil")
                        {
                            N = round(Utilization(tasksN.tasks_) * 10.0 / core_m_dag);
                            std::cout << N << std::endl;
                        }
                    }
                    else
                    {
                        CoutError("Unrecognized TaskSetType!");
                    }

                    auto start = std::chrono::high_resolution_clock::now();
                    ResetCallingTimes();
                    if (optimizerType <= 4)
                    {
                        if (TaskSetType::Type() == "normal")
                            res = Energy_Opt<TaskSetType, Schedul_Analysis>::OptimizeTaskSet(tasksN);
                        else if (TaskSetType::Type() == "Nasri19")
                            res = Energy_OptDAG<TaskSetType, Schedul_Analysis>::OptimizeTaskSetIterative(tasksN).second;
                    }
                    else if (optimizerType == 5) // simulateed annealing
                    {
                        auto resStruct = OptimizeSchedulingSA<TaskSetType, Schedul_Analysis>(tasksN);
                        res = resStruct.optimizeError / resStruct.initialError;
                    }
                    else if (optimizerType == 6) // IPM-Ifopt
                    {
                        res = OptimizeEnergyIfopt<TaskSetType, Schedul_Analysis>(tasksN);
                    }
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                    timeTaken = double(duration.count()) / 1e6;

                    size_t rtaCall = ReadCallingTimes();
                    rtaCallTime.push_back(rtaCall);

                    WriteToResultFile(pathDataset, file, res, timeTaken);
                }

                if (res >= 0 && res <= 1)
                {
                    energySaveRatioVec.push_back(res);
                    runTime.push_back(timeTaken);
                }
                else
                {
                    errorFiles.push_back(file);
                }
            }
        }

        double avEnergy = -1;
        double aveTime = -1;
        double aveRTA = -1;
        int n = runTime.size();
        if (n != 0)
        {
            avEnergy = Average(energySaveRatioVec);
            aveTime = Average(runTime);
            aveRTA = Average(rtaCallTime);
        }
        std::cout << Color::blue << std::endl;
        std::cout << "Average energy saving ratio is " << avEnergy << std::endl;
        std::cout << "Average time consumed is " << aveTime << std::endl;
        std::cout << "Average RTA calling time is " << aveRTA << std::endl;
        std::cout << "The number of tasksets under analyzation is " << energySaveRatioVec.size() << std::endl;

        std::string pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                              batchOptimizeFolder + "/EnergySaveRatio/N" +
                              std::to_string(Nn) + ".txt";
        AddEntry(pathRes, avEnergy);

        pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                  batchOptimizeFolder + "/Time/N" +
                  std::to_string(Nn) + ".txt";
        AddEntry(pathRes, aveTime);

        pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                  batchOptimizeFolder + "/RTACalling/N" +
                  std::to_string(Nn) + ".txt";
        AddEntry(pathRes, aveRTA);

        if (printFailureFile)
        {
            std::cout << std::endl;
            for (auto &file : errorFiles)
                std::cout << file << std::endl;
        }
        // if (debugMode)
        std::cout << "The total number of optimization failure files is " << errorFiles.size() << std::endl;
        std::cout << Color::def << std::endl;
        return;
    }
} // namespace rt_num_opt