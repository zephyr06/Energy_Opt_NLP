#pragma once
#include "sources/BatchTestutils.h"
#include "sources/EnergyOptimization/Optimize.h"
#include "sources/RTA/RTA_Melani.h"
#include "sources/RTA/RTA_Narsi19.h"

namespace rt_num_opt
{
    void AddEntry(std::string pathRes, double val)
    {
        std::ofstream outfileWrite;
        outfileWrite.open(pathRes,
                          std::ios_base::app);
        outfileWrite << val << std::endl;
        outfileWrite.close();
    }

    inline std::string GetResFileName(const std::string &pathDataset, const std::string &file)
    {
        return pathDataset + file + "_Res.txt";
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
                    }
                    // else if (TaskSetType::Type() == "dag")
                    // {
                    //     tasksN = ReadDAG_Tasks(path, readTaskMode);
                    //     N = tasksN.tasks_.size();
                    // }
                    else if (TaskSetType::Type() == "Narsi19")
                    {
                        tasksN = ReadDAGNarsi19_Tasks(path);
                        N = Nn;
                    }
                    else
                    {
                        CoutError("Unrecognized TaskSetType!");
                    }

                    auto start = std::chrono::high_resolution_clock::now();
                    double res = Energy_Opt<TaskSetType, Schedul_Analysis>::OptimizeTaskSet(tasksN);
                    // std::cout << "The energy saving ratio is " << res << std::endl;
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                    double timeTaken = double(duration.count()) / 1e6;
                }

                WriteToResultFile(pathDataset, file, res, timeTaken);
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
        int n = runTime.size();
        if (n != 0)
        {
            avEnergy = Average(energySaveRatioVec);
            aveTime = Average(runTime);
        }
        std::cout << Color::blue << std::endl;
        std::cout << "Average energy saving ratio is " << avEnergy << std::endl;
        std::cout << "Average time consumed is " << aveTime << std::endl;
        std::cout << "The number of tasksets under analyzation is " << energySaveRatioVec.size() << std::endl;

        std::string pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                              batchOptimizeFolder + "/EnergySaveRatio/N" +
                              std::to_string(N) + ".txt";
        AddEntry(pathRes, avEnergy);
        // old saving path:
        //  pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
        //       batchOptimizeFolder + "/time_task_number.txt";
        pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                  batchOptimizeFolder + "/Time/N" +
                  std::to_string(N) + ".txt";
        AddEntry(pathRes, aveTime);

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