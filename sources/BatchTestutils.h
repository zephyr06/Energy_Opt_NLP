#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>

#include "sources/EnergyOptimization/Optimize.h"
// using namespace std::chrono;
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

    template <typename T>
    double Average(std::vector<T> &data)
    {
        if (data.size())
        {
            T sum = 0;
            for (int i = 0; i < int(data.size()); i++)
                sum += data[i];
            return double(sum) / data.size();
        }
        else
        {
            return -1;
        }
    }

    std::vector<std::string> ReadFilesInDirectory(const char *path)
    {
        std::vector<std::string> files;
        DIR *dr;
        struct dirent *en;
        dr = opendir(path);
        if (dr)
        {
            while ((en = readdir(dr)) != NULL)
            {
                files.push_back(en->d_name); // print all directory name
            }
            closedir(dr); // close all directory
        }
        sort(files.begin(), files.end());
        return files;
    }

    /**
     * @brief
     *
     * @param path periodic-set-1-syntheticJobs.csv
     * @return std::string "1"
     */
    int ExtractIndex(std::string path, std::string delimiter = "-")
    {
        int pos = 0;

        while (!(path[0] >= '0' && path[0] <= '9') && path.size() > 0)
        {
            pos = path.find(delimiter);
            path.erase(0, pos + delimiter.length());
        }
        pos = path.find(delimiter);
        std::string token = path.substr(0, pos);
        int temp = atoi(token.c_str());
        return temp;
    }
    /**
     * @brief read Yecheng's result, given path in my format;
     *
     * @param path like, periodic-set-1-syntheticJobs.csv
     * @return pair<double, double> objective, time
     */
    std::pair<double, double> ReadBaselineResult(std::string &pathInPeriodicDataset, int N)
    {
        std::string yechengRepoPath = "/home/zephyr/Programming/others/YechengRepo/Experiment/WCETEnergyOpt/TestCases/NSweep/N" + std::to_string(N) + "/";

        int index = ExtractIndex(pathInPeriodicDataset);
        std::string targetFilePathGP = yechengRepoPath + "Case" + std::to_string(index) + ".txt" + "_RM_GPResult.txt";
        std::string targetFilePathBF = yechengRepoPath + "Case" + std::to_string(index) + ".txt" + "_RM_BFSResult.txt";
        if (debugMode == 1)
            std::cout << "targetFilePathBF " << targetFilePathBF << std::endl;
        std::string fileName;
        if (baselineLLCompare == 1)
            fileName = targetFilePathBF;
        else if (baselineLLCompare == 2)
            fileName = targetFilePathGP;
        else
        {
            CoutError("Unrecognized baselineLLCompare! Current value is " + std::to_string(baselineLLCompare));
        }

        std::ifstream cResultFile(fileName.data());
        try
        {
            assert(cResultFile.is_open());
        }
        catch (...)
        {
            std::cout << "Error in reading "
                      << batchOptimizeFolder
                      << "'s result files" << fileName << std::endl;
        }

        double runTime = 0, obj = 0;
        cResultFile >> runTime >> obj;
        double nd = 0;
        cResultFile >> nd;
        int n = round(nd);
        std::vector<int> values(n, 0);
        for (int i = 0; i < n; i++)
        {
            double val = 0;
            cResultFile >> val;
            values[i] = abs(round(val));
        }

        // check schedulability
        auto taskSet1 = ReadTaskSet(pathInPeriodicDataset, "orig");
        TaskSet tasksInit = taskSet1;
        UpdateTaskSetExecutionTime(taskSet1, Vector2Eigen(values));
        taskSet1 = Reorder(taskSet1, "deadline");

        RTA_LL r(taskSet1);
        bool schedulale_flag = r.CheckSchedulability(
            debugMode == 1);
        if (not schedulale_flag)
        {
            if (baselineLLCompare == 1)
                CoutError("Found one unschedulable result in Zhao20!");
            obj = EstimateEnergyTaskSet(tasksInit).sum() / weightEnergy;
        }
        if (baselineLLCompare == 1)
            obj = obj / 1e9;

        return {runTime, obj};
    }
} // namespace rt_num_opt