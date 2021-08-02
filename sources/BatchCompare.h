#include <iostream>
#pragma once
#include <string>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>

#include "Optimize.h"
using namespace std::chrono;

double Average(vector<double> &data)
{
    double sum = 0;
    for (int i = 0; i < int(data.size()); i++)
        sum += data[i];
    return sum / data.size();
}

vector<string> ReadFilesInDirectory(const char *path)
{
    vector<string> files;
    DIR *dr;
    struct dirent *en;
    dr = opendir(path);
    if (dr)
    {
        while ((en = readdir(dr)) != NULL)
        {
            files.push_back(en->d_name); //print all directory name
        }
        closedir(dr); //close all directory
    }
    sort(files.begin(), files.end());
    return files;
}

/**
 * @brief 
 * 
 * @param path periodic-set-1-syntheticJobs.csv
 * @return string "1"
 */
int ExtractIndex(string path, string delimiter = "-")
{
    int pos = 0;

    while (!(path[0] >= '0' && path[0] <= '9') && path.size() > 0)
    {
        pos = path.find(delimiter);
        path.erase(0, pos + delimiter.length());
    }
    pos = path.find(delimiter);
    string token = path.substr(0, pos);
    int temp = atoi(token.c_str());
    return temp;
}

/**
 * @brief read Yecheng's result, given path in my format;
 * 
 * @param path like, periodic-set-1-syntheticJobs.csv
 * @return pair<double, double> objective, time
 */
pair<double, double> ReadBaselineResult(string &pathInPeriodicDataset, int N)
{
    string yechengRepoPath = "/home/zephyr/Programming/YechengRepo/Experiment/WCETEnergyOpt/TestCases/NSweep/N" + to_string(N) + "/";

    int index = ExtractIndex(pathInPeriodicDataset);
    string targetFilePathGP = yechengRepoPath + "Case" + to_string(index) + ".txt" + "_RM_GPResult.txt";
    string targetFilePathBF = yechengRepoPath + "Case" + to_string(index) + ".txt" + "_RM_BFSResult.txt";
    if (debugMode == 1)
        cout << "targetFilePathBF " << targetFilePathBF << endl;
    string fileName = targetFilePathBF;

    ifstream cResultFile(fileName.data());
    try
    {
        assert(cResultFile.is_open());
    }
    catch (...)
    {
        cout << "Error in " << fileName << endl;
    }

    double runTime = 0, obj = 0;
    cResultFile >> runTime >> obj;
    double nd = 0;
    cResultFile >> nd;
    int n = round(nd);
    vector<int> values(n, 0);
    for (int i = 0; i < n; i++)
    {
        double val = 0;
        cResultFile >> val;
        values[i] = round(val);
    }
    return {runTime, obj};
}

void BatchCompare()
{
    const char *pathDataset = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number";
    vector<double> energySaveRatioVec;
    vector<double> runTimeW, runTimeZ;

    vector<string> errorFiles;
    string worstFile = "";
    double worstValue = 0.0;
    for (const auto &file : ReadFilesInDirectory(pathDataset))
    {
        if (debugMode)
            cout << file << endl;
        string delimiter = "-";

        if (file.substr(0, file.find(delimiter)) == "periodic")
        {
            string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/" + file;
            TaskSet taskSet1 = ReadTaskSet(path, readTaskMode);
            auto start = chrono::high_resolution_clock::now();
            double res = OptimizeTaskSet(taskSet1);
            // cout << "The energy saving ratio is " << res << endl;
            auto stop = chrono::high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            double timeTaken = double(duration.count()) / 1e6;

            auto baselineResult = ReadBaselineResult(path, taskSet1.size());

            if (res >= 0 && res <= 1)
            {
                energySaveRatioVec.push_back(res / (baselineResult.second / 1e9));
                if (energySaveRatioVec.back() > worstValue)
                {
                    worstValue = energySaveRatioVec.back();
                    worstFile = path;
                }
                runTimeW.push_back(timeTaken);
                runTimeZ.push_back(baselineResult.first);

                if (debugMode == 3)
                    cout << "One compare: " << res / (baselineResult.second / 1e9) << endl;
                ofstream outfileWrite;
                outfileWrite.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/ResultFiles/N" + to_string(taskSet1.size()) + ".txt", std::ios_base::app);
                outfileWrite << energySaveRatioVec.back() << endl;
                outfileWrite.close();
            }
            else if (res == -1 || res > 1)
            {
                errorFiles.push_back(file);
            }
        }
    }

    double avEnergy = -1;
    double aveTime = -1;
    int n = energySaveRatioVec.size();
    if (n != 0)
    {
        avEnergy = Average(energySaveRatioVec);
        aveTime = Average(runTimeW) / Average(runTimeZ);
    }

    ofstream outfile1, outfile2;
    outfile1.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/data_buffer_energy_task_number.txt", std::ios_base::app);
    outfile1 << avEnergy << endl;
    // if (debugMode)
    // {
    cout << "Average energy optimization objective (NLP: MUA) ratio is " << avEnergy << endl;
    cout << "The worst value is " << worstValue << endl;
    cout << "The worst file is " << worstFile << endl;
    cout << "Average time consumed ratio (NLP: MUA) is " << aveTime << endl;
    cout << "The number of tasksets under analyzation is " << energySaveRatioVec.size() << endl;
    // }

    outfile2.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/time_task_number.txt", std::ios_base::app);
    outfile2 << Average(runTimeW) << ", " << Average(runTimeZ) << endl;
    if (debugMode == 1)
        cout << endl;
    for (auto &file : errorFiles)
        cout << file << endl;
    // if (debugMode)
    cout << "The total number of optimization failure files is " << errorFiles.size() << endl;

    return;
}