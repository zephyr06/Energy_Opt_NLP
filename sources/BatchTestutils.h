#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>

#include "../sources/Optimize.h"
using namespace std::chrono;

double Average(vector<double> &data)
{
    if (data.size())
    {
        double sum = 0;
        for (int i = 0; i < int(data.size()); i++)
            sum += data[i];
        return sum / data.size();
    }
    else
    {
        return -1;
    }
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