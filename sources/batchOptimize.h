#include <iostream>
#include <filesystem>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>

#include "Optimize.h"
using namespace std::chrono;

double Average(vector<float> &data)
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
    return files;
}

void BatchOptimize()
{
    const char *pathDataset = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData";
    vector<float> energySaveRatioVec;
    vector<float> runTime;

    for (const auto &file : ReadFilesInDirectory(pathDataset))
    {
        cout << file << endl;
        string delimiter = "-";
        if (file.substr(0, file.find(delimiter)) == "periodic")
        {
            string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + file;
            TaskSet taskSet1 = ReadTaskSet(path, "RM");
            auto start = chrono::high_resolution_clock::now();
            float res = OptimizeTaskSet(taskSet1);
            // cout << "The energy saving ratio is " << res << endl;
            auto stop = chrono::high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            float timeTaken = float(duration.count()) / 1e6;
            if (res != -1)
            {
                energySaveRatioVec.push_back(res);
                runTime.push_back(timeTaken);
            }
        }
    }

    float avEnergy = -1;
    float aveTime = -1;
    int n = runTime.size();
    if (n != 0)
    {
        avEnergy = Average(energySaveRatioVec);
        aveTime = Average(runTime);
    }

    ofstream outfile1, outfile2;
    outfile1.open("/home/zephyr/Programming/Energy_Opt_NLP/Visualization/data_buffer_energy_task_number.txt", std::ios_base::app);
    outfile1 << avEnergy << endl;
    outfile2.open("/home/zephyr/Programming/Energy_Opt_NLP/Visualization/time_task_number.txt", std::ios_base::app);
    outfile2 << aveTime << endl;

    return;
}