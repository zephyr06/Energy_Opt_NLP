#pragma once
#include "BatchTestutils.h"

void BatchOptimize()
{
    const char *pathDataset = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number";
    vector<double> energySaveRatioVec;
    vector<double> runTime;

    vector<string> errorFiles;
    ofstream outfileWrite;
    outfileWrite.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/data_buffer_energy_task_number.txt", std::ios_base::app);

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
            double res = Energy_Opt<RTA_LL>::OptimizeTaskSet(taskSet1);
            // cout << "The energy saving ratio is " << res << endl;
            auto stop = chrono::high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            double timeTaken = double(duration.count()) / 1e6;
            if (res >= 0 && res <= 1)
            {
                energySaveRatioVec.push_back(res);
                runTime.push_back(timeTaken);
                outfileWrite << energySaveRatioVec.back() << endl;
            }
            else if (res == -1 || res > 1)
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

    ofstream outfile1, outfile2;
    outfile1.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/data_buffer_energy_task_number.txt", std::ios_base::app);
    outfile1 << avEnergy << endl;
    // if (debugMode)
    // {
    cout << "Average energy saving ratio is " << avEnergy << endl;
    cout << "Average time consumed is " << aveTime << endl;
    cout << "The number of tasksets under analyzation is " << energySaveRatioVec.size() << endl;
    // }

    outfile2.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/time_task_number.txt", std::ios_base::app);
    outfile2 << aveTime << endl;
    if (debugMode == 1)
        cout << endl;
    for (auto &file : errorFiles)
        cout << file << endl;
    // if (debugMode)
    cout << "The total number of optimization failure files is " << errorFiles.size() << endl;

    return;
}