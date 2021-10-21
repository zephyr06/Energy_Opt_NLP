#pragma once
#include "BatchTestutils.h"

void WriteWeightTrajectory()
{

    // configure parameters
    exactJacobian = 1;

    const char *pathDataset = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number";
    vector<double> energySaveRatioVec;

    vector<string> errorFiles;
    vector<double> errorWeight;
    ofstream outfileWrite;
    outfileWrite.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/weight_task_number.txt", ios::out | ios::trunc);

    for (double currWeight = weightDrawBegin; currWeight < weightDrawEnd; currWeight *= 10)
    {
        weightLogBarrier = 1 / currWeight * -1;
        energySaveRatioVec.clear();
        // weightEnergy = currWeight;
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
                auto stop = chrono::high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                double timeTaken = double(duration.count()) / 1e6;
                if (isinf(res))
                    int a = 1;
                energySaveRatioVec.push_back(res);
                // outfileWrite << energySaveRatioVec.back() << endl;
                if (res == -1 || res > 1)
                {
                    errorFiles.push_back(file);
                }
            }
        }
        double avEnergy = -1;
        double aveTime = -1;
        if (energySaveRatioVec.size() != 0)
        {
            avEnergy = Average(energySaveRatioVec);
        }
        outfileWrite << avEnergy << endl;
    }
    outfileWrite.close();

    // if (debugMode == 1)
    //     cout << endl;
    for (auto &file : errorFiles)
        cout << file << endl;

    cout << "The total number of optimization failure files is " << errorWeight.size() << endl;

    return;
}