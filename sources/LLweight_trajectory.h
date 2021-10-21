#pragma once
#include "BatchTestutils.h"

void WriteWeightTrajectory(string path)
{

    // configure parameters
    exactJacobian = 1;

    const char *pathDataset = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number";
    vector<double> energySaveRatioVec;
    vector<double> runTime;

    vector<double> errorWeight;
    ofstream outfileWrite;
    outfileWrite.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/weight_task_number.txt", ios::out | ios::trunc);

    TaskSet taskSet1 = ReadTaskSet(path, readTaskMode);

    for (double currWeight = weightDrawBegin; currWeight < weightDrawEnd; currWeight *= 10)
    {
        weightLogBarrier = 1 / currWeight * -1;
        // weightEnergy = currWeight;
        auto start = chrono::high_resolution_clock::now();
        double res = Energy_Opt<RTA_LL>::OptimizeTaskSet(taskSet1);
        auto stop = chrono::high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        double timeTaken = double(duration.count()) / 1e6;

        energySaveRatioVec.push_back(res);
        runTime.push_back(timeTaken);
        outfileWrite << energySaveRatioVec.back() << endl;

        if (res == -1 || res > 1)
        {
            errorWeight.push_back(currWeight);
        }
    }
    outfileWrite.close();

    cout << "The number of tasksets under analyzation is " << energySaveRatioVec.size() << endl;

    if (debugMode == 1)
        cout << endl;
    for (auto &file : errorWeight)
        cout << file << endl;

    cout << "The total number of optimization failure files is " << errorWeight.size() << endl;

    return;
}