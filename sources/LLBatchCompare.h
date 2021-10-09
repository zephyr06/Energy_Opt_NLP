
#pragma once
#include "BatchTestutils.h"

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
            double res = Energy_Opt<RTA_LL>::OptimizeTaskSet(taskSet1);
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
    cout << Color::blue << endl;
    cout << "Average energy optimization objective (NLP: MUA) ratio is " << avEnergy << endl;
    cout << "The worst value is " << worstValue << endl;
    cout << "The worst file is " << worstFile << endl;
    cout << "Average time consumed ratio (NLP: MUA) is " << aveTime << endl;
    cout << "The number of tasksets under analyzation is " << energySaveRatioVec.size() << endl;
    cout << Color::def << endl;

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
