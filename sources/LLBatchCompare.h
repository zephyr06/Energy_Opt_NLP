
#pragma once
#include "BatchTestutils.h"
#include "FactorGraphEnergyLL.h"
#include "EnergyOptimize.h"

namespace rt_num_opt
{

    void BatchCompare(int N = -1)
    {
        const char *pathDataset = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number";
        vector<double> energySaveRatioVec;
        vector<double> runTimeW, runTimeZ;

        vector<string> errorFiles;
        runMode = "compare";
        string worstFile = "";
        double worstValue = 0.0;
        for (const auto &file : ReadFilesInDirectory(pathDataset))
        {
            // if (debugMode)
            cout << file << endl;
            string delimiter = "-";

            if (file.substr(0, file.find(delimiter)) == "periodic")
            {
                string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/" + file;
                auto taskSet1 = ReadTaskSet(path, readTaskMode);
                TaskSetNormal tasksN(taskSet1);
                auto start = chrono::high_resolution_clock::now();
                double res;
                if (LLCompareWithGeneralizedElimination)
                {

                    res = EnergyOptimize::OptimizeTaskSetIterative<FactorGraphEnergyLL>(taskSet1).second;
                }
                else
                {
                    res = Energy_Opt<TaskSetNormal, RTA_LL>::OptimizeTaskSet(tasksN);
                }

                // cout << "The energy saving ratio is " << res << endl;
                auto stop = chrono::high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                double timeTaken = double(duration.count()) / 1e6;

                auto baselineResult = ReadBaselineResult(path, taskSet1.size());

                if (res >= 0 && res <= 1)
                {
                    energySaveRatioVec.push_back(res / (baselineResult.second));
                    if (energySaveRatioVec.back() > worstValue)
                    {
                        worstValue = energySaveRatioVec.back();
                        worstFile = path;
                    }
                    runTimeW.push_back(timeTaken);
                    runTimeZ.push_back(baselineResult.first);

                    if (debugMode == 3)
                        cout << "One compare: " << res / (baselineResult.second) << endl;
                    ofstream outfileWrite;
                    outfileWrite.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" + batchOptimizeFolder + "/EnergySaveRatio/N" +
                                          to_string(taskSet1.size()) + ".txt",
                                      std::ios_base::app);
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

        cout << Color::blue << endl;
        cout << "Average energy optimization objective (NLP: SA) ratio is " << avEnergy << endl;
        cout << "The worst value is " << worstValue << endl;
        cout << "The worst file is " << worstFile << endl;
        cout << "Average time consumed ratio (NLP: SA) is " << aveTime << endl;
        cout << "The number of tasksets under analyzation is " << energySaveRatioVec.size() << endl;
        cout << Color::def << endl;

        ofstream outfile2;
        outfile2.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" + batchOptimizeFolder + "/time_task_number.txt", std::ios_base::app);
        outfile2 << Average(runTimeW) << ", " << Average(runTimeZ) << endl;
        if (debugMode == 1)
            cout << endl;
        for (auto &file : errorFiles)
            cout << file << endl;
        // if (debugMode)
        cout << "The total number of optimization failure files is " << errorFiles.size() << endl;

        return;
    }
} // namespace rt_num_opt