#pragma once
#include "BatchTestutils.h"
#include "Optimize.h"
#include "RTA_DAG.h"

template <class TaskSetType, class Schedul_Analysis>
void BatchOptimize(int Nn = -1)
{
    const char *pathDataset;
    string str = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/N" + to_string(Nn) + "/";
    if (Nn == -1)
        pathDataset = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/";
    else
    {
        pathDataset = str.c_str();
        if (debugMode == 1)
            printf("Directory: %s\n", pathDataset);
    }
    vector<double> energySaveRatioVec;
    vector<double> runTime;
    int N;
    if (debugMode == 1)
        printf("Directory: %s\n", pathDataset);
    vector<string> errorFiles;
    for (const auto &file : ReadFilesInDirectory(pathDataset))
    {
        if (debugMode)
            cout << file << endl;
        string delimiter = "-";
        if (file.substr(0, file.find(delimiter)) == "periodic")
        {
            string path = pathDataset + file;
            TaskSetType tasksN;
            if (TaskSetType::Type() == "normal")
            {
                auto tasks = ReadTaskSet(path, readTaskMode);
                tasksN.UpdateTaskSet(tasks);
                N = tasks.size();
            }

            else if (TaskSetType::Type() == "dag")
            {
                tasksN = ReadDAG_Task(path, readTaskMode);
                N = tasksN.tasks_.size();
            }
            else
            {
                CoutError("Unrecognized TaskSetType!");
            }

            auto start = chrono::high_resolution_clock::now();
            double res = Energy_Opt<TaskSetType, Schedul_Analysis>::OptimizeTaskSet(tasksN);
            // cout << "The energy saving ratio is " << res << endl;
            auto stop = chrono::high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            double timeTaken = double(duration.count()) / 1e6;
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
    cout << Color::blue << endl;
    cout << "Average energy saving ratio is " << avEnergy << endl;
    cout << "Average time consumed is " << aveTime << endl;
    cout << "The number of tasksets under analyzation is " << energySaveRatioVec.size() << endl;

    ofstream outfileWrite;
    string pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                     batchOptimizeFolder + "/EnergySaveRatio/N" +
                     to_string(N) + ".txt";
    outfileWrite.open(pathRes,
                      std::ios_base::app);
    outfileWrite << avEnergy << endl;
    outfileWrite.close();

    if (printFailureFile)
    {
        cout << endl;
        for (auto &file : errorFiles)
            cout << file << endl;
    }
    // if (debugMode)
    cout << "The total number of optimization failure files is " << errorFiles.size() << endl;
    cout << Color::def << endl;
    return;
}