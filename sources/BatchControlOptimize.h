#pragma once
#include "BatchTestutils.h"
#include "ControlOptimize.h"

void AddEntry(string pathRes, double val)
{
    ofstream outfileWrite;
    outfileWrite.open(pathRes,
                      std::ios_base::app);
    outfileWrite << val << endl;
    outfileWrite.close();
}
void AddEntry(string pathRes, double val1, double val2)
{
    ofstream outfileWrite;
    outfileWrite.open(pathRes,
                      std::ios_base::app);
    outfileWrite << val1 << ", " << val2 << endl;
    outfileWrite.close();
}

/**
 * @brief check whether the file in the directory is what we're looking for; this function assumes the possible input are only one of the following: Case<n>.m, Case<n>.txt, Case<n>.txt_RM_BFSResult.txt, Case<n>.txt_RM_GPResult.txt
 *
 * @param file
 * @return int
 *  0 means target source file;
 *  1 means baseline result, MILP
 *  2 means baseline result, MUAi
 *  3 means unrelated;
 */
int TargetFileType(string file)
{
    if (file.find(".txt") == file.length() - 4 && file.find(".txt") != string::npos)
        return 0;
    else if (file.find("GPResult") != string::npos)
        return 1;
    else if (file.find("BFSResult") != string::npos)
        return 2;
    else
        return 3;
}

/**
 * @brief read baseline result file, e.g., Case0.txt_RM_GPResult.txt, or Case0.txt_RM_BFSResult.txt
 *
 * @param path
 * @return pair<double, double> {runTime, obj}
 */
pair<double, double> ReadBaselineZhao20(string path)
{
    fstream file;
    file.open(path, ios::in);
    if (file.is_open())
    {
        string line;
        getline(file, line);
        auto data = ReadLine(line, " ");
        return make_pair(data[0], data[1]);
    }
    else
    {
        CoutError("Path error in ReadBaselineGPR");
        return make_pair(-1, -1);
    }
}
/**
 * @brief
 *
 * @param v1
 * @param v2
 * @return (v1[i]-v2[i])/v2[i]
 */
double RelativeGap(std::vector<double> &v1, std::vector<double> &v2)
{
    double gap = 0;
    for (uint i = 0; i < v1.size(); i++)
    {
        gap += (v1[i] - v2[i]) / v2[i];
    }
    return gap / v1.size();
}
/* return v1/v2 */
double SpeedRatio(std::vector<double> &v1, std::vector<double> &v2)
{
    double ratio = 0;
    for (uint i = 0; i < v1.size(); i++)
    {
        ratio += v1[i] / v2[i];
    }
    return ratio / v1.size();
}
template <typename FactorGraphType>
void BatchOptimize(int Nn = 5)
{
    runMode = "normal";
    const char *pathDataset;
    string str = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N" + to_string(Nn) + "/";
    pathDataset = str.c_str();
    if (debugMode == 1)
        printf("Directory: %s\n", pathDataset);

    vector<vector<double>> objVecAll;
    vector<double> dummy;
    for (int i = 0; i < 3; i++)
        objVecAll.push_back(dummy);
    vector<vector<double>> runTimeAll = objVecAll;
    vector<string> failedFiles;
    int N;
    if (debugMode == 1)
        printf("Directory: %s\n", pathDataset);
    vector<string> errorFiles;
    for (const auto &file : ReadFilesInDirectory(pathDataset))
    {
        // if (debugMode)
        int type = TargetFileType(file);

        string path = pathDataset + file;
        switch (type)
        {
        case 0: // perform optimization
        {
            cout << file << endl;
            relativeErrorTolerance = relativeErrorToleranceInit;
            TaskSet tasks;
            VectorDynamic coeff;
            std::tie(tasks, coeff) = ReadControlCase(path);
            N = tasks.size();
            std::vector<bool> maskForElimination(tasks.size(), false); // TODO: try *2 to ?
            auto start = chrono::high_resolution_clock::now();
            auto res = OptimizeTaskSetIterative<FactorGraphType>(tasks, coeff, maskForElimination);
            auto stop = chrono::high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            double timeTaken = double(duration.count()) / 1e6;
            runTimeAll[0].push_back(timeTaken);
            objVecAll[0].push_back(res.second);

            // check schedulability
            UpdateTaskSetPeriod(tasks, res.first);
            RTA_LL r(tasks);
            if (!r.CheckSchedulability())
            {
                failedFiles.push_back(file);
            }
            break;
        }
        case 1: // read MILP result
        {
            auto res = ReadBaselineZhao20(path);
            runTimeAll[1].push_back(res.first);
            objVecAll[1].push_back(res.second);
            break;
        }
        case 2: // read MUA result
        {
            auto res = ReadBaselineZhao20(path);
            runTimeAll[2].push_back(res.first);
            objVecAll[2].push_back(res.second);
            // record results for plot
            string pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                             batchOptimizeFolder + "/EnergySaveRatio/N" +
                             to_string(N) + ".txt";
            AddEntry(pathRes, objVecAll[0].back() / res.second);
            pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                      batchOptimizeFolder + "/time_task_number.txt";
            AddEntry(pathRes, runTimeAll[0].back(), res.first);
            break;
        }
        default:
        {
            continue;
        }
        }
    }

    cout << Color::blue << endl;
    cout << "Average relative performance gap (NO: MUA) is " << RelativeGap(objVecAll[0], objVecAll[2]) << endl;
    cout << "Speed ratio (NO: MUA) is " << SpeedRatio(runTimeAll[0], runTimeAll[2]) << endl;
    cout << "Average time consumed is " << Average(runTimeAll[0]) << endl;
    cout << Color::def << endl;

    if (printFailureFile)
    {
        cout << endl;
        for (auto &file : failedFiles)
            cout << file << endl;
    }
}