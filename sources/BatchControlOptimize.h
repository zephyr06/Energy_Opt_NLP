#pragma once
#include "sources/BatchTestutils.h"
#include "sources/ControlOptimization/ControlOptimize.h"
#include "sources/ControlOptimization/ControlIfoptSpec.h"
namespace rt_num_opt
{
    using namespace ControlOptimize;

    void AddEntry(std::string pathRes, double val1, double val2)
    {
        std::ofstream outfileWrite;
        outfileWrite.open(pathRes,
                          std::ios_base::app);
        outfileWrite << val1 << ", " << val2 << std::endl;
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
    int TargetFileType(std::string file)
    {
        if (file.find(".txt") == file.length() - 4 && file.find(".txt") != std::string::npos)
            return 0;
        else if (file.find("GPResult") != std::string::npos)
            return 1;
        else if (file.find("BFSResult") != std::string::npos)
            return 2;
        else
            return 3;
    }
    /* Extract case ID, input requirements are the same as above*/
    int ExtractCaseID(std::string file)
    {
        int id;
        std::istringstream(SplitStringMy(file, ".")[0].substr(4)) >> id;
        return id;
    }

    /**
     * @brief read baseline result file, e.g., Case0.txt_RM_GPResult.txt, or Case0.txt_RM_BFSResult.txt
     *
     * @param directory absolute folder directory
     * @param path name of target file
     * @return pair<double, double> {runTime, obj}
     */
    std::pair<double, double> ReadBaselineZhao20(std::string directory, std::string path)
    {
        std::fstream file;
        file.open(directory + "/" + path, std::ios::in);
        if (file.is_open())
        {
            std::string line;
            getline(file, line);
            auto data = ReadLine(line, " ");
            TaskSet tasks;
            VectorDynamic coeff;
            std::tie(tasks, coeff) = ReadControlCase(directory + "/" + "Case" + std::to_string(ExtractCaseID(path)) + ".txt");
            double initialError = FactorGraphInManifold::RealObj(tasks, coeff);
            VectorDynamic periods;
            if (TargetFileType(path) == 1) // GP
            {
                std::vector<double>::const_iterator first = data.begin() + 3;
                std::vector<double>::const_iterator last = data.end();
                std::vector<double> newVec(first, last);
                for (auto &x : newVec)
                {
                    x = round(x);
                }
                periods = Vector2Eigen(newVec);
            }
            else if (TargetFileType(path) == 2) // BFS, or MUA
            {
                getline(file, line);
                auto data = ReadLine(line, " ");
                std::vector<double>::const_iterator first = data.begin() + 1;
                std::vector<double>::const_iterator last = data.end() - 1;
                std::vector<double> newVec(first, last);
                periods = Vector2Eigen(newVec);
                for (auto &x : newVec)
                {
                    x = round(x);
                }
                periods = Vector2Eigen(newVec);
            }
            else
            {
                CoutError("Unrecognized file type in ReadBaselineZhao20");
            }

            UpdateTaskSetPeriod(tasks, periods);
            tasks = Reorder(tasks, "RM");
            RTA_LL r(tasks);
            if (r.CheckSchedulability() || TargetFileType(path) == 2)
            {
                return std::make_pair(data[0], data[1]);
            }
            else // return initial estimate
            {
                if (TargetFileType(path) == 2)
                    CoutWarning("Find one unschedulable baseline assignment: " + path);

                return std::make_pair(data[0], initialError);
            }
        }
        else
        {
            CoutError("Path error in ReadBaselineGPR");
            return std::make_pair(-1, -1);
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
        std::string str = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N" + std::to_string(Nn) + "/";
        pathDataset = str.c_str();
        if (debugMode == 1)
            printf("Directory: %s\n", pathDataset);

        std::vector<std::vector<double>> objVecAll;
        std::vector<double> dummy;
        for (int i = 0; i < 3; i++)
            objVecAll.push_back(dummy);
        std::vector<std::vector<double>> runTimeAll = objVecAll;
        std::vector<std::string> failedFiles;
        int N;
        if (debugMode == 1)
            printf("Directory: %s\n", pathDataset);
        std::vector<std::string> errorFiles;
        std::vector<double> rtaCallTime;
        double worstObjRatio = -100;
        std::string worstFile = "";
        for (const auto &file : ReadFilesInDirectory(pathDataset))
        {
            // if (debugMode)
            int type = TargetFileType(file);

            std::string path = pathDataset + file;
            switch (type)
            {
            case 0: // perform optimization
            {
                std::cout << file << std::endl;
                TaskSet tasks;
                VectorDynamic coeff;
                std::tie(tasks, coeff) = ReadControlCase(path);
                N = tasks.size();
                std::vector<bool> maskForElimination(tasks.size(), false); // TODO: try *2 to ?
                ResetCallingTimes();
                auto start = std::chrono::high_resolution_clock::now();
                std::pair<VectorDynamic, double> res;
                if (optimizerType <= 4)
                    res = OptimizeTaskSetIterative<FactorGraphType>(tasks, coeff, maskForElimination);
                else if (optimizerType == 6)
                {
                    TaskSetNormal tasksN(tasks);
                    res = OptimizeControlIfopt<TaskSetNormal, RTA_LL>(tasksN, coeff);
                }
                auto stop = std::chrono::high_resolution_clock::now();
                size_t rtaCall = ReadRTAControl();
                rtaCallTime.push_back(rtaCall / double(N));
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
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
                auto res = ReadBaselineZhao20(pathDataset, file);
                runTimeAll[1].push_back(res.first);
                objVecAll[1].push_back(res.second);
                break;
            }
            case 2: // read MUA result
            {
                auto res = ReadBaselineZhao20(pathDataset, file);
                runTimeAll[2].push_back(res.first);
                objVecAll[2].push_back(res.second);
                // record results for plot
                std::string pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                                      batchOptimizeFolder + "/EnergySaveRatio/N" +
                                      std::to_string(N) + ".txt";
                AddEntry(pathRes, objVecAll[0].back() / res.second);
                double relativeGapCurr = (objVecAll[0].back() - objVecAll[2].back()) / objVecAll[2].back();
                if (relativeGapCurr > worstObjRatio)
                {
                    worstObjRatio = relativeGapCurr;
                    worstFile = file;
                }
                break;
            }
            default:
            {
                continue;
            }
            }
        }
        std::string pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                              batchOptimizeFolder + "/time_task_number.txt";
        AddEntry(pathRes, Average(runTimeAll[0]), Average(runTimeAll[2]));

        std::cout << Color::blue << std::endl;
        std::cout << "Average relative performance gap (NO: MUA) is " << RelativeGap(objVecAll[0], objVecAll[2]) << std::endl;
        // try
        // {
        //     std::cout << "Average relative performance gap (NO: MIGP) is " << RelativeGap(objVecAll[0], objVecAll[1]) << std::endl;
        // }
        // catch (...)
        // {
        //     std::cout << "No MILP information found" << std::endl;
        // }
        std::cout << "Speed ratio (NO: MUA) is " << SpeedRatio(runTimeAll[0], runTimeAll[2]) << std::endl;
        std::cout << "Average time consumed is " << Average(runTimeAll[0]) << std::endl;
        std::cout << Color::def << std::endl;

        std::cout << "Worst relative gap is " << worstObjRatio << std::endl;
        std::cout << "Worst file is " << worstFile << std::endl;

        // used for ControlPerformance's speed figure
        std::ofstream outfileWrite;
        outfileWrite.open("/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" + batchOptimizeFolder + "/Time/N" +
                              std::to_string(Nn) + ".txt",
                          std::ios_base::app);
        outfileWrite << Average(runTimeAll[0]) << std::endl;
        outfileWrite.close();

        // record RTA calling times
        double aveRTA = -1;
        aveRTA = Average(rtaCallTime);
        pathRes = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +
                  batchOptimizeFolder + "/RTACalling/N" +
                  std::to_string(Nn) + ".txt";
        AddEntry(pathRes, aveRTA);
        std::cout << "Average rta calling times is " << aveRTA << std::endl;
        if (printFailureFile)
        {
            std::cout << std::endl;
            for (auto &file : failedFiles)
                std::cout << file << std::endl;
        }
    }
} // namespace rt_num_opt