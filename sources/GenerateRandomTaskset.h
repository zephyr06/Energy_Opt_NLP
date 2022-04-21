#include <time.h>
#include <cmath>

#include <boost/program_options/options_description.hpp>

#include "Tasks.h"
#include "DAG_Task.h"
#include "Parameters.h"
#include "FrequencyModel.h"
namespace rt_num_opt
{

    namespace po = boost::program_options;

    std::vector<double> Uunifast(int N, double utilAll)
    {
        double sumU = utilAll;
        std::vector<double> utilVec(N, 0);

        double nextU;
        for (int i = 1; i < N; i++)
        {

            nextU = sumU * pow(double(rand()) / RAND_MAX, 1.0 / (N - 1));
            utilVec[i - 1] = sumU - nextU;
            sumU = nextU;
        }
        utilVec[N - 1] = nextU;
        return utilVec;
    }

    TaskSet GenerateTaskSet(int N, double totalUtilization,
                            int numberOfProcessor, int periodMin,
                            int periodMax, int deadlineType = 0)
    {
        std::vector<double> utilVec = Uunifast(N, totalUtilization);
        TaskSet tasks;
        // int periodMaxRatio = periodMax / periodMin;

        for (int i = 0; i < N; i++)
        {

            // int periodCurr = (1 + rand() % periodMaxRatio) * periodMin;
            int periodCurr = RandRange(periodMin, periodMax);
            double deadline = periodCurr;
            if (deadlineType == 1)
                deadline = RandRange(ceil(periodCurr * utilVec[i]), periodCurr);
            double randomRatio = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            Task task(0, periodCurr,
                      ceil(periodCurr * utilVec[i]) * 0.1 * randomRatio,
                      ceil(periodCurr * utilVec[i]),
                      deadline, i,
                      rand() % numberOfProcessor);
            tasks.push_back(task);
        }
        return tasks;
    }

    void WriteTaskSets(std::ofstream &file, TaskSet &tasks)
    {
        int N = tasks.size();
        file << "JobID,Offset,Period,Overhead,ExecutionTime,DeadLine,processorId\n";
        for (int i = 0; i < N; i++)
        {
            file << tasks[i].id << "," << tasks[i].offset << ","
                 << tasks[i].period << "," << tasks[i].overhead << ","
                 << tasks[i].executionTime << "," << tasks[i].deadline
                 << "," << tasks[i].processorId << "\n";
        }
        WriteFrequencyModelRatio(file);
    }

    DAG_Model GenerateDAG(int N, double totalUtilization,
                          int numberOfProcessor, int periodMin,
                          int periodMax, int deadlineType = 0)
    {

        TaskSet tasks = GenerateTaskSet(N, totalUtilization,
                                        numberOfProcessor, periodMin, periodMax, deadlineType);

        DAG_Model dagModel(tasks);
        // add edges randomly
        for (int i = 0; i < N; i++)
        {
            for (int j = i + 1; j < N; j++)
            {
                if (double(rand()) / RAND_MAX < parallelFactor)
                {
                    dagModel.AddEdge(i, j);
                }
            }
        }
        return dagModel;
    }

    void WriteDAGMelani(std::ofstream &file, std::vector<DAG_Model> &tasks, std::vector<double> &weightVec)
    {
        int NN = tasks.size();
        file << "JobID,Offset,Period,Overhead,ExecutionTime,DeadLine,processorId,volume,longestPath,weightInControl\n";
        for (int i = 0; i < NN; i++)
        {
            file << i << "," << tasks[i].tasks_[0].offset << ","
                 << tasks[i].tasks_[0].period << "," << tasks[i].tasks_[0].overhead << ","
                 << tasks[i].tasks_[0].executionTime << ","
                 << tasks[i].tasks_[0].deadline << ","
                 << tasks[i].tasks_[0].processorId << ","
                 << tasks[i].Volume() << ","
                 << tasks[i].CriticalPath() << ","
                 << weightVec[i]
                 << "\n";
        }
        WriteFrequencyModelRatio(file);
    }
} // namespace rt_num_opt